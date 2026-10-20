//! In-game heads-up display. The host lays out and rasterizes a small,
//! premultiplied RGBA panel; `etsads_gfx.dll` only blends that image over the
//! backbuffer. A model older than `MODEL_STALE` is never published, so a hung
//! or exited host makes the HUD disappear instead of freezing an old state.
use ab_glyph::{Font, FontArc, PxScale, ScaleFont, point};
use etsads_msgs::protocol::{
    HUD_ANCHOR_TOP_LEFT, HUD_MAPPING, HUD_MAX_HEIGHT, HUD_MAX_WIDTH, HUD_STATUS_MAPPING, HudImage,
    HudStatus, SCHEMA_VERSION,
};
use etsads_shm::{SharedCell, SharedRing, SingleWriterGuard, qpc, zeroed_box};
use std::{
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
    },
    time::{Duration, Instant},
};

const MODEL_STALE: Duration = Duration::from_millis(400);
const PERIOD: Duration = Duration::from_millis(100);
/// Layout units are scaled by backbuffer height / 940, i.e. 1.15x at 1080p,
/// which keeps the smallest hint line legible over a moving scene.
const DESIGN_HEIGHT: f32 = 940.0;
const WIDTH: f32 = 440.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tone {
    /// Not available: neutral grey.
    Idle,
    /// Ready to engage.
    Ready,
    /// Engaged and driving.
    Active,
    /// Driver attention required soon.
    Warning,
    /// Safety braking or a fault.
    Alert,
}
#[derive(Clone, Debug, PartialEq)]
pub struct HudModel {
    pub tone: Tone,
    pub title: String,
    /// Right-aligned on the title row, e.g. remaining time.
    pub badge: String,
    pub speed_kmh: Option<f64>,
    pub target_kmh: Option<f64>,
    pub limit_kmh: Option<f64>,
    /// Upper end of the speed bar.
    pub scale_kmh: f64,
    pub detail: Vec<String>,
    pub hint: String,
    /// Blink the accent bar (takeover request).
    pub blink: bool,
}
impl Default for HudModel {
    fn default() -> Self {
        Self {
            tone: Tone::Idle,
            title: String::new(),
            badge: String::new(),
            speed_kmh: None,
            target_kmh: None,
            limit_kmh: None,
            scale_kmh: 90.0,
            detail: Vec::new(),
            hint: String::new(),
            blink: false,
        }
    }
}

fn tone_color(tone: Tone) -> [u8; 3] {
    match tone {
        Tone::Idle => [150, 163, 178],
        Tone::Ready => [86, 190, 255],
        Tone::Active => [64, 221, 160],
        Tone::Warning => [255, 186, 72],
        Tone::Alert => [255, 84, 84],
    }
}

pub fn font() -> FontArc {
    if let Some(font) = std::env::var_os("ETSADS_FONT")
        .and_then(|p| std::fs::read(p).ok())
        .and_then(|bytes| FontArc::try_from_vec(bytes).ok())
    {
        return font;
    }
    FontArc::try_from_slice(include_bytes!(
        "../../../vendor/fonts/NotoSansSC-Regular.otf"
    ))
    .expect("bundled font parses")
}

/// Premultiplied RGBA8 raster, row-major.
pub struct Canvas {
    pub width: usize,
    pub height: usize,
    pub pixels: Vec<u8>,
}
impl Canvas {
    fn new(width: usize, height: usize) -> Self {
        Self {
            width,
            height,
            pixels: vec![0; width * height * 4],
        }
    }
    /// Source-over blend of a straight-alpha colour with coverage `a`.
    fn blend(&mut self, x: i64, y: i64, rgb: [u8; 3], a: f32) {
        if x < 0 || y < 0 || x as usize >= self.width || y as usize >= self.height || a <= 0.0 {
            return;
        }
        let a = a.min(1.0);
        let i = (y as usize * self.width + x as usize) * 4;
        let keep = 1.0 - a;
        for (k, channel) in rgb.into_iter().enumerate() {
            let v = channel as f32 * a + self.pixels[i + k] as f32 * keep;
            self.pixels[i + k] = v.round().min(255.0) as u8;
        }
        let v = 255.0 * a + self.pixels[i + 3] as f32 * keep;
        self.pixels[i + 3] = v.round().min(255.0) as u8;
    }
    /// Anti-aliased rounded rectangle.
    #[allow(clippy::too_many_arguments)]
    fn round_rect(&mut self, x: f32, y: f32, w: f32, h: f32, r: f32, rgb: [u8; 3], alpha: f32) {
        let r = r.min(w / 2.0).min(h / 2.0).max(0.0);
        for py in y.floor() as i64..(y + h).ceil() as i64 {
            for px in x.floor() as i64..(x + w).ceil() as i64 {
                let (cx, cy) = (px as f32 + 0.5, py as f32 + 0.5);
                // Signed distance to the rounded box.
                let qx = (cx - (x + w / 2.0)).abs() - (w / 2.0 - r);
                let qy = (cy - (y + h / 2.0)).abs() - (h / 2.0 - r);
                let outside = qx.max(0.0).hypot(qy.max(0.0)) + qx.max(qy).min(0.0) - r;
                let coverage = (0.5 - outside).clamp(0.0, 1.0);
                self.blend(px, py, rgb, coverage * alpha);
            }
        }
    }
}

pub struct Painter {
    font: FontArc,
    pub scale: f32,
}
impl Painter {
    pub fn new(font: FontArc) -> Self {
        Self { font, scale: 1.0 }
    }
    fn px(&self, v: f32) -> f32 {
        v * self.scale
    }
    fn measure(&self, text: &str, size: f32) -> f32 {
        let font = self.font.as_scaled(PxScale::from(self.px(size)));
        let mut width = 0.0;
        let mut last = None;
        for c in text.chars() {
            let id = font.glyph_id(c);
            if let Some(previous) = last {
                width += font.kern(previous, id);
            }
            width += font.h_advance(id);
            last = Some(id);
        }
        width
    }
    /// Draw `text` with its top-left at (x, y) in design units; returns width.
    fn text(
        &self,
        canvas: &mut Canvas,
        x: f32,
        y: f32,
        text: &str,
        size: f32,
        rgb: [u8; 3],
    ) -> f32 {
        let px_size = self.px(size);
        let font = self.font.as_scaled(PxScale::from(px_size));
        let baseline = self.px(y) + font.ascent();
        let mut caret = self.px(x);
        let mut last = None;
        for c in text.chars() {
            let id = font.glyph_id(c);
            if let Some(previous) = last {
                caret += font.kern(previous, id);
            }
            let glyph = id.with_scale_and_position(px_size, point(caret, baseline));
            caret += font.h_advance(id);
            last = Some(id);
            if let Some(outline) = self.font.outline_glyph(glyph) {
                let bounds = outline.px_bounds();
                outline.draw(|gx, gy, coverage| {
                    canvas.blend(
                        bounds.min.x as i64 + gx as i64,
                        bounds.min.y as i64 + gy as i64,
                        rgb,
                        coverage,
                    );
                });
            }
        }
        (caret - self.px(x)) / self.scale
    }
    /// Longest prefix that fits, with an ellipsis when shortened.
    fn fit(&self, text: &str, size: f32, width: f32) -> String {
        if self.measure(text, size) <= self.px(width) {
            return text.to_owned();
        }
        let mut out: String = text.chars().collect();
        while !out.is_empty() && self.measure(&format!("{out}…"), size) > self.px(width) {
            out.pop();
        }
        format!("{out}…")
    }
    /// Lay out and rasterize the panel. `blink_on` toggles the accent bar.
    pub fn render(&self, model: &HudModel, blink_on: bool) -> Canvas {
        const PAD: f32 = 16.0;
        let inner = WIDTH - 2.0 * PAD - 6.0;
        let detail: Vec<String> = model
            .detail
            .iter()
            .take(3)
            .map(|d| self.fit(d, 15.0, inner))
            .collect();
        let speed_row = model.speed_kmh.is_some();
        let mut height = PAD + 28.0;
        if speed_row {
            height += 52.0 + 14.0;
        }
        height += detail.len() as f32 * 22.0;
        if !model.hint.is_empty() {
            height += 8.0 + 20.0;
        }
        height += PAD;
        let width = (self.px(WIDTH).ceil() as usize).min(HUD_MAX_WIDTH);
        let pixel_height = (self.px(height).ceil() as usize).min(HUD_MAX_HEIGHT);
        let mut canvas = Canvas::new(width, pixel_height);
        let accent = tone_color(model.tone);
        // Panel and accent bar.
        canvas.round_rect(
            0.0,
            0.0,
            width as f32,
            pixel_height as f32,
            self.px(12.0),
            [14, 20, 28],
            0.9,
        );
        if !model.blink || blink_on {
            canvas.round_rect(
                self.px(6.0),
                self.px(10.0),
                self.px(5.0),
                pixel_height as f32 - self.px(20.0),
                self.px(2.5),
                accent,
                1.0,
            );
        }
        let left = PAD + 6.0;
        let mut y = PAD - 2.0;
        self.text(
            &mut canvas,
            left,
            y,
            &self.fit(&model.title, 21.0, inner - 90.0),
            21.0,
            accent,
        );
        if !model.badge.is_empty() {
            let w = self.measure(&model.badge, 15.0) / self.scale;
            self.text(
                &mut canvas,
                WIDTH - PAD - w,
                y + 4.0,
                &model.badge,
                15.0,
                [200, 210, 222],
            );
        }
        y += 28.0;
        if let Some(speed) = model.speed_kmh {
            let digits = format!("{:.0}", speed.abs().max(0.0));
            let w = self.text(&mut canvas, left, y - 6.0, &digits, 46.0, [244, 247, 250]);
            self.text(
                &mut canvas,
                left + w + 6.0,
                y + 22.0,
                "km/h",
                15.0,
                [150, 163, 178],
            );
            // Right column: target and limit.
            let column = left + 170.0;
            let mut parts = Vec::new();
            if let Some(t) = model.target_kmh {
                parts.push(("目标", format!("{t:.0}")));
            }
            parts.push((
                "限速",
                model.limit_kmh.map_or("—".into(), |l| format!("{l:.0}")),
            ));
            let mut x = column;
            for (label, value) in parts {
                self.text(&mut canvas, x, y + 2.0, label, 13.0, [150, 163, 178]);
                self.text(&mut canvas, x, y + 18.0, &value, 24.0, [232, 238, 244]);
                x += 92.0;
            }
            y += 52.0;
            // Speed bar with target tick and limit tick.
            let bar_w = inner;
            let scale = model.scale_kmh.max(10.0);
            let frac = |v: f64| ((v / scale).clamp(0.0, 1.0)) as f32;
            canvas.round_rect(
                self.px(left),
                self.px(y),
                self.px(bar_w),
                self.px(6.0),
                self.px(3.0),
                [44, 58, 74],
                1.0,
            );
            canvas.round_rect(
                self.px(left),
                self.px(y),
                self.px(bar_w * frac(speed.max(0.0))).max(self.px(6.0)),
                self.px(6.0),
                self.px(3.0),
                accent,
                1.0,
            );
            for (value, rgb) in [
                (model.limit_kmh, [255, 255, 255]),
                (model.target_kmh, [255, 214, 120]),
            ] {
                if let Some(v) = value {
                    canvas.round_rect(
                        self.px(left + bar_w * frac(v) - 1.5),
                        self.px(y - 4.0),
                        self.px(3.0),
                        self.px(14.0),
                        self.px(1.5),
                        rgb,
                        1.0,
                    );
                }
            }
            y += 14.0;
        }
        for line in &detail {
            self.text(&mut canvas, left, y, line, 15.0, [206, 214, 224]);
            y += 22.0;
        }
        if !model.hint.is_empty() {
            y += 8.0;
            let hint = self.fit(&model.hint, 13.5, inner);
            self.text(&mut canvas, left, y, &hint, 13.5, [150, 163, 178]);
        }
        canvas
    }
}

/// Publishes the latest model from a background thread at 10 Hz.
pub struct HudPublisher {
    model: Arc<Mutex<Option<(Instant, HudModel)>>>,
    stop: Arc<AtomicBool>,
    worker: Option<std::thread::JoinHandle<()>>,
}
impl HudPublisher {
    /// Fails if another process already owns the HUD.
    pub fn start() -> anyhow::Result<Self> {
        let model = Arc::new(Mutex::new(None::<(Instant, HudModel)>));
        let stop = Arc::new(AtomicBool::new(false));
        let (worker_model, worker_stop) = (model.clone(), stop.clone());
        let (ready_tx, ready_rx) = std::sync::mpsc::sync_channel(1);
        let worker = std::thread::Builder::new()
            .name("etsads-hud".into())
            .spawn(move || {
                // Win32 handles are not Send; own them on this thread only.
                let setup = (|| -> anyhow::Result<_> {
                    let guard = SingleWriterGuard::acquire("ETSADS_Hud_Writer")?;
                    let images = SharedRing::<HudImage, 2>::create(HUD_MAPPING, SCHEMA_VERSION)?;
                    images.recover_after_writer_crash();
                    Ok((guard, images))
                })();
                let (_guard, images) = match setup {
                    Ok(v) => {
                        let _ = ready_tx.send(Ok(()));
                        v
                    }
                    Err(e) => {
                        let _ = ready_tx.send(Err(e));
                        return;
                    }
                };
                let mut painter = Painter::new(font());
                let mut image = zeroed_box::<HudImage>();
                let mut status = None::<SharedCell<HudStatus>>;
                let mut sequence = 0_u64;
                let started = Instant::now();
                while !worker_stop.load(Ordering::Acquire) {
                    if status.is_none() {
                        status = SharedCell::open(HUD_STATUS_MAPPING, SCHEMA_VERSION).ok();
                    }
                    let height = status
                        .as_ref()
                        .and_then(|s| s.latest())
                        .map(|s| s.backbuffer_height)
                        .filter(|h| (480..=4320).contains(h));
                    painter.scale = height.map_or(1.0, |h| h as f32 / DESIGN_HEIGHT);
                    let current = worker_model
                        .lock()
                        .ok()
                        .and_then(|m| m.clone())
                        .filter(|(at, _)| at.elapsed() < MODEL_STALE);
                    sequence += 1;
                    image.sequence = sequence;
                    image.write_qpc = qpc();
                    image.anchor = HUD_ANCHOR_TOP_LEFT;
                    image.offset_px = [24.0 * painter.scale, 120.0 * painter.scale];
                    if let Some((_, model)) = current {
                        let blink = (started.elapsed().as_millis() / 400).is_multiple_of(2);
                        let canvas = painter.render(&model, blink);
                        image.enabled = 1;
                        image.width = canvas.width as u32;
                        image.height = canvas.height as u32;
                        image.pixels[..canvas.pixels.len()].copy_from_slice(&canvas.pixels);
                    } else {
                        image.enabled = 0;
                    }
                    let _ = images.publish(&image);
                    std::thread::sleep(PERIOD);
                }
                image.enabled = 0;
                image.write_qpc = qpc();
                let _ = images.publish(&image);
            })?;
        ready_rx
            .recv()
            .map_err(|_| anyhow::anyhow!("HUD 线程启动失败"))??;
        Ok(Self {
            model,
            stop,
            worker: Some(worker),
        })
    }
    pub fn show(&self, model: HudModel) {
        if let Ok(mut m) = self.model.lock() {
            let previous = m.as_ref().map(|(_, p)| p.tone);
            if previous.is_some_and(|p| p != model.tone) {
                chime(previous, model.tone);
            }
            *m = Some((Instant::now(), model));
        }
    }
    /// Whether the renderer drew one of this host's images within 1 s.
    pub fn visible() -> bool {
        SharedCell::<HudStatus>::open(HUD_STATUS_MAPPING, SCHEMA_VERSION)
            .ok()
            .and_then(|s| s.latest())
            .is_some_and(|s| {
                s.drawn_sequence != 0
                    && etsads_safety::fresh(qpc(), s.present_qpc, etsads_shm::qpc_frequency(), 1000)
            })
    }
}
impl Drop for HudPublisher {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
    }
}

/// Audible cue for transitions the driver must not miss. `MessageBeep` queues
/// the system sound asynchronously and never blocks the caller.
fn chime(previous: Option<Tone>, next: Tone) {
    let sound = match (previous, next) {
        (_, Tone::Active) if previous != Some(Tone::Warning) => 0x0000_0000, // MB_OK
        (_, Tone::Warning | Tone::Alert) => 0x0000_0030,                     // exclamation
        (Some(Tone::Active | Tone::Warning | Tone::Alert), Tone::Idle) => 0x0000_0040, // asterisk
        _ => return,
    };
    #[cfg(windows)]
    // SAFETY: plain Win32 call with a constant sound identifier.
    unsafe {
        windows_sys::Win32::System::Diagnostics::Debug::MessageBeep(sound);
    }
    #[cfg(not(windows))]
    let _ = sound;
}

/// Render representative states over a road-like backdrop for review.
pub fn preview(dir: &std::path::Path) -> anyhow::Result<Vec<std::path::PathBuf>> {
    std::fs::create_dir_all(dir)?;
    let mut painter = Painter::new(font());
    let samples = [
        (
            "standby-unavailable",
            HudModel {
                tone: Tone::Idle,
                title: "待命 · 暂不可用".into(),
                speed_kmh: Some(0.0),
                limit_kmh: Some(50.0),
                scale_kmh: 55.0,
                detail: vec!["请选择前进挡并松开驻车制动".into()],
                hint: "条件满足后按 F8 启用".into(),
                ..Default::default()
            },
        ),
        (
            "standby-ready",
            HudModel {
                tone: Tone::Ready,
                title: "可以启用自动驾驶".into(),
                speed_kmh: Some(32.0),
                target_kmh: Some(50.0),
                limit_kmh: Some(50.0),
                scale_kmh: 55.0,
                detail: vec!["导航路段 1.2 km 可用".into()],
                hint: "按 F8 启用 · 松开方向和油门键".into(),
                ..Default::default()
            },
        ),
        (
            "engaged",
            HudModel {
                tone: Tone::Active,
                title: "自动驾驶中".into(),
                badge: "不限时".into(),
                speed_kmh: Some(48.6),
                target_kmh: Some(50.0),
                limit_kmh: Some(50.0),
                scale_kmh: 55.0,
                detail: vec![
                    "前车 38 m · 同向 46 km/h".into(),
                    "前方路口需人工 · 820 m".into(),
                ],
                hint: "F8 退出 · 方向键 / WASD / 空格 立即接管".into(),
                ..Default::default()
            },
        ),
        (
            "takeover",
            HudModel {
                tone: Tone::Warning,
                title: "请准备接管".into(),
                badge: "4 s".into(),
                speed_kmh: Some(21.0),
                target_kmh: Some(0.0),
                limit_kmh: Some(50.0),
                scale_kmh: 55.0,
                detail: vec!["前方路口、交通灯或通行规则需要手动处理".into()],
                hint: "方向键 / WASD / 空格 接管".into(),
                blink: true,
            },
        ),
        (
            "alert",
            HudModel {
                tone: Tone::Alert,
                title: "安全制动".into(),
                speed_kmh: Some(12.0),
                target_kmh: Some(0.0),
                limit_kmh: Some(50.0),
                scale_kmh: 55.0,
                detail: vec!["独立监督检测到接近风险".into()],
                hint: "随时可接管".into(),
                ..Default::default()
            },
        ),
        (
            "released",
            HudModel {
                tone: Tone::Idle,
                title: "已交还人工驾驶".into(),
                detail: vec!["人工接管（方向键）".into(), "约 5 秒后回到待命".into()],
                ..Default::default()
            },
        ),
    ];
    let mut out = Vec::new();
    for scale in [1.0_f32, 1.333] {
        painter.scale = scale;
        for (name, model) in &samples {
            let canvas = painter.render(model, true);
            let (w, h) = (canvas.width + 80, canvas.height + 80);
            let mut rgb = image::RgbImage::new(w as u32, h as u32);
            for (x, y, p) in rgb.enumerate_pixels_mut() {
                // Sky-to-asphalt gradient roughly matching daytime game colours.
                let t = y as f32 / h as f32;
                let base = [
                    (150.0 - 90.0 * t) as u8,
                    (175.0 - 100.0 * t) as u8,
                    (200.0 - 120.0 * t) as u8,
                ];
                let mut c = base;
                let (cx, cy) = (x as i64 - 40, y as i64 - 40);
                if cx >= 0
                    && cy >= 0
                    && (cx as usize) < canvas.width
                    && (cy as usize) < canvas.height
                {
                    let i = (cy as usize * canvas.width + cx as usize) * 4;
                    let a = canvas.pixels[i + 3] as f32 / 255.0;
                    for k in 0..3 {
                        c[k] = (canvas.pixels[i + k] as f32 + base[k] as f32 * (1.0 - a)).min(255.0)
                            as u8;
                    }
                }
                *p = image::Rgb(c);
            }
            let path = dir.join(format!("hud-{name}-{:.0}p.png", 1080.0 * scale));
            rgb.save(&path)?;
            out.push(path);
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn engaged() -> HudModel {
        HudModel {
            tone: Tone::Active,
            title: "自动驾驶中".into(),
            badge: "不限时".into(),
            speed_kmh: Some(48.6),
            target_kmh: Some(50.0),
            limit_kmh: Some(60.0),
            scale_kmh: 66.0,
            detail: vec!["前车 38 m · 同向 52 km/h".into(), "车道偏差 +0.12 m".into()],
            hint: "F8 退出 · 方向键 / WASD / 空格 立即接管".into(),
            blink: false,
        }
    }
    #[test]
    fn panel_fits_the_protocol_at_every_supported_scale() {
        let mut painter = Painter::new(font());
        for scale in [0.5, 1.0, 1.33, 2.0] {
            painter.scale = scale;
            let canvas = painter.render(&engaged(), true);
            assert!(canvas.width <= HUD_MAX_WIDTH && canvas.height <= HUD_MAX_HEIGHT);
            assert_eq!(canvas.pixels.len(), canvas.width * canvas.height * 4);
            // Premultiplied: no colour channel may exceed its alpha.
            assert!(
                canvas
                    .pixels
                    .as_chunks::<4>()
                    .0
                    .iter()
                    .all(|p| p[0] <= p[3] && p[1] <= p[3] && p[2] <= p[3])
            );
        }
    }
    #[test]
    fn long_reasons_are_shortened_not_overflowing() {
        let painter = Painter::new(font());
        let long =
            "这是一条非常非常长的原因说明，用来确认不会超出面板宽度而被裁掉一半文字".repeat(3);
        let fitted = painter.fit(&long, 15.0, 300.0);
        assert!(fitted.ends_with('…') && painter.measure(&fitted, 15.0) <= 300.0);
        let mut model = engaged();
        model.speed_kmh = None;
        model.detail = vec![long];
        let canvas = painter.render(&model, false);
        assert!(canvas.height < painter.render(&engaged(), false).height);
    }
}
