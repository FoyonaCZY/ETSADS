//! The app's look: European traffic-sign colours (RAL 5017, 6024, 1023, 3020)
//! on asphalt. The one loud element is the driving state, drawn as a road
//! sign; everything around it stays flat and quiet.
use eframe::egui::{
    self, Color32, CornerRadius, FontFamily, FontId, Pos2, Rect, RichText, Sense, Stroke,
    StrokeKind, Vec2,
};

pub const ASPHALT: Color32 = Color32::from_rgb(32, 36, 40);
pub const ASPHALT_DEEP: Color32 = Color32::from_rgb(24, 27, 30);
pub const RAISED: Color32 = Color32::from_rgb(42, 47, 52);
pub const LINE: Color32 = Color32::from_rgb(58, 64, 70);
/// Road-marking white, the text colour.
pub const MARKING: Color32 = Color32::from_rgb(240, 241, 236);
pub const MUTED: Color32 = Color32::from_rgb(164, 172, 179);
pub const SIGN_GREY: Color32 = Color32::from_rgb(66, 73, 80);
pub const SIGN_BLUE: Color32 = Color32::from_rgb(0, 91, 140);
pub const SIGN_GREEN: Color32 = Color32::from_rgb(38, 118, 62);
pub const SIGN_YELLOW: Color32 = Color32::from_rgb(247, 181, 0);
pub const SIGN_RED: Color32 = Color32::from_rgb(196, 22, 28);
pub const INK: Color32 = Color32::from_rgb(20, 22, 24);
/// The sign colours lightened for text on asphalt.
pub const TEXT_GREEN: Color32 = Color32::from_rgb(104, 208, 136);
pub const TEXT_RED: Color32 = Color32::from_rgb(255, 118, 108);

pub fn bold(size: f32) -> FontId {
    FontId::new(size, FontFamily::Name("bold".into()))
}
/// Sign lettering, for numbers.
pub fn digits(size: f32) -> FontId {
    FontId::new(size, FontFamily::Name("digits".into()))
}
pub fn apply(ctx: &egui::Context) {
    ctx.set_theme(egui::Theme::Dark);
    // The redistributable fonts keep Chinese labels readable even when the OS
    // has no Chinese language pack. Never bundle the system's fonts.
    let regular = std::env::var_os("ETSADS_FONT")
        .and_then(|path| std::fs::read(path).ok())
        .map(egui::FontData::from_owned)
        .unwrap_or_else(|| {
            egui::FontData::from_static(include_bytes!(
                "../../../vendor/fonts/NotoSansSC-Regular.otf"
            ))
        });
    let mut fonts = egui::FontDefinitions::default();
    fonts.font_data.insert("cjk".into(), regular.into());
    fonts.font_data.insert(
        "cjk-bold".into(),
        egui::FontData::from_static(include_bytes!("../../../vendor/fonts/NotoSansSC-Bold.otf"))
            .into(),
    );
    fonts.font_data.insert(
        "sign".into(),
        egui::FontData::from_static(include_bytes!(
            "../../../vendor/fonts/BarlowSemiCondensed-SemiBold.ttf"
        ))
        .into(),
    );
    let symbols = ["NotoEmoji-Regular", "emoji-icon-font"];
    let family = |first: &[&str]| -> Vec<String> {
        first
            .iter()
            .chain(symbols.iter())
            .map(|s| (*s).to_owned())
            .collect()
    };
    fonts
        .families
        .insert(FontFamily::Proportional, family(&["cjk"]));
    fonts
        .families
        .insert(FontFamily::Name("bold".into()), family(&["cjk-bold"]));
    fonts.families.insert(
        FontFamily::Name("digits".into()),
        family(&["sign", "cjk-bold"]),
    );
    fonts
        .families
        .entry(FontFamily::Monospace)
        .or_default()
        .push("cjk".into());
    ctx.set_fonts(fonts);

    let mut visuals = egui::Visuals::dark();
    visuals.panel_fill = ASPHALT;
    visuals.window_fill = RAISED;
    visuals.extreme_bg_color = ASPHALT_DEEP;
    visuals.faint_bg_color = RAISED;
    visuals.code_bg_color = ASPHALT_DEEP;
    visuals.selection.bg_fill = SIGN_BLUE;
    visuals.selection.stroke = Stroke::new(1.5_f32, MARKING);
    visuals.warn_fg_color = SIGN_YELLOW;
    visuals.error_fg_color = TEXT_RED;
    visuals.slider_trailing_fill = true;
    let radius = CornerRadius::same(5);
    let w = &mut visuals.widgets;
    w.noninteractive.bg_fill = RAISED;
    w.noninteractive.weak_bg_fill = RAISED;
    w.noninteractive.bg_stroke = Stroke::new(1.0_f32, LINE);
    w.noninteractive.fg_stroke = Stroke::new(1.0_f32, Color32::from_rgb(218, 221, 220));
    w.noninteractive.corner_radius = radius;
    for (state, fill) in [
        (&mut w.inactive, Color32::from_rgb(58, 64, 70)),
        (&mut w.hovered, Color32::from_rgb(72, 79, 86)),
        (&mut w.active, Color32::from_rgb(84, 92, 100)),
        (&mut w.open, Color32::from_rgb(58, 64, 70)),
    ] {
        state.bg_fill = fill;
        state.weak_bg_fill = fill;
        state.bg_stroke = Stroke::NONE;
        state.fg_stroke = Stroke::new(1.0_f32, MARKING);
        state.corner_radius = radius;
        state.expansion = 0.0;
    }
    ctx.set_visuals_of(egui::Theme::Dark, visuals);

    ctx.all_styles_mut(|style| {
        style.spacing.item_spacing = Vec2::new(10.0, 8.0);
        style.spacing.button_padding = Vec2::new(14.0, 7.0);
        style.spacing.slider_width = 240.0;
        style.spacing.interact_size.y = 26.0;
        for (name, font) in [
            (egui::TextStyle::Body, FontId::proportional(15.0)),
            (egui::TextStyle::Button, FontId::proportional(15.0)),
            (egui::TextStyle::Small, FontId::proportional(13.0)),
            (egui::TextStyle::Heading, bold(24.0)),
        ] {
            style.text_styles.insert(name, font);
        }
    });
}
/// A section title on the asphalt; sections are set apart by space, not boxes.
pub fn section(ui: &mut egui::Ui, title: &str) {
    ui.label(RichText::new(title).font(bold(16.0)).color(MARKING));
}
pub fn note(text: impl Into<String>) -> RichText {
    RichText::new(text.into()).size(13.0).color(MUTED)
}
/// A setting: a switch, its name and what it does.
pub fn switch_row(ui: &mut egui::Ui, on: &mut bool, title: &str, what: &str) {
    ui.horizontal_top(|ui| {
        let (rect, mut response) = ui.allocate_exact_size(Vec2::new(38.0, 22.0), Sense::click());
        let name_clicked = ui
            .vertical(|ui| {
                let name = ui.add(egui::Label::new(title).sense(Sense::click()));
                if !what.is_empty() {
                    ui.add_space(-5.0);
                    ui.label(note(what));
                }
                name.clicked()
            })
            .inner;
        if response.clicked() || name_clicked {
            *on = !*on;
            response.mark_changed();
        }
        let t = ui.ctx().animate_bool(response.id, *on);
        let painter = ui.painter();
        painter.rect_filled(rect, 11.0, LINE.lerp_to_gamma(SIGN_BLUE, t));
        if response.has_focus() {
            painter.rect_stroke(
                rect,
                11.0,
                Stroke::new(1.5_f32, MARKING),
                StrokeKind::Outside,
            );
        }
        let x = egui::lerp(rect.left() + 11.0..=rect.right() - 11.0, t);
        painter.circle_filled(Pos2::new(x, rect.center().y), 8.0, MARKING);
    });
}
/// One row of the side navigation; the current page carries a lane marking.
pub fn nav_item(ui: &mut egui::Ui, current: bool, name: &str, font: FontId) -> bool {
    let (rect, response) =
        ui.allocate_exact_size(Vec2::new(ui.available_width(), 34.0), Sense::click());
    let painter = ui.painter();
    if current {
        painter.rect_filled(rect, 4.0, RAISED);
        painter.rect_filled(
            Rect::from_min_size(rect.min + Vec2::new(0.0, 7.0), Vec2::new(3.0, 20.0)),
            1.5,
            MARKING,
        );
    } else if response.hovered() {
        painter.rect_filled(rect, 4.0, ASPHALT);
    }
    if response.has_focus() {
        painter.rect_stroke(rect, 4.0, Stroke::new(1.5_f32, MARKING), StrokeKind::Inside);
    }
    painter.text(
        Pos2::new(rect.left() + 14.0, rect.center().y),
        egui::Align2::LEFT_CENTER,
        name,
        font,
        if current { MARKING } else { MUTED },
    );
    response.clicked()
}
/// A sign plate: solid colour with the inset line real signs carry.
pub fn plate<R>(
    ui: &mut egui::Ui,
    fill: Color32,
    ink: Color32,
    content: impl FnOnce(&mut egui::Ui) -> R,
) -> R {
    let shown = egui::Frame::new()
        .fill(fill)
        .corner_radius(10)
        .inner_margin(egui::Margin::symmetric(26, 22))
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            content(ui)
        });
    ui.painter().rect_stroke(
        shown.response.rect.shrink(5.0),
        6.0,
        Stroke::new(2.0_f32, ink),
        StrokeKind::Inside,
    );
    shown.inner
}
/// A round speed-limit sign; `None` is a blank grey one.
pub fn roundel(ui: &mut egui::Ui, d: f32, limit: Option<f64>) {
    let (rect, _) = ui.allocate_exact_size(Vec2::splat(d), Sense::hover());
    let c = rect.center();
    let painter = ui.painter();
    painter.circle_filled(c, d / 2.0, MARKING);
    let (ring, face, ink) = match limit {
        Some(_) => (SIGN_RED, MARKING, INK),
        None => (SIGN_GREY, Color32::from_rgb(92, 100, 108), MARKING),
    };
    painter.circle_filled(c, d / 2.0 - 2.0, ring);
    painter.circle_filled(c, d / 2.0 - d * 0.14, face);
    let text = limit.map_or("—".to_owned(), |l| format!("{l:.0}"));
    let size = if text.chars().count() > 2 {
        d * 0.34
    } else {
        d * 0.42
    };
    painter.text(c, egui::Align2::CENTER_CENTER, text, digits(size), ink);
}
