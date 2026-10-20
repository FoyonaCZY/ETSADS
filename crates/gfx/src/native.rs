//! One Present detour: consume prior staging copy, queue pre-overlay copy,
//! paint validated rectangles, then invoke the game's original Present.
use crate::{hud::Hud, publisher::CapturePublisher, validated_rect};
use etsads_msgs::protocol::{OverlayFrame, SCHEMA_VERSION};
#[cfg(not(test))]
use etsads_msgs::protocol::{HUD_MAPPING, HUD_STATUS_MAPPING, OVERLAY_MAPPING};
use etsads_sdk_sys::{scs_telemetry_init_params_t, scs_telemetry_init_params_v101_t};
use etsads_shm::{qpc, qpc_frequency, SharedCell};
use minhook::MinHook;
use std::{
    ffi::c_void,
    panic::{catch_unwind, AssertUnwindSafe},
    sync::{
        atomic::{AtomicBool, AtomicPtr, AtomicU64, AtomicUsize, Ordering},
        Mutex,
    },
    time::Duration,
};
use windows::{
    core::{w, Interface, HRESULT, PCWSTR},
    Win32::{
        Foundation::{HMODULE, RECT},
        Graphics::{
            Direct3D::{D3D_DRIVER_TYPE_HARDWARE, D3D_FEATURE_LEVEL_11_0},
            Direct3D11::*,
            Dxgi::{
                Common::{DXGI_FORMAT_R8G8B8A8_UNORM, DXGI_MODE_DESC, DXGI_SAMPLE_DESC},
                IDXGISwapChain, DXGI_SWAP_CHAIN_DESC, DXGI_SWAP_EFFECT_DISCARD,
                DXGI_USAGE_RENDER_TARGET_OUTPUT,
            },
        },
        System::LibraryLoader::{
            GetModuleHandleExW, GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS,
            GET_MODULE_HANDLE_EX_FLAG_PIN,
        },
        UI::WindowsAndMessaging::{
            CreateWindowExW, DestroyWindow, WINDOW_EX_STYLE, WS_OVERLAPPEDWINDOW,
        },
    },
};

static TARGET: AtomicPtr<c_void> = AtomicPtr::new(std::ptr::null_mut());
static ORIGINAL: AtomicPtr<c_void> = AtomicPtr::new(std::ptr::null_mut());
static STOPPING: AtomicBool = AtomicBool::new(true);
static ACTIVE: AtomicUsize = AtomicUsize::new(0);
static ERRORS: AtomicU64 = AtomicU64::new(0);
static RENDER: Mutex<Option<RenderState>> = Mutex::new(None);
type Present = unsafe extern "system" fn(*mut c_void, u32, u32) -> HRESULT;

struct Pending {
    texture: ID3D11Texture2D,
    present_qpc: Option<u64>,
}
struct RenderState {
    publisher: CapturePublisher,
    overlay: SharedCell<OverlayFrame>,
    hud: Hud,
    staging: Vec<Pending>,
    description: Option<D3D11_TEXTURE2D_DESC>,
    device_id: usize,
    next: usize,
}
impl RenderState {
    fn new() -> Result<Self, String> {
        #[cfg(not(test))]
        let publisher = CapturePublisher::create().map_err(|e| e.to_string())?;
        #[cfg(test)]
        let publisher = CapturePublisher::create_named(
            &test_name("Control"),
            &test_name("Data_"),
            &test_name("Writer"),
        )
        .map_err(|e| e.to_string())?;
        #[cfg(not(test))]
        let overlay_name = OVERLAY_MAPPING.to_owned();
        #[cfg(test)]
        let overlay_name = test_name("Overlay");
        #[cfg(not(test))]
        let hud = Hud::create(HUD_MAPPING, HUD_STATUS_MAPPING)?;
        #[cfg(test)]
        let hud = Hud::create(&test_name("Hud"), &test_name("HudStatus"))?;
        Ok(Self {
            publisher,
            overlay: SharedCell::create(&overlay_name, SCHEMA_VERSION)
                .map_err(|e| e.to_string())?,
            hud,
            staging: Vec::new(),
            description: None,
            device_id: 0,
            next: 0,
        })
    }
    unsafe fn frame(
        &mut self,
        chain: &IDXGISwapChain,
        present_qpc: u64,
    ) -> windows::core::Result<()> {
        let back: ID3D11Texture2D = unsafe { chain.GetBuffer(0)? };
        let mut desc = D3D11_TEXTURE2D_DESC::default();
        unsafe { back.GetDesc(&mut desc) };
        // Ignore dummy/test/small auxiliary swap chains. No assumptions about
        // the contents of the game's UI or save are needed.
        if desc.Width < 64 || desc.Height < 64 || desc.Width > 16384 || desc.Height > 16384 {
            return Ok(());
        }
        let device: ID3D11Device = unsafe { chain.GetDevice()? };
        let context = unsafe { device.GetImmediateContext()? };
        if self.publisher.has_readers() && desc.SampleDesc.Count == 1 {
            let changed = self.device_id != device.as_raw() as usize
                || self.description.is_none_or(|d| {
                    d.Width != desc.Width || d.Height != desc.Height || d.Format != desc.Format
                });
            if changed {
                self.staging.clear();
                self.next = 0;
                let mut staging_desc = desc;
                staging_desc.Usage = D3D11_USAGE_STAGING;
                staging_desc.BindFlags = 0;
                staging_desc.CPUAccessFlags = D3D11_CPU_ACCESS_READ.0 as u32;
                staging_desc.MiscFlags = 0;
                for _ in 0..3 {
                    let mut texture = None;
                    unsafe { device.CreateTexture2D(&staging_desc, None, Some(&mut texture))? };
                    self.staging.push(Pending {
                        texture: texture.expect("successful texture allocation"),
                        present_qpc: None,
                    });
                }
                self.description = Some(desc);
                self.device_id = device.as_raw() as usize;
            }
            let slot = &mut self.staging[self.next];
            if let Some(timestamp) = slot.present_qpc {
                let mut mapped = D3D11_MAPPED_SUBRESOURCE::default();
                if unsafe {
                    context.Map(
                        &slot.texture,
                        0,
                        D3D11_MAP_READ,
                        D3D11_MAP_FLAG_DO_NOT_WAIT.0 as u32,
                        Some(&mut mapped),
                    )
                }
                .is_ok()
                {
                    // Map succeeded; the resource remains mapped only within
                    // this lexical guard, even if publication unwinds.
                    struct Unmap<'a>(&'a ID3D11DeviceContext, &'a ID3D11Texture2D);
                    impl Drop for Unmap<'_> {
                        fn drop(&mut self) {
                            unsafe { self.0.Unmap(self.1, 0) }
                        }
                    }
                    let _unmap = Unmap(&context, &slot.texture);
                    let length = mapped.RowPitch as usize * desc.Height as usize;
                    if !mapped.pData.is_null()
                        && mapped.RowPitch >= desc.Width * 4
                        && length <= 512 * 1024 * 1024
                    {
                        let bytes = unsafe {
                            std::slice::from_raw_parts(mapped.pData.cast::<u8>(), length)
                        };
                        if self
                            .publisher
                            .publish(
                                desc.Width,
                                desc.Height,
                                desc.Format.0 as u32,
                                mapped.RowPitch as usize,
                                bytes,
                                timestamp,
                            )
                            .is_err()
                        {
                            self.publisher.dropped();
                        }
                    } else {
                        self.publisher.dropped();
                    }
                    slot.present_qpc = None;
                }
            }
            if slot.present_qpc.is_none() {
                // Ordering invariant: the GPU copy is queued before ClearView.
                unsafe { context.CopyResource(&slot.texture, &back) };
                slot.present_qpc = Some(present_qpc);
            } else {
                self.publisher.dropped();
            }
            self.next = (self.next + 1) % self.staging.len();
        }
        let mut view = None;
        unsafe { device.CreateRenderTargetView(&back, None, Some(&mut view))? };
        // D3D11.1 ClearView does not mutate the game's graphics pipeline state.
        // Rectangles are opaque diagnostics; no blend/sampler/shader state is changed.
        if let Some(overlay) = self.overlay.latest_boxed().filter(|o| {
            o.enabled == 1
                && o.count <= 256
                && o.write_qpc <= present_qpc
                && present_qpc - o.write_qpc <= qpc_frequency() / 5
        }) {
            if let (Ok(context1), Some(view)) = (context.cast::<ID3D11DeviceContext1>(), &view) {
                for rect in overlay.rectangles.iter().take(overlay.count as usize) {
                    if let Some(([left, top, right, bottom], color)) =
                        validated_rect(rect, desc.Width, desc.Height)
                    {
                        unsafe {
                            context1.ClearView(
                                view,
                                &color,
                                Some(&[RECT {
                                    left,
                                    top,
                                    right,
                                    bottom,
                                }]),
                            )
                        };
                    }
                }
            }
        }
        // The HUD is drawn last, after the capture copy was queued, inside its
        // own device-context state.
        if let Some(view) = &view {
            unsafe {
                self.hud.draw(
                    &device,
                    &context,
                    view,
                    desc.Width,
                    desc.Height,
                    present_qpc,
                )
            };
        }
        Ok(())
    }
}

struct Invocation;
impl Invocation {
    fn enter() -> Self {
        ACTIVE.fetch_add(1, Ordering::SeqCst);
        Self
    }
}
impl Drop for Invocation {
    fn drop(&mut self) {
        ACTIVE.fetch_sub(1, Ordering::SeqCst);
    }
}
unsafe extern "system" fn present(this: *mut c_void, sync_interval: u32, flags: u32) -> HRESULT {
    let _invocation = Invocation::enter();
    let timestamp = qpc();
    if !STOPPING.load(Ordering::Acquire) && flags & 1 == 0 {
        let outcome = catch_unwind(AssertUnwindSafe(|| {
            if let (Some(chain), Ok(mut state)) = (
                unsafe { IDXGISwapChain::from_raw_borrowed(&this) },
                RENDER.try_lock(),
            ) {
                if let Some(state) = state.as_mut() {
                    if unsafe { state.frame(chain, timestamp) }.is_err() {
                        ERRORS.fetch_add(1, Ordering::Relaxed);
                    }
                }
            }
        }));
        if outcome.is_err() {
            STOPPING.store(true, Ordering::Release);
            ERRORS.fetch_add(1, Ordering::Relaxed);
        }
    }
    let original = ORIGINAL.load(Ordering::Acquire);
    if original.is_null() {
        return HRESULT(0x80004005_u32 as i32);
    }
    let original: Present = unsafe { std::mem::transmute(original) };
    unsafe { original(this, sync_interval, flags) }
}

unsafe fn probe_present() -> windows::core::Result<*mut c_void> {
    let (_window, chain) = unsafe { create_swapchain(16, 16)? };
    Ok(chain.vtable().Present as *const () as *mut c_void)
}
struct Window(windows::Win32::Foundation::HWND);
impl Drop for Window {
    fn drop(&mut self) {
        let _ = unsafe { DestroyWindow(self.0) };
    }
}
unsafe fn create_swapchain(
    width: u32,
    height: u32,
) -> windows::core::Result<(Window, IDXGISwapChain)> {
    let window = unsafe {
        CreateWindowExW(
            WINDOW_EX_STYLE(0),
            w!("STATIC"),
            w!("ETSADS DX11 probe"),
            WS_OVERLAPPEDWINDOW,
            0,
            0,
            width as i32,
            height as i32,
            None,
            None,
            None,
            None,
        )?
    };
    let window_guard = Window(window);
    let desc = DXGI_SWAP_CHAIN_DESC {
        BufferDesc: DXGI_MODE_DESC {
            Width: width,
            Height: height,
            Format: DXGI_FORMAT_R8G8B8A8_UNORM,
            ..Default::default()
        },
        SampleDesc: DXGI_SAMPLE_DESC {
            Count: 1,
            Quality: 0,
        },
        BufferUsage: DXGI_USAGE_RENDER_TARGET_OUTPUT,
        BufferCount: 1,
        OutputWindow: window,
        Windowed: true.into(),
        SwapEffect: DXGI_SWAP_EFFECT_DISCARD,
        ..Default::default()
    };
    let mut chain = None;
    let mut device = None;
    let mut context = None;
    unsafe {
        D3D11CreateDeviceAndSwapChain(
            None,
            D3D_DRIVER_TYPE_HARDWARE,
            HMODULE::default(),
            D3D11_CREATE_DEVICE_FLAG(0),
            Some(&[D3D_FEATURE_LEVEL_11_0]),
            D3D11_SDK_VERSION,
            Some(&desc),
            Some(&mut chain),
            Some(&mut device),
            None,
            Some(&mut context),
        )?
    };
    let chain = chain.expect("successful swapchain creation");
    Ok((window_guard, chain))
}

/// # Safety
/// SCS supplies a valid initialization structure for the negotiated version.
#[no_mangle]
pub unsafe extern "C" fn scs_telemetry_init(
    version: u32,
    params: *const scs_telemetry_init_params_t,
) -> i32 {
    if version != 0x10001 {
        return -1;
    }
    if params.is_null() {
        return -2;
    }
    let params = unsafe { &*params.cast::<scs_telemetry_init_params_v101_t>() };
    let result = catch_unwind(AssertUnwindSafe(|| -> Result<(), String> {
        let mut render = RENDER.lock().map_err(|_| "poisoned render state")?;
        if render.is_some() && !STOPPING.load(Ordering::Acquire) {
            return Err("graphics plugin already active".into());
        }
        // Pin code and the disabled trampoline until process exit. This avoids
        // a late CPU that fetched the detour address just before disable from
        // ever jumping into unloaded code. SDK shutdown still releases GPU and
        // shared-memory resources; replacing this DLL requires exiting ETS2.
        let mut module = HMODULE::default();
        unsafe {
            GetModuleHandleExW(
                GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_PIN,
                PCWSTR(scs_telemetry_init as *const () as *const u16),
                &mut module,
            )
        }
        .map_err(|e| e.to_string())?;
        let mut target = TARGET.load(Ordering::Acquire);
        if target.is_null() {
            target = unsafe { probe_present() }.map_err(|e| e.to_string())?;
            let original =
                unsafe { MinHook::create_hook(target, present as *const () as *mut c_void) }
                    .map_err(|e| format!("create Present hook: {e:?}"))?;
            ORIGINAL.store(original, Ordering::Release);
            TARGET.store(target, Ordering::Release);
        }
        *render = Some(RenderState::new()?);
        STOPPING.store(false, Ordering::Release);
        unsafe { MinHook::enable_hook(target) }
            .map_err(|e| format!("enable Present hook: {e:?}"))?;
        Ok(())
    }));
    match result {
        Ok(Ok(())) => {
            if let Some(log) = params.common.log {
                unsafe {
                    log(0,c"etsads_gfx: DX11 capture-before-overlay active; restart game to replace gfx DLL".as_ptr())
                }
            }
            0
        }
        _ => {
            STOPPING.store(true, Ordering::Release);
            if let Some(log) = params.common.log {
                unsafe {
                    log(
                        2,
                        c"etsads_gfx initialization failed; telemetry bridge remains independent"
                            .as_ptr(),
                    )
                }
            }
            -7
        }
    }
}
#[no_mangle]
pub extern "C" fn scs_telemetry_shutdown() {
    let _ = catch_unwind(AssertUnwindSafe(|| {
        STOPPING.store(true, Ordering::Release);
        let target = TARGET.load(Ordering::Acquire);
        if !target.is_null() {
            let _ = unsafe { MinHook::disable_hook(target) };
        }
        for _ in 0..100 {
            if ACTIVE.load(Ordering::SeqCst) == 0 {
                break;
            }
            std::thread::sleep(Duration::from_millis(1));
        }
        if ACTIVE.load(Ordering::SeqCst) == 0 {
            if let Ok(mut state) = RENDER.lock() {
                *state = None;
            }
        }
        // Deliberately retain the disabled trampoline with the pinned code module.
    }));
}

#[cfg(test)]
fn test_name(suffix: &str) -> String {
    format!("ETSADS_HookTest_{}_{}", std::process::id(), suffix)
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    #[ignore = "requires a Windows interactive session and D3D11 hardware device"]
    fn gpu_capture_precedes_overlay_without_game() {
        let prefix = format!("ETSADS_GpuTest_{}_{}", std::process::id(), qpc());
        let control = format!("{prefix}_Control");
        let data_prefix = format!("{prefix}_Data_");
        let publisher =
            CapturePublisher::create_named(&control, &data_prefix, &format!("{prefix}_Writer"))
                .unwrap();
        let overlay =
            SharedCell::<OverlayFrame>::create(&format!("{prefix}_Overlay"), SCHEMA_VERSION)
                .unwrap();
        let mut batch = etsads_shm::zeroed_box::<OverlayFrame>();
        batch.enabled = 1;
        batch.count = 1;
        batch.write_qpc = qpc();
        batch.rectangles[0] = etsads_msgs::protocol::OverlayRect {
            bounds: [0.0, 0.0, 16.0, 16.0],
            color_rgba: [0.0, 1.0, 0.0, 1.0],
        };
        overlay.publish(&batch).unwrap();
        let mut state = RenderState {
            publisher,
            overlay,
            hud: Hud::create(&format!("{prefix}_Hud"), &format!("{prefix}_HudStatus")).unwrap(),
            staging: Vec::new(),
            description: None,
            device_id: 0,
            next: 0,
        };
        let mut reader =
            etsads_game::capture::CaptureReader::connect_to(&control, &data_prefix).unwrap();
        let (_window, chain) = unsafe { create_swapchain(64, 64) }.unwrap();
        let device: ID3D11Device = unsafe { chain.GetDevice() }.unwrap();
        let context = unsafe { device.GetImmediateContext() }.unwrap();
        let back: ID3D11Texture2D = unsafe { chain.GetBuffer(0) }.unwrap();
        let mut view = None;
        unsafe { device.CreateRenderTargetView(&back, None, Some(&mut view)) }.unwrap();
        let view = view.unwrap();
        let mut frame = None;
        for _ in 0..40 {
            batch.write_qpc = qpc();
            state.overlay.publish(&batch).unwrap();
            unsafe {
                context.ClearRenderTargetView(&view, &[1.0, 0.0, 0.0, 1.0]);
                state.frame(&chain, qpc()).unwrap();
                context.Flush();
            }
            if let Some(read) = reader.poll().unwrap() {
                frame = Some(read);
                break;
            }
            std::thread::sleep(Duration::from_millis(5));
        }
        let frame = frame.expect("GPU staging frame within 200ms");
        assert_eq!(
            &frame.data[..4],
            &[255, 0, 0, 255],
            "capture must contain red scene, not green overlay"
        );
        let mut desc = D3D11_TEXTURE2D_DESC::default();
        unsafe { back.GetDesc(&mut desc) };
        desc.Usage = D3D11_USAGE_STAGING;
        desc.BindFlags = 0;
        desc.CPUAccessFlags = D3D11_CPU_ACCESS_READ.0 as u32;
        desc.MiscFlags = 0;
        let mut verify = None;
        unsafe { device.CreateTexture2D(&desc, None, Some(&mut verify)) }.unwrap();
        let verify = verify.unwrap();
        unsafe { context.CopyResource(&verify, &back) };
        let mut mapped = D3D11_MAPPED_SUBRESOURCE::default();
        unsafe { context.Map(&verify, 0, D3D11_MAP_READ, 0, Some(&mut mapped)) }.unwrap();
        let color = unsafe { std::slice::from_raw_parts(mapped.pData.cast::<u8>(), 4) }.to_vec();
        unsafe { context.Unmap(&verify, 0) };
        assert_eq!(
            color,
            [0, 255, 0, 255],
            "overlay must actually reach backbuffer"
        );
    }
    #[test]
    #[ignore = "requires a Windows interactive session and D3D11 hardware device"]
    fn gpu_hud_blends_after_capture_and_restores_game_state() {
        use etsads_msgs::protocol::{HudImage, HudStatus};
        let prefix = format!("ETSADS_GpuHud_{}_{}", std::process::id(), qpc());
        let control = format!("{prefix}_Control");
        let data_prefix = format!("{prefix}_Data_");
        let mut state = RenderState {
            publisher: CapturePublisher::create_named(
                &control,
                &data_prefix,
                &format!("{prefix}_Writer"),
            )
            .unwrap(),
            overlay: SharedCell::create(&format!("{prefix}_Overlay"), SCHEMA_VERSION).unwrap(),
            hud: Hud::create(&format!("{prefix}_Hud"), &format!("{prefix}_HudStatus")).unwrap(),
            staging: Vec::new(),
            description: None,
            device_id: 0,
            next: 0,
        };
        let images =
            etsads_shm::SharedRing::<HudImage, 2>::create(&format!("{prefix}_Hud"), SCHEMA_VERSION)
                .unwrap();
        let status =
            SharedCell::<HudStatus>::open(&format!("{prefix}_HudStatus"), SCHEMA_VERSION).unwrap();
        let mut image = etsads_shm::zeroed_box::<HudImage>();
        image.enabled = 1;
        image.width = 8;
        image.height = 4;
        image.sequence = 77;
        for pixel in image.pixels[..8 * 4 * 4].as_chunks_mut::<4>().0 {
            pixel.copy_from_slice(&[0, 128, 0, 128]);
        }
        let mut reader =
            etsads_game::capture::CaptureReader::connect_to(&control, &data_prefix).unwrap();
        let (_window, chain) = unsafe { create_swapchain(64, 64) }.unwrap();
        let device: ID3D11Device = unsafe { chain.GetDevice() }.unwrap();
        let context = unsafe { device.GetImmediateContext() }.unwrap();
        let back: ID3D11Texture2D = unsafe { chain.GetBuffer(0) }.unwrap();
        let mut view = None;
        unsafe { device.CreateRenderTargetView(&back, None, Some(&mut view)) }.unwrap();
        let view = view.unwrap();
        // The game's state at Present: its own target and an odd viewport.
        let game_viewport = D3D11_VIEWPORT {
            TopLeftX: 3.0,
            TopLeftY: 5.0,
            Width: 7.0,
            Height: 9.0,
            MinDepth: 0.0,
            MaxDepth: 1.0,
        };
        let mut captured = None;
        for _ in 0..40 {
            image.write_qpc = qpc();
            images.publish(&image).unwrap();
            unsafe {
                context.ClearRenderTargetView(&view, &[1.0, 0.0, 0.0, 1.0]);
                context.OMSetRenderTargets(Some(&[Some(view.clone())]), None);
                context.RSSetViewports(Some(&[game_viewport]));
                state.frame(&chain, qpc()).unwrap();
                context.Flush();
            }
            if let Some(read) = reader.poll().unwrap() {
                captured = Some(read);
                break;
            }
            std::thread::sleep(Duration::from_millis(5));
        }
        let captured = captured.expect("GPU staging frame within 200ms");
        assert_eq!(
            &captured.data[..4],
            &[255, 0, 0, 255],
            "capture excludes the HUD"
        );
        let mut count = 1;
        let mut viewports = [D3D11_VIEWPORT::default()];
        unsafe { context.RSGetViewports(&mut count, Some(viewports.as_mut_ptr())) };
        assert_eq!(
            (count, viewports[0]),
            (1, game_viewport),
            "viewport restored"
        );
        let mut targets = [None];
        unsafe { context.OMGetRenderTargets(Some(&mut targets), None) };
        assert_eq!(targets[0].as_ref().map(|v| v.as_raw()), Some(view.as_raw()));
        let mut desc = D3D11_TEXTURE2D_DESC::default();
        unsafe { back.GetDesc(&mut desc) };
        desc.Usage = D3D11_USAGE_STAGING;
        desc.BindFlags = 0;
        desc.CPUAccessFlags = D3D11_CPU_ACCESS_READ.0 as u32;
        desc.MiscFlags = 0;
        let mut verify = None;
        unsafe { device.CreateTexture2D(&desc, None, Some(&mut verify)) }.unwrap();
        let verify = verify.unwrap();
        unsafe { context.CopyResource(&verify, &back) };
        let mut mapped = D3D11_MAPPED_SUBRESOURCE::default();
        unsafe { context.Map(&verify, 0, D3D11_MAP_READ, 0, Some(&mut mapped)) }.unwrap();
        let row = unsafe { std::slice::from_raw_parts(mapped.pData.cast::<u8>(), 64 * 4) }.to_vec();
        unsafe { context.Unmap(&verify, 0) };
        let blended = &row[..4];
        assert!(
            blended[0].abs_diff(127) <= 2 && blended[1].abs_diff(128) <= 2 && blended[2] == 0,
            "half-transparent green over red, got {blended:?}"
        );
        assert_eq!(
            &row[10 * 4..10 * 4 + 4],
            &[255, 0, 0, 255],
            "outside the HUD"
        );
        assert_eq!(status.latest().unwrap().drawn_sequence, 77);
    }
    #[test]
    #[ignore = "requires a Windows interactive session and D3D11 hardware device"]
    fn real_present_hook_can_deactivate_and_reactivate() {
        for _ in 0..5 {
            let params = scs_telemetry_init_params_v101_t::default();
            assert_eq!(
                unsafe {
                    scs_telemetry_init(
                        0x10001,
                        (&params as *const scs_telemetry_init_params_v101_t).cast(),
                    )
                },
                0
            );
            let mut reader = etsads_game::capture::CaptureReader::connect_to(
                &test_name("Control"),
                &test_name("Data_"),
            )
            .unwrap();
            let (_window, chain) = unsafe { create_swapchain(64, 64) }.unwrap();
            let device: ID3D11Device = unsafe { chain.GetDevice() }.unwrap();
            let context = unsafe { device.GetImmediateContext() }.unwrap();
            let back: ID3D11Texture2D = unsafe { chain.GetBuffer(0) }.unwrap();
            let mut view = None;
            unsafe { device.CreateRenderTargetView(&back, None, Some(&mut view)) }.unwrap();
            let view = view.unwrap();
            let mut captured = false;
            for _ in 0..40 {
                unsafe {
                    context.ClearRenderTargetView(&view, &[0.0, 0.0, 1.0, 1.0]);
                    let _ = chain.Present(0, windows::Win32::Graphics::Dxgi::DXGI_PRESENT(0));
                    context.Flush();
                }
                if let Some(frame) = reader.poll().unwrap() {
                    assert_eq!(&frame.data[..4], &[0, 0, 255, 255]);
                    captured = true;
                    break;
                }
                std::thread::sleep(Duration::from_millis(5));
            }
            scs_telemetry_shutdown();
            assert!(STOPPING.load(Ordering::Acquire));
            assert_eq!(ACTIVE.load(Ordering::Acquire), 0);
            assert!(RENDER.lock().unwrap().is_none());
            assert!(captured, "real Present detour must publish a frame");
            unsafe {
                let _ = chain.Present(0, windows::Win32::Graphics::Dxgi::DXGI_PRESENT(0));
            }
        }
    }
}
