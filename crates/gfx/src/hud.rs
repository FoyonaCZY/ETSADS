//! Heads-up display: one host-rendered, premultiplied RGBA image drawn as an
//! alpha-blended quad. Drawing runs inside a private device-context state
//! object, so none of the game's pipeline state is read, changed or restored
//! piecemeal; `SwapDeviceContextState` puts the game's exact state back.
use etsads_msgs::protocol::{
    HudImage, HudStatus, HUD_ANCHOR_BOTTOM_CENTER, HUD_ANCHOR_TOP_CENTER, HUD_ANCHOR_TOP_RIGHT,
    HUD_MAX_HEIGHT, HUD_MAX_WIDTH, SCHEMA_VERSION,
};
use etsads_shm::{qpc_frequency, zeroed_box, SharedCell, SharedRing};
use windows::{
    core::{s, Interface, PCSTR},
    Win32::Graphics::{
        Direct3D::{
            Fxc::{D3DCompile, D3DCOMPILE_OPTIMIZATION_LEVEL3},
            ID3DBlob, D3D11_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP,
        },
        Direct3D11::*,
        Dxgi::Common::{DXGI_FORMAT_R8G8B8A8_UNORM, DXGI_SAMPLE_DESC},
    },
};

/// Images older than this are never drawn.
const STALE_MS: u64 = 500;
const SHADER: &str = r"
cbuffer Hud : register(b0) { float4 rect; float4 uv; };
struct V { float4 pos : SV_Position; float2 tex : TEXCOORD0; };
V vs_main(uint id : SV_VertexID) {
    float2 c = float2(id & 1, id >> 1);
    V o;
    o.pos = float4(lerp(rect.xy, rect.zw, c), 0.0, 1.0);
    o.tex = c * uv.xy;
    return o;
}
Texture2D image : register(t0);
SamplerState nearest : register(s0);
float4 ps_main(V i) : SV_Target { return image.Sample(nearest, i.tex); }
";

struct Pipeline {
    device_id: usize,
    state: ID3DDeviceContextState,
    vertex: ID3D11VertexShader,
    pixel: ID3D11PixelShader,
    sampler: ID3D11SamplerState,
    blend: ID3D11BlendState,
    raster: ID3D11RasterizerState,
    constants: ID3D11Buffer,
    texture: ID3D11Texture2D,
    view: ID3D11ShaderResourceView,
    uploaded: u64,
}

pub(crate) struct Hud {
    images: SharedRing<HudImage, 2>,
    status: SharedCell<HudStatus>,
    image: Box<HudImage>,
    index: u64,
    pipeline: Option<Pipeline>,
    frames: u64,
    errors: u32,
}
impl Hud {
    pub(crate) fn create(image_name: &str, status_name: &str) -> Result<Self, String> {
        Ok(Self {
            images: SharedRing::create(image_name, SCHEMA_VERSION).map_err(|e| e.to_string())?,
            status: SharedCell::create(status_name, SCHEMA_VERSION).map_err(|e| e.to_string())?,
            // Never on the stack: the image is 2 MiB and this runs on the
            // game's render thread.
            image: zeroed_box(),
            index: 0,
            pipeline: None,
            frames: 0,
            errors: 0,
        })
    }
    /// Draw the latest fresh image over `target`. Errors only skip the HUD.
    pub(crate) unsafe fn draw(
        &mut self,
        device: &ID3D11Device,
        context: &ID3D11DeviceContext,
        target: &ID3D11RenderTargetView,
        width: u32,
        height: u32,
        present_qpc: u64,
    ) {
        let drawn =
            match unsafe { self.try_draw(device, context, target, width, height, present_qpc) } {
                Ok(drawn) => drawn,
                Err(_) => {
                    self.errors = self.errors.saturating_add(1);
                    // A failed pipeline is rebuilt on the next frame.
                    self.pipeline = None;
                    false
                }
            };
        if drawn {
            self.frames += 1;
        }
        let _ = self.status.publish(&HudStatus {
            present_qpc,
            drawn_sequence: if drawn { self.image.sequence } else { 0 },
            frames_drawn: self.frames,
            backbuffer_width: width,
            backbuffer_height: height,
            errors: self.errors,
            reserved: 0,
        });
    }
    unsafe fn try_draw(
        &mut self,
        device: &ID3D11Device,
        context: &ID3D11DeviceContext,
        target: &ID3D11RenderTargetView,
        width: u32,
        height: u32,
        present_qpc: u64,
    ) -> windows::core::Result<bool> {
        let latest = self.images.latest_index();
        if latest != 0 && latest != self.index && self.images.read_into(latest, &mut self.image) {
            self.index = latest;
        }
        let image = &*self.image;
        let (w, h) = (image.width as usize, image.height as usize);
        if self.index == 0
            || image.enabled != 1
            || !(1..=HUD_MAX_WIDTH).contains(&w)
            || !(1..=HUD_MAX_HEIGHT).contains(&h)
            || image.write_qpc > present_qpc
            || present_qpc - image.write_qpc > qpc_frequency() * STALE_MS / 1000
            || !image.offset_px.iter().all(|v| v.is_finite())
        {
            return Ok(false);
        }
        let device_id = device.as_raw() as usize;
        if self
            .pipeline
            .as_ref()
            .is_none_or(|p| p.device_id != device_id)
        {
            self.pipeline = Some(unsafe { Pipeline::new(device)? });
        }
        let pipeline = self.pipeline.as_mut().expect("pipeline just created");
        if pipeline.uploaded != self.index {
            let mut mapped = D3D11_MAPPED_SUBRESOURCE::default();
            unsafe {
                context.Map(
                    &pipeline.texture,
                    0,
                    D3D11_MAP_WRITE_DISCARD,
                    0,
                    Some(&mut mapped),
                )?
            };
            if !mapped.pData.is_null() && mapped.RowPitch as usize >= w * 4 {
                for row in 0..h {
                    // SAFETY: the mapped texture is HUD_MAX_WIDTH x HUD_MAX_HEIGHT
                    // and `row < h <= HUD_MAX_HEIGHT`, `w <= HUD_MAX_WIDTH`.
                    unsafe {
                        std::ptr::copy_nonoverlapping(
                            image.pixels.as_ptr().add(row * w * 4),
                            mapped
                                .pData
                                .cast::<u8>()
                                .add(row * mapped.RowPitch as usize),
                            w * 4,
                        )
                    };
                }
            }
            unsafe { context.Unmap(&pipeline.texture, 0) };
            pipeline.uploaded = self.index;
        }
        let (x, y) = placement(image, width, height);
        let (x0, y0) = (x as f32, y as f32);
        let (x1, y1) = (x0 + w as f32, y0 + h as f32);
        let (fw, fh) = (width as f32, height as f32);
        let constants: [f32; 8] = [
            x0 / fw * 2.0 - 1.0,
            1.0 - y0 / fh * 2.0,
            x1 / fw * 2.0 - 1.0,
            1.0 - y1 / fh * 2.0,
            w as f32 / HUD_MAX_WIDTH as f32,
            h as f32 / HUD_MAX_HEIGHT as f32,
            0.0,
            0.0,
        ];
        let context1: ID3D11DeviceContext1 = context.cast()?;
        let mut previous = None;
        unsafe { context1.SwapDeviceContextState(&pipeline.state, Some(&mut previous)) };
        let Some(previous) = previous else {
            return Ok(false);
        };
        unsafe {
            context.UpdateSubresource(
                &pipeline.constants,
                0,
                None,
                constants.as_ptr().cast(),
                0,
                0,
            );
            context.OMSetRenderTargets(Some(&[Some(target.clone())]), None);
            context.OMSetBlendState(&pipeline.blend, None, 0xffff_ffff);
            context.RSSetState(&pipeline.raster);
            context.RSSetViewports(Some(&[D3D11_VIEWPORT {
                TopLeftX: 0.0,
                TopLeftY: 0.0,
                Width: fw,
                Height: fh,
                MinDepth: 0.0,
                MaxDepth: 1.0,
            }]));
            context.IASetInputLayout(None);
            context.IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP);
            context.VSSetShader(&pipeline.vertex, None);
            context.VSSetConstantBuffers(0, Some(&[Some(pipeline.constants.clone())]));
            context.PSSetShader(&pipeline.pixel, None);
            context.PSSetShaderResources(0, Some(&[Some(pipeline.view.clone())]));
            context.PSSetSamplers(0, Some(&[Some(pipeline.sampler.clone())]));
            context.Draw(4, 0);
            // Unbind the backbuffer from the private state before it is
            // presented, then restore the game's complete state.
            context.OMSetRenderTargets(None, None);
            context1.SwapDeviceContextState(&previous, None);
        }
        Ok(true)
    }
}

/// Top-left pixel of the image, kept fully on screen.
fn placement(image: &HudImage, width: u32, height: u32) -> (i64, i64) {
    let (w, h) = (image.width as i64, image.height as i64);
    let (sw, sh) = (width as i64, height as i64);
    let [dx, dy] = image.offset_px.map(|v| v.round() as i64);
    let (x, y) = match image.anchor {
        HUD_ANCHOR_TOP_CENTER => ((sw - w) / 2 + dx, dy),
        HUD_ANCHOR_TOP_RIGHT => (sw - w - dx, dy),
        HUD_ANCHOR_BOTTOM_CENTER => ((sw - w) / 2 + dx, sh - h - dy),
        _ => (dx, dy),
    };
    (x.clamp(0, (sw - w).max(0)), y.clamp(0, (sh - h).max(0)))
}

impl Pipeline {
    unsafe fn new(device: &ID3D11Device) -> windows::core::Result<Self> {
        let device1: ID3D11Device1 = device.cast()?;
        let flags =
            if unsafe { device.GetCreationFlags() } & D3D11_CREATE_DEVICE_SINGLETHREADED.0 != 0 {
                D3D11_1_CREATE_DEVICE_CONTEXT_STATE_SINGLETHREADED.0 as u32
            } else {
                0
            };
        let level = unsafe { device.GetFeatureLevel() };
        let mut state = None;
        unsafe {
            device1.CreateDeviceContextState(
                flags,
                &[level],
                D3D11_SDK_VERSION,
                &ID3D11Device::IID,
                None,
                Some(&mut state),
            )?
        };
        let vs = unsafe { compile(s!("vs_main"), s!("vs_4_0"))? };
        let ps = unsafe { compile(s!("ps_main"), s!("ps_4_0"))? };
        let mut vertex = None;
        let mut pixel = None;
        unsafe {
            device.CreateVertexShader(blob(&vs), None, Some(&mut vertex))?;
            device.CreatePixelShader(blob(&ps), None, Some(&mut pixel))?;
        }
        let mut sampler = None;
        unsafe {
            device.CreateSamplerState(
                &D3D11_SAMPLER_DESC {
                    Filter: D3D11_FILTER_MIN_MAG_MIP_POINT,
                    AddressU: D3D11_TEXTURE_ADDRESS_CLAMP,
                    AddressV: D3D11_TEXTURE_ADDRESS_CLAMP,
                    AddressW: D3D11_TEXTURE_ADDRESS_CLAMP,
                    ComparisonFunc: D3D11_COMPARISON_NEVER,
                    MaxLOD: f32::MAX,
                    ..Default::default()
                },
                Some(&mut sampler),
            )?
        };
        let mut blend_desc = D3D11_BLEND_DESC::default();
        blend_desc.RenderTarget[0] = D3D11_RENDER_TARGET_BLEND_DESC {
            BlendEnable: true.into(),
            SrcBlend: D3D11_BLEND_ONE,
            DestBlend: D3D11_BLEND_INV_SRC_ALPHA,
            BlendOp: D3D11_BLEND_OP_ADD,
            SrcBlendAlpha: D3D11_BLEND_ONE,
            DestBlendAlpha: D3D11_BLEND_INV_SRC_ALPHA,
            BlendOpAlpha: D3D11_BLEND_OP_ADD,
            RenderTargetWriteMask: D3D11_COLOR_WRITE_ENABLE_ALL.0 as u8,
        };
        let mut blend = None;
        unsafe { device.CreateBlendState(&blend_desc, Some(&mut blend))? };
        let mut raster = None;
        unsafe {
            device.CreateRasterizerState(
                &D3D11_RASTERIZER_DESC {
                    FillMode: D3D11_FILL_SOLID,
                    CullMode: D3D11_CULL_NONE,
                    DepthClipEnable: true.into(),
                    ..Default::default()
                },
                Some(&mut raster),
            )?
        };
        let mut constants = None;
        unsafe {
            device.CreateBuffer(
                &D3D11_BUFFER_DESC {
                    ByteWidth: 32,
                    Usage: D3D11_USAGE_DEFAULT,
                    BindFlags: D3D11_BIND_CONSTANT_BUFFER.0 as u32,
                    ..Default::default()
                },
                None,
                Some(&mut constants),
            )?
        };
        let mut texture = None;
        unsafe {
            device.CreateTexture2D(
                &D3D11_TEXTURE2D_DESC {
                    Width: HUD_MAX_WIDTH as u32,
                    Height: HUD_MAX_HEIGHT as u32,
                    MipLevels: 1,
                    ArraySize: 1,
                    Format: DXGI_FORMAT_R8G8B8A8_UNORM,
                    SampleDesc: DXGI_SAMPLE_DESC {
                        Count: 1,
                        Quality: 0,
                    },
                    Usage: D3D11_USAGE_DYNAMIC,
                    BindFlags: D3D11_BIND_SHADER_RESOURCE.0 as u32,
                    CPUAccessFlags: D3D11_CPU_ACCESS_WRITE.0 as u32,
                    MiscFlags: 0,
                },
                None,
                Some(&mut texture),
            )?
        };
        let texture = texture.expect("successful texture creation");
        let mut view = None;
        unsafe { device.CreateShaderResourceView(&texture, None, Some(&mut view))? };
        let missing = || windows::core::Error::from(windows::Win32::Foundation::E_FAIL);
        Ok(Self {
            device_id: device.as_raw() as usize,
            state: state.ok_or_else(missing)?,
            vertex: vertex.ok_or_else(missing)?,
            pixel: pixel.ok_or_else(missing)?,
            sampler: sampler.ok_or_else(missing)?,
            blend: blend.ok_or_else(missing)?,
            raster: raster.ok_or_else(missing)?,
            constants: constants.ok_or_else(missing)?,
            texture,
            view: view.ok_or_else(missing)?,
            uploaded: 0,
        })
    }
}
unsafe fn compile(entry: PCSTR, target: PCSTR) -> windows::core::Result<ID3DBlob> {
    let mut code = None;
    unsafe {
        D3DCompile(
            SHADER.as_ptr().cast(),
            SHADER.len(),
            s!("etsads_hud"),
            None,
            None,
            entry,
            target,
            D3DCOMPILE_OPTIMIZATION_LEVEL3,
            0,
            &mut code,
            None,
        )?
    };
    code.ok_or_else(|| windows::core::Error::from(windows::Win32::Foundation::E_FAIL))
}
fn blob(blob: &ID3DBlob) -> &[u8] {
    // SAFETY: a compiled blob owns `GetBufferSize` bytes at `GetBufferPointer`
    // for its lifetime, which bounds the returned slice.
    unsafe { std::slice::from_raw_parts(blob.GetBufferPointer().cast(), blob.GetBufferSize()) }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn placement_keeps_the_image_on_screen() {
        let mut image = zeroed_box::<HudImage>();
        image.width = 400;
        image.height = 100;
        image.offset_px = [24.0, 24.0];
        assert_eq!(placement(&image, 1920, 1080), (24, 24));
        image.anchor = HUD_ANCHOR_TOP_RIGHT;
        assert_eq!(placement(&image, 1920, 1080), (1496, 24));
        image.anchor = HUD_ANCHOR_BOTTOM_CENTER;
        assert_eq!(placement(&image, 1920, 1080), (784, 956));
        image.offset_px = [5000.0, -50.0];
        image.anchor = 0;
        assert_eq!(placement(&image, 1920, 1080), (1520, 0));
    }
}
