//! `etsads --ui-preview <dir>` draws the app's pages to PNG files with no
//! window, GPU or game connection, so the look can be checked while the game
//! holds the foreground.
use eframe::egui::{self, Color32, TextureId, epaint};
use std::{collections::HashMap, path::Path};

/// egui sends the font atlas once, then only its changes: keep it between
/// pictures.
#[derive(Default)]
pub struct Textures(HashMap<TextureId, Texture>);
struct Texture {
    size: [usize; 2],
    pixels: Vec<Color32>,
}
fn update(textures: &mut HashMap<TextureId, Texture>, delta: &egui::TexturesDelta) {
    for (id, change) in &delta.set {
        let egui::ImageData::Color(image) = &change.image;
        match change.pos {
            None => {
                textures.insert(
                    *id,
                    Texture {
                        size: image.size,
                        pixels: image.pixels.clone(),
                    },
                );
            }
            Some([x, y]) => {
                let Some(texture) = textures.get_mut(id) else {
                    continue;
                };
                for row in 0..image.size[1] {
                    for col in 0..image.size[0] {
                        texture.pixels[(y + row) * texture.size[0] + x + col] =
                            image.pixels[row * image.size[0] + col];
                    }
                }
            }
        }
    }
}
/// Source-over triangle in egui's premultiplied gamma space.
fn triangle(
    canvas: &mut [[f32; 3]],
    width: usize,
    clip: [i64; 4],
    texture: &Texture,
    v: [&epaint::Vertex; 3],
) {
    let (a, b, c) = (v[0].pos, v[1].pos, v[2].pos);
    let area = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    if area.abs() < 1e-6 {
        return;
    }
    let low = |p: f32, q: f32, r: f32| p.min(q).min(r).floor() as i64;
    let high = |p: f32, q: f32, r: f32| p.max(q).max(r).ceil() as i64;
    for y in low(a.y, b.y, c.y).max(clip[1])..high(a.y, b.y, c.y).min(clip[3]) {
        for x in low(a.x, b.x, c.x).max(clip[0])..high(a.x, b.x, c.x).min(clip[2]) {
            let (px, py) = (x as f32 + 0.5, y as f32 + 0.5);
            let w0 = ((b.x - px) * (c.y - py) - (b.y - py) * (c.x - px)) / area;
            let w1 = ((c.x - px) * (a.y - py) - (c.y - py) * (a.x - px)) / area;
            let w2 = 1.0 - w0 - w1;
            if w0 < 0.0 || w1 < 0.0 || w2 < 0.0 {
                continue;
            }
            let u = w0 * v[0].uv.x + w1 * v[1].uv.x + w2 * v[2].uv.x;
            let t = w0 * v[0].uv.y + w1 * v[1].uv.y + w2 * v[2].uv.y;
            let tx = ((u * texture.size[0] as f32) as usize).min(texture.size[0] - 1);
            let ty = ((t * texture.size[1] as f32) as usize).min(texture.size[1] - 1);
            let sample = texture.pixels[ty * texture.size[0] + tx].to_array();
            let mut out = [0.0_f32; 4];
            for (k, value) in out.iter_mut().enumerate() {
                let tint = w0 * v[0].color.to_array()[k] as f32
                    + w1 * v[1].color.to_array()[k] as f32
                    + w2 * v[2].color.to_array()[k] as f32;
                *value = tint * sample[k] as f32 / 255.0;
            }
            let pixel = &mut canvas[y as usize * width + x as usize];
            let keep = 1.0 - out[3] / 255.0;
            for k in 0..3 {
                pixel[k] = out[k] + pixel[k] * keep;
            }
        }
    }
}
/// Run `draw` for a few frames on `ctx` and save the last one.
pub fn shoot(
    textures: &mut Textures,
    ctx: &egui::Context,
    path: &Path,
    size: [usize; 2],
    mut draw: impl FnMut(&egui::Context),
) -> anyhow::Result<()> {
    let mut last = None;
    for frame in 0..4 {
        let input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::Pos2::ZERO,
                egui::vec2(size[0] as f32, size[1] as f32),
            )),
            time: Some(frame as f64 * 0.5),
            ..Default::default()
        };
        let output = ctx.run(input, &mut draw);
        update(&mut textures.0, &output.textures_delta);
        last = Some(output);
    }
    let output = last.expect("ran at least once");
    let meshes = ctx.tessellate(output.shapes, output.pixels_per_point);
    let mut canvas = vec![[0.0_f32; 3]; size[0] * size[1]];
    for primitive in meshes {
        let epaint::Primitive::Mesh(mesh) = primitive.primitive else {
            continue;
        };
        let Some(texture) = textures.0.get(&mesh.texture_id) else {
            continue;
        };
        let r = primitive.clip_rect;
        let clip = [
            r.left().floor().max(0.0) as i64,
            r.top().floor().max(0.0) as i64,
            r.right().ceil().min(size[0] as f32) as i64,
            r.bottom().ceil().min(size[1] as f32) as i64,
        ];
        for t in mesh.indices.as_chunks::<3>().0 {
            let corner = |i: usize| &mesh.vertices[t[i] as usize];
            triangle(
                &mut canvas,
                size[0],
                clip,
                texture,
                [corner(0), corner(1), corner(2)],
            );
        }
    }
    let bytes = canvas
        .iter()
        .flat_map(|p| p.map(|v| v.round().clamp(0.0, 255.0) as u8))
        .collect::<Vec<_>>();
    image::save_buffer(
        path,
        &bytes,
        size[0] as u32,
        size[1] as u32,
        image::ColorType::Rgb8,
    )?;
    Ok(())
}
