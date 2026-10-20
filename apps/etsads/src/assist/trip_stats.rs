//! Summary of assist logs: distance, speed against the limit by road class,
//! what held the speed target down, and every release or minimal-risk stop.
//! Read-only; used to evaluate long hauls and town speed.

use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::path::PathBuf;

/// The Windows performance counter runs at 10 MHz on every supported system.
const QPC_HZ: f64 = 1e7;

#[derive(Default)]
struct Bucket {
    seconds: f64,
    metres: f64,
}
impl Bucket {
    fn add(&mut self, seconds: f64, metres: f64) {
        self.seconds += seconds;
        self.metres += metres;
    }
    fn kmh(&self) -> f64 {
        if self.seconds > 0.0 {
            self.metres / self.seconds * 3.6
        } else {
            0.0
        }
    }
}

struct Drift {
    at_km: f64,
    peak_m: f64,
    speed_kmh: f64,
    limit_kmh: f64,
    binding: String,
    lane: String,
}

/// The given log files (or every `assist-*.jsonl` in a directory), read in
/// order of their counter timestamp.
pub fn summarize(inputs: &[PathBuf]) -> anyhow::Result<String> {
    let mut files = Vec::new();
    for input in inputs {
        if input.is_dir() {
            for entry in std::fs::read_dir(input)? {
                let path = entry?.path();
                let name = path.file_name().unwrap_or_default().to_string_lossy();
                if name.starts_with("assist-") && name.ends_with(".jsonl") {
                    files.push(path);
                }
            }
        } else {
            files.push(input.clone());
        }
    }
    let stamp = |p: &PathBuf| -> u64 {
        p.file_stem()
            .and_then(|s| s.to_str())
            .and_then(|s| s.rsplit('-').next())
            .and_then(|s| s.parse().ok())
            .unwrap_or(0)
    };
    files.sort_by_key(stamp);
    anyhow::ensure!(!files.is_empty(), "没有找到辅助日志");

    let mut total = Bucket::default();
    let mut by_binding: BTreeMap<String, Bucket> = BTreeMap::new();
    let mut by_class: BTreeMap<&'static str, (Bucket, f64, f64)> = BTreeMap::new();
    let mut by_reason: BTreeMap<String, Bucket> = BTreeMap::new();
    let mut events: BTreeMap<String, u64> = BTreeMap::new();
    let mut stops = Vec::new();
    let mut lateral = Vec::new();
    let mut drift = None::<Drift>;
    let mut drifts = Vec::new();
    let mut max_kmh: f64 = 0.0;
    let mut run_m = 0.0;
    let mut longest_m: f64 = 0.0;
    let mut stopped_s = 0.0;
    let mut game_km = 0.0;
    let mut fuel_used = 0.0;
    let mut previous: Option<(u64, [f64; 3], bool, f64, f64)> = None;
    for file in &files {
        let text = std::fs::read_to_string(file)?;
        for line in text.lines() {
            let Ok(value) = serde_json::from_str::<serde_json::Value>(line) else {
                continue;
            };
            if let Some(event) = value.get("event").and_then(|e| e.as_str()) {
                *events.entry(event.to_owned()).or_default() += 1;
                if matches!(
                    event,
                    "released" | "minimal_risk_stop" | "minimal_risk_brake_hold"
                ) {
                    stops.push(format!(
                        "{} · {}",
                        event,
                        value.get("reason").and_then(|r| r.as_str()).unwrap_or("")
                    ));
                }
                if event == "released" {
                    longest_m = longest_m.max(run_m);
                    run_m = 0.0;
                }
                continue;
            }
            let Some(mode) = value.get("mode").and_then(|m| m.as_str()) else {
                continue;
            };
            let engaged = matches!(mode, "Engaged" | "TakeoverSoon");
            let qpc = value
                .get("source_qpc")
                .and_then(|q| q.as_u64())
                .unwrap_or(0);
            let position = value
                .get("position_enu")
                .and_then(|p| serde_json::from_value::<[f64; 3]>(p.clone()).ok())
                .unwrap_or_default();
            let number = |key: &str| value.get(key).and_then(|v| v.as_f64()).unwrap_or(0.0);
            if let Some((last_qpc, last_position, last_engaged, last_nav, last_fuel)) = previous
                && engaged
                && last_engaged
                && qpc > last_qpc
            {
                let seconds = (qpc - last_qpc) as f64 / QPC_HZ;
                let metres = (position[0] - last_position[0]).hypot(position[1] - last_position[1]);
                // A gap in the log or a teleport is not driven distance.
                if seconds < 3.0 && metres < 40.0 * seconds + 2.0 {
                    total.add(seconds, metres);
                    // The GPS distance falls by the scaled kilometres driven; a
                    // new destination or reroute jumps and is not counted.
                    let nav = number("navigation_km");
                    if nav > 0.0 && last_nav > nav && last_nav - nav < 2.0 * seconds {
                        game_km += last_nav - nav;
                    }
                    let fuel = number("fuel_l");
                    if last_fuel > fuel && last_fuel - fuel < 1.0 {
                        fuel_used += last_fuel - fuel;
                    }
                    run_m += metres;
                    let speed = number("speed_kmh");
                    max_kmh = max_kmh.max(speed);
                    if speed < 1.0 {
                        stopped_s += seconds;
                    }
                    let binding = value
                        .get("binding")
                        .and_then(|b| b.as_str())
                        .unwrap_or("unknown")
                        .to_owned();
                    by_binding.entry(binding).or_default().add(seconds, metres);
                    let limit = number("limit_kmh");
                    let class = match limit {
                        l if l <= 1.0 => "无限速数据",
                        l if l <= 55.0 => "城镇 ≤55",
                        l if l <= 75.0 => "乡道 60–70",
                        _ => "高速 ≥80",
                    };
                    let entry = by_class.entry(class).or_default();
                    entry.0.add(seconds, metres);
                    entry.1 += limit * seconds;
                    if speed < 1.0 {
                        entry.2 += seconds;
                    }
                    let reason = value
                        .get("reason")
                        .and_then(|r| r.as_str())
                        .unwrap_or("")
                        .to_owned();
                    by_reason.entry(reason).or_default().add(seconds, metres);
                    let offset = number("lateral_m");
                    lateral.push((offset.abs(), seconds));
                    // A drift episode: from 0.6 m until back under 0.3 m.
                    match drift.as_mut() {
                        None if offset.abs() >= 0.6 => {
                            drift = Some(Drift {
                                at_km: total.metres / 1000.0,
                                peak_m: offset,
                                speed_kmh: speed,
                                limit_kmh: limit,
                                binding: value
                                    .get("binding")
                                    .and_then(|b| b.as_str())
                                    .unwrap_or("unknown")
                                    .to_owned(),
                                lane: value
                                    .get("lane")
                                    .and_then(|b| b.as_str())
                                    .unwrap_or("")
                                    .to_owned(),
                            });
                        }
                        Some(d) if offset.abs() > d.peak_m.abs() => d.peak_m = offset,
                        Some(_) if offset.abs() < 0.3 => drifts.extend(drift.take()),
                        _ => {}
                    }
                }
            }
            if !engaged {
                longest_m = longest_m.max(run_m);
                run_m = 0.0;
            }
            previous = Some((
                qpc,
                position,
                engaged,
                number("navigation_km"),
                number("fuel_l"),
            ));
        }
    }
    longest_m = longest_m.max(run_m);
    lateral.sort_by(|a, b| a.0.total_cmp(&b.0));
    let quantile = |q: f64| {
        let goal = lateral.iter().map(|l| l.1).sum::<f64>() * q;
        let mut sum = 0.0;
        for (value, weight) in &lateral {
            sum += weight;
            if sum >= goal {
                return *value;
            }
        }
        lateral.last().map_or(0.0, |l| l.0)
    };

    let mut out = String::new();
    writeln!(out, "日志 {} 个", files.len())?;
    writeln!(
        out,
        "辅助驾驶 {:.1} km，{:.1} h，平均 {:.1} km/h（停车 {:.0} 分钟），最高 {:.0} km/h，最长连续 {:.1} km",
        total.metres / 1000.0,
        total.seconds / 3600.0,
        total.kmh(),
        stopped_s / 60.0,
        max_kmh,
        longest_m / 1000.0
    )?;
    writeln!(
        out,
        "导航里程（游戏比例）{game_km:.0} km · 耗油 {fuel_used:.0} L"
    )?;
    writeln!(
        out,
        "横向偏差 p50 {:.2} m · p95 {:.2} m · p99 {:.2} m · 最大 {:.2} m",
        quantile(0.5),
        quantile(0.95),
        quantile(0.99),
        lateral.last().map_or(0.0, |l| l.0)
    )?;
    writeln!(out, "\n按道路限速：")?;
    for (class, (bucket, limit_seconds, stopped)) in &by_class {
        // Waiting (a red light, a blocked road) is shown apart: r13 stood
        // 30 min behind a police stop and the rural average meant nothing.
        writeln!(
            out,
            "  {class:<10} {:>7.1} km {:>6.1} min  平均 {:>5.1} km/h · 行驶中 {:>5.1} km/h（停车 {:>4.1} min，平均限速 {:>5.1}）",
            bucket.metres / 1000.0,
            bucket.seconds / 60.0,
            bucket.kmh(),
            bucket.metres / (bucket.seconds - stopped).max(1e-9) * 3.6,
            stopped / 60.0,
            limit_seconds / bucket.seconds.max(1e-9)
        )?;
    }
    writeln!(out, "\n目标车速由谁决定（按时间）：")?;
    let mut bindings: Vec<_> = by_binding.iter().collect();
    bindings.sort_by(|a, b| b.1.seconds.total_cmp(&a.1.seconds));
    for (binding, bucket) in bindings {
        writeln!(
            out,
            "  {binding:<17} {:>6.1} min {:>7.1} km  平均 {:>5.1} km/h",
            bucket.seconds / 60.0,
            bucket.metres / 1000.0,
            bucket.kmh()
        )?;
    }
    writeln!(out, "\n提示原因（前 12）：")?;
    let mut reasons: Vec<_> = by_reason.iter().collect();
    reasons.sort_by(|a, b| b.1.seconds.total_cmp(&a.1.seconds));
    for (reason, bucket) in reasons.into_iter().take(12) {
        writeln!(
            out,
            "  {:>6.1} min  平均 {:>5.1} km/h  {reason}",
            bucket.seconds / 60.0,
            bucket.kmh()
        )?;
    }
    writeln!(out, "\n事件：")?;
    for (event, count) in &events {
        writeln!(out, "  {event} × {count}")?;
    }
    drifts.extend(drift.take());
    if !drifts.is_empty() {
        writeln!(
            out,
            "
偏离 ≥0.6 m（{} 次）：",
            drifts.len()
        )?;
        for d in &drifts {
            writeln!(
                out,
                "  {:>7.2} km 峰值 {:+.2} m · 起始 {:.0} km/h（限速 {:.0}）· {} · {}",
                d.at_km, d.peak_m, d.speed_kmh, d.limit_kmh, d.binding, d.lane
            )?;
        }
    }
    if !stops.is_empty() {
        writeln!(out, "\n释放 / 最小风险停车：")?;
        for stop in &stops {
            writeln!(out, "  {stop}")?;
        }
    }
    Ok(out)
}
