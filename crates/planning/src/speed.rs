use crate::PlanningError;
#[derive(Clone, Copy, Debug)]
pub struct IdmConfig {
    pub desired_speed: f64,
    pub time_headway: f64,
    pub minimum_gap: f64,
    pub max_acceleration: f64,
    pub comfortable_braking: f64,
    pub emergency_braking: f64,
}
impl Default for IdmConfig {
    fn default() -> Self {
        Self {
            desired_speed: 25.0,
            time_headway: 2.0,
            minimum_gap: 8.0,
            max_acceleration: 1.0,
            comfortable_braking: 1.5,
            emergency_braking: 4.0,
        }
    }
}
impl IdmConfig {
    pub fn acceleration(self, speed: f64, lead: Option<(f64, f64)>) -> Result<f64, PlanningError> {
        if ![
            self.desired_speed,
            self.time_headway,
            self.minimum_gap,
            self.max_acceleration,
            self.comfortable_braking,
            self.emergency_braking,
            speed,
        ]
        .iter()
        .all(|v| v.is_finite())
            || self.desired_speed <= 0.0
            || self.desired_speed > 60.0
            || self.time_headway < 0.5
            || self.minimum_gap <= 0.0
            || self.max_acceleration <= 0.0
            || self.comfortable_braking <= 0.0
            || self.emergency_braking < self.comfortable_braking
            || self.emergency_braking > 10.0
            || speed < 0.0
        {
            return Err(PlanningError::Invalid("IDM parameters"));
        }
        let interaction = if let Some((gap, lead_speed)) = lead {
            if !gap.is_finite() || !lead_speed.is_finite() || lead_speed < 0.0 {
                return Err(PlanningError::Invalid("IDM lead"));
            }
            if gap <= 0.0 {
                return Ok(-self.emergency_braking);
            }
            let desired = self.minimum_gap
                + (speed * self.time_headway
                    + speed * (speed - lead_speed)
                        / (2.0 * (self.max_acceleration * self.comfortable_braking).sqrt()))
                .max(0.0);
            (desired / gap).powi(2)
        } else {
            0.0
        };
        Ok(
            (self.max_acceleration * (1.0 - (speed / self.desired_speed).powi(4) - interaction))
                .clamp(-self.emergency_braking, self.max_acceleration),
        )
    }
}
pub fn curve_speed_limit(
    navigation: f64,
    map: f64,
    curvature: f64,
    loaded: bool,
) -> Result<f64, PlanningError> {
    if ![navigation, map, curvature].iter().all(|v| v.is_finite())
        || navigation <= 0.0
        || map <= 0.0
    {
        return Err(PlanningError::Invalid("speed limit"));
    }
    let lateral = if loaded { 1.2 } else { 1.8 };
    Ok(navigation
        .min(map)
        .min((lateral / curvature.abs().max(1e-9)).sqrt()))
}
/// Conservative front/rear gaps after braking for one reaction horizon.
pub fn lane_change_safe(
    ego_speed: f64,
    front: Option<(f64, f64)>,
    rear: Option<(f64, f64)>,
) -> bool {
    if !ego_speed.is_finite() || ego_speed < 0.0 {
        return false;
    }
    let safe = |pair: Option<(f64, f64)>, behind: bool| {
        pair.is_none_or(|(gap, speed)| {
            if !gap.is_finite() || !speed.is_finite() || gap < 0.0 || speed < 0.0 {
                return false;
            }
            let reaction = 1.5;
            let ego_travel = ego_speed * reaction;
            let other_travel = speed * reaction - 0.5 * 2.0 * reaction * reaction;
            let remaining = if behind {
                gap + ego_travel - other_travel.max(0.0)
            } else {
                gap + other_travel.max(0.0) - ego_travel
            };
            remaining
                >= 8.0
                    + 1.5
                        * if behind {
                            (speed - 3.0).max(0.0)
                        } else {
                            ego_speed
                        }
        })
    };
    safe(front, false) && safe(rear, true)
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn idm_brakes_for_stopped_lead() {
        let c = IdmConfig::default();
        assert_eq!(c.acceleration(25.0, Some((20.0, 0.0))).unwrap(), -4.0);
        assert!(c.acceleration(0.0, None).unwrap() > 0.0);
        assert!(c.acceleration(f64::NAN, None).is_err());
    }
    #[test]
    fn cutin_rejected() {
        assert!(!lane_change_safe(20.0, None, Some((10.0, 30.0))));
        assert!(lane_change_safe(
            20.0,
            Some((100.0, 20.0)),
            Some((100.0, 20.0))
        ));
    }
}
