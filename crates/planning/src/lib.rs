//! Rules, sampling and parking planners. All planners fail closed on bad data.
pub mod behavior;
pub mod frenet;
pub mod geometry;
pub mod ilqr;
pub mod parking;
pub mod speed;
#[derive(Debug, thiserror::Error)]
pub enum PlanningError {
    #[error("invalid planning input: {0}")]
    Invalid(&'static str),
    #[error("no safe trajectory within search budget")]
    NoPath,
    #[error(transparent)]
    Map(#[from] etsads_map::MapError),
    #[error(transparent)]
    Control(#[from] etsads_control::ControlError),
}
