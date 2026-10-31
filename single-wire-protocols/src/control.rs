//! Shared output-request and control errors for SCP, FCP and AFC sessions.

use crate::Error;

#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum RequestError {
    #[error("complete adapter ratings are unavailable")]
    MissingRatings,
    #[error("output request is outside the advertised range")]
    Range,
    #[error("output request is not representable at the advertised step")]
    Step,
    #[error("output request exceeds the advertised power")]
    Power,
    #[error("a previous output procedure is still running")]
    Busy,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum ControlError<E> {
    #[error("single-wire control transaction failed: {0}")]
    Wire(Error<E>),
    #[error("invalid output request: {0}")]
    Request(RequestError),
    #[error("output control readback mismatch")]
    Readback,
    #[error("output protection or watchdog reset")]
    Protection,
}
