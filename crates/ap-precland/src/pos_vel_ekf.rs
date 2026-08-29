//! 1-D position/velocity EKF, upstream `libraries/AC_PrecLand/PosVelEKF`.
//!
//! Tracked as **COP-028**. Two instances (`_ekf_x`, `_ekf_y`) estimate
//! landing-target North and East relative to the vehicle. State is
//! `[position, velocity]`. Covariance is stored as the upper triangle
//! `[P00, P01, P11]`.

/// 1-D position/velocity EKF, upstream `PosVelEKF`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PosVelEKF {
    /// `_state[0]` position, `_state[1]` velocity.
    state: [f32; 2],
    /// `_cov[0]` = P00, `_cov[1]` = P01 = P10, `_cov[2]` = P11.
    cov: [f32; 3],
}

impl PosVelEKF {
    /// Zero state and covariance. Upstream the members are uninitialised
    /// until the first [`Self::init`].
    #[must_use]
    pub const fn new() -> Self {
        Self {
            state: [0.0, 0.0],
            cov: [0.0, 0.0, 0.0],
        }
    }

    /// `PosVelEKF::init`. First sighting or re-acquire after a loss.
    pub fn init(&mut self, pos: f32, pos_var: f32, vel: f32, vel_var: f32) {
        self.state[0] = pos;
        self.state[1] = vel;
        self.cov[0] = pos_var;
        self.cov[1] = 0.0;
        self.cov[2] = vel_var;
    }

    /// `PosVelEKF::predict`. Called at 400 Hz with `-vehicleDelVel`.
    ///
    /// `newState = [pos + dt*vel, vel + dVel]`. Process noise is
    /// `Q = diag(0, dVelNoise²)`.
    pub fn predict(&mut self, dt: f32, d_vel: f32, d_vel_noise: f32) {
        let new_state = [dt * self.state[1] + self.state[0], d_vel + self.state[1]];
        let new_cov = [
            dt * self.cov[1] + dt * (dt * self.cov[2] + self.cov[1]) + self.cov[0],
            dt * self.cov[2] + self.cov[1],
            d_vel_noise * d_vel_noise + self.cov[2],
        ];
        self.state = new_state;
        self.cov = new_cov;
    }

    /// `PosVelEKF::fusePos`. Direct position measurement, `H = [1 0]`.
    pub fn fuse_pos(&mut self, pos: f32, pos_var: f32) {
        let innovation_residual = pos - self.state[0];
        let innovation_covariance = self.cov[0] + pos_var;
        let new_state = [
            self.cov[0] * innovation_residual / innovation_covariance + self.state[0],
            self.cov[1] * innovation_residual / innovation_covariance + self.state[1],
        ];
        let new_cov = [
            self.cov[0] * pos_var / innovation_covariance,
            self.cov[1] * pos_var / innovation_covariance,
            -self.cov[1] * self.cov[1] / innovation_covariance + self.cov[2],
        ];
        self.state = new_state;
        self.cov = new_cov;
    }

    /// `PosVelEKF::getPos`.
    #[must_use]
    pub fn pos(&self) -> f32 {
        self.state[0]
    }

    /// `PosVelEKF::getVel`.
    #[must_use]
    pub fn vel(&self) -> f32 {
        self.state[1]
    }

    /// `PosVelEKF::getPosNIS`. `innovation² / (P00 + posVar)`.
    #[must_use]
    pub fn pos_nis(&self, pos: f32, pos_var: f32) -> f32 {
        let innovation_residual = pos - self.state[0];
        let innovation_covariance = self.cov[0] + pos_var;
        (innovation_residual * innovation_residual) / innovation_covariance
    }

    /// Upper-triangle covariance. Test / leftover inspection only.
    #[must_use]
    pub fn cov(&self) -> [f32; 3] {
        self.cov
    }
}

impl Default for PosVelEKF {
    fn default() -> Self {
        Self::new()
    }
}
