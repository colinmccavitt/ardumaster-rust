//! Host-supplied INS recording buffers for SITL file playback runtime.

use ap_ins::sitl::{SitlInsInstanceFiles, SITL_INS_MAX_INSTANCES};

pub const SITL_INS_HOST_FILE_CAP: usize = 512;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SitlInsHostFiles {
    pub accel: [u8; SITL_INS_HOST_FILE_CAP],
    pub accel_len: usize,
    pub gyro: [u8; SITL_INS_HOST_FILE_CAP],
    pub gyro_len: usize,
}

impl Default for SitlInsHostFiles {
    fn default() -> Self {
        Self {
            accel: [0; SITL_INS_HOST_FILE_CAP],
            accel_len: 0,
            gyro: [0; SITL_INS_HOST_FILE_CAP],
            gyro_len: 0,
        }
    }
}

impl SitlInsHostFiles {
    #[must_use]
    pub fn as_instance_files(&self) -> SitlInsInstanceFiles<'_> {
        SitlInsInstanceFiles {
            accel: (self.accel_len > 0).then(|| &self.accel[..self.accel_len]),
            gyro: (self.gyro_len > 0).then(|| &self.gyro[..self.gyro_len]),
        }
    }
}

pub fn sitl_ins_host_files_fill<'a>(
    host: &'a [SitlInsHostFiles; SITL_INS_MAX_INSTANCES],
    count: u8,
    out: &'a mut [SitlInsInstanceFiles<'a>; SITL_INS_MAX_INSTANCES],
) -> &'a [SitlInsInstanceFiles<'a>] {
    let n = count as usize;
    for i in 0..n {
        out[i] = host[i].as_instance_files();
    }
    &out[..n]
}

#[cfg(test)]
mod tests {
    use super::*;
    use ap_math::vector3::Vector3f;

    fn encode(v: Vector3f) -> [u8; 12] {
        let mut out = [0_u8; 12];
        for (i, component) in [v.x, v.y, v.z].into_iter().enumerate() {
            out[i * 4..i * 4 + 4].copy_from_slice(&component.to_le_bytes());
        }
        out
    }

    #[test]
    fn host_files_expose_nonempty_accel_slice() {
        let mut host = SitlInsHostFiles::default();
        let frame = encode(Vector3f::new(0.0, 0.0, -4.0));
        host.accel[..12].copy_from_slice(&frame);
        host.accel_len = 12;
        let view = host.as_instance_files();
        assert_eq!(view.accel.unwrap().len(), 12);
        assert!(view.gyro.is_none());
    }
}
