//! Persist learned hard-iron offsets, upstream `Compass::save_offsets`. FW-014.
//!
//! Learn latches `COMPASS_OFS` in RAM on the backend. Save copies those
//! offsets into the parameter table so they survive a reboot
//! (`_state[i].offset.save()` plus `dev_id` in upstream).

use ap_math::vector3::Vector3f;

use crate::params::CompassParams;
use crate::sitl::SitlCompassCluster;

/// Copy one instance offset into params (`COMPASS_OFS` / `OFS2`).
#[must_use]
pub fn save_instance_offset(params: &mut CompassParams, instance: u8, offset: Vector3f) -> bool {
    match instance {
        0 => {
            params.compass1.offset = offset;
            true
        }
        1 => {
            params.compass2.offset = offset;
            true
        }
        _ => false,
    }
}

/// Persist every registered backend offset into the param table.
///
/// Upstream `Compass::save_offsets()` walks all instances. Returns true when
/// at least one instance was written.
#[must_use]
pub fn save_offsets(params: &mut CompassParams, cluster: &SitlCompassCluster) -> bool {
    let mut any = false;
    for i in 0..cluster.instance_count() {
        if let Some(backend) = cluster.backend(i) {
            if save_instance_offset(params, i, backend.config().offset) {
                any = true;
            }
        }
    }
    any
}

/// True when params already hold the same offsets as the backends.
#[must_use]
pub fn offsets_already_saved(params: &CompassParams, cluster: &SitlCompassCluster) -> bool {
    if cluster.instance_count() == 0 {
        return false;
    }
    for i in 0..cluster.instance_count() {
        let Some(backend) = cluster.backend(i) else {
            return false;
        };
        let stored = if i == 0 {
            params.compass1.offset
        } else {
            params.compass2.offset
        };
        if stored != backend.config().offset {
            return false;
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sitl::{SitlCompassBackend, SitlCompassConfig};

    #[test]
    fn save_instance_writes_compass_ofs() {
        let mut params = CompassParams::default();
        let ofs = Vector3f::new(-0.05, 0.02, 0.01);
        assert!(save_instance_offset(&mut params, 0, ofs));
        assert_eq!(params.compass1.offset, ofs);
        assert_eq!(params.compass2.offset, Vector3f::zero());
        assert!(!save_instance_offset(&mut params, 2, ofs));
    }

    #[test]
    fn save_offsets_copies_cluster_into_params() {
        let mut cluster = SitlCompassCluster::default();
        let ofs = Vector3f::new(-0.04, 0.0, 0.01);
        cluster.backend_mut(0).unwrap().set_config(SitlCompassConfig {
            offset: ofs,
            ..SitlCompassConfig::default()
        });
        let mut params = CompassParams::default();
        assert!(!offsets_already_saved(&params, &cluster));
        assert!(save_offsets(&mut params, &cluster));
        assert_eq!(params.compass1.offset, ofs);
        assert!(offsets_already_saved(&params, &cluster));
    }

    #[test]
    fn saved_params_restore_offset_on_fresh_backend() {
        let ofs = Vector3f::new(-0.05, 0.02, 0.0);
        let mut params = CompassParams::default();
        assert!(save_instance_offset(&mut params, 0, ofs));
        let mut backend = SitlCompassBackend::default();
        params.apply_instance(0, &mut backend);
        assert_eq!(backend.config().offset, ofs);
    }
}
