extern crate nalgebra as na;
use na::Vector3;
use na::Vector2;
use na::geometry::{Isometry3, Point3};

use crate::primitive::{Ray};

pub struct PerspectiveCamera {
    pub isometry: Isometry3<f32>,
}

impl PerspectiveCamera {
    pub fn new() -> PerspectiveCamera {
        PerspectiveCamera {
            isometry: Isometry3::<f32>::identity(),
        }
    }

    pub fn get_ray(&self, p: Vector2<f32>) -> Ray {
        Ray {
            origin: self.isometry.inverse_transform_point(&Point3::origin()),
            direction: self.isometry.inverse_transform_vector(
                &Vector3::<f32>::new(p[0], p[1], 1.0).normalize()),
        }
    }
}
