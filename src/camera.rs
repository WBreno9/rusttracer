extern crate nalgebra as na;
use na::Vector3;
use na::Vector2;

use crate::ray::*;

pub trait Camera {
    fn get_ray(&self, p: Vector2<f32>) -> Ray;
}

pub struct PerspectiveCamera {
    pub pos: Vector3<f32>,
    pub forward: Vector3<f32>,
    pub right: Vector3<f32>,
    pub up: Vector3<f32>,
}

impl PerspectiveCamera {
    pub fn new() -> PerspectiveCamera {
        PerspectiveCamera {
            pos: Vector3::new(0.0, 0.0, 0.0),
            forward: Vector3::new(0.0, 0.0, 1.0),
            right: Vector3::new(1.0, 0.0, 0.0),
            up: Vector3::new(0.0, 1.0, 0.0),
        }
    }
}

impl Camera for PerspectiveCamera {
    fn get_ray(&self, p: Vector2<f32>) -> Ray {
        let origin = self.pos;
        let direction: Vector3<f32>;

        direction = self.forward + self.right * p[0] + self.up * p[1];

        Ray {
            origin,
            direction: direction.normalize(),
        }
    }
}

pub struct OrthographicCamera {
    pub pos: Vector3<f32>,
    pub forward: Vector3<f32>,
    pub right: Vector3<f32>,
    pub up: Vector3<f32>,
}

impl OrthographicCamera {
    pub fn new() -> OrthographicCamera {
        OrthographicCamera {
            pos: Vector3::new(0.0, 0.0, 0.0),
            forward: Vector3::new(0.0, 0.0, 1.0),
            right: Vector3::new(1.0, 0.0, 0.0),
            up: Vector3::new(0.0, 1.0, 0.0),
        }
    }
}

impl Camera for OrthographicCamera {
    fn get_ray(&self, p: Vector2<f32>) -> Ray {
        let origin = self.right * p[0] + self.up * p[1];
        let direction = origin + self.forward;

        Ray {
            origin,
            direction: direction.normalize(),
        }
    }
}

