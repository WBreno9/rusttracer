extern crate nalgebra as na;
use na::{Isometry3, Point3, Vector3};

// use crate::brdf::{BRDFInput, BRDF};

use crate::sample;

use rand::random;
use std::f64::consts::PI;

pub trait Light {
    fn sample_point(&self) -> (Point3<f64>, f64);

    fn shade(&self, m: &Isometry3<f64>, p: &Point3<f64>, v: &Vector3<f64>) -> Vector3<f64>;
}

pub struct DiskLight {
    pub pos: Point3<f64>,
    pub color: Vector3<f64>,
    pub power: f64,
    pub radius: f64,
    pub normal: Vector3<f64>,
}

impl Light for DiskLight {
    fn sample_point(&self) -> (Point3<f64>, f64) {
        let sm = sample::onb(&self.pos, &self.normal);

        let theta = 2.0 * PI * random::<f64>();
        let r = self.radius * random::<f64>();

        (
            sm.inverse_transform_point(&Point3::<f64>::new(
                r.sqrt() * theta.cos(),
                r.sqrt() * theta.sin(),
                0.0,
            )),
            1.0 / std::f64::consts::PI * self.radius * self.radius,
        )
    }

    fn shade(&self, m: &Isometry3<f64>, _: &Point3<f64>, v: &Vector3<f64>) -> Vector3<f64> {
        (m * self.normal).dot(&-v).max(0.0).min(1.0) * self.color * self.power
    }
}
