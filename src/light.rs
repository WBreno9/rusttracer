extern crate nalgebra as na;
use na::{Vector3, Isometry3, Point3};

use crate::brdf::{BRDF, BRDFInput};

pub trait Light {
  fn sample(&self, m: &Isometry3<f64>, n: &Vector3<f64>, v: &Vector3<f64>, p: &Point3<f64>, brdf: &Box<dyn BRDF>) -> Vector3<f64>;
}

pub struct PointLight {
  pub pos: Point3<f64>,
  pub color: Vector3<f64>,
  pub power: f64
}

impl Light for PointLight {
  fn sample(&self, m: &Isometry3<f64>, n: &Vector3<f64>, v: &Vector3<f64>, p: &Point3<f64>, brdf: &Box<dyn BRDF>) -> Vector3<f64> {
        let lp = m * self.pos;
        let l = (lp - p).normalize();
        let d = (lp - p).norm_squared();
        let lc = self.color * self.power;
        let lf = brdf.f(&BRDFInput::new(&n, &l, &v));

        let attenuation = 1.0 / d;

        let c = lc * attenuation;

        lf.component_mul(&c) * n.dot(&l).max(0.0).min(1.0)
  }
}