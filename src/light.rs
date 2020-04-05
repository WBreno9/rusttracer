extern crate nalgebra as na;
use na::{Isometry3, Point3, Vector3};

use crate::brdf::{BRDFInput, BRDF};

use crate::sample;

use rand::random;
use std::f64::consts::PI;

pub trait Light {
    fn sample_point(&self) -> Point3<f64>;

    fn shade(
        &self,
        m: &Isometry3<f64>,
        lp: &Point3<f64>,
        n: &Vector3<f64>,
        v: &Vector3<f64>,
        p: &Point3<f64>,
        brdf: &Box<dyn BRDF>
    ) -> Vector3<f64>;
}

// pub struct PointLight {
//     pub pos: Point3<f64>,
//     pub color: Vector3<f64>,
//     pub power: f64,
// }
// 
// impl Light for PointLight {
//     fn sample(
//         &self,
//         m: &Isometry3<f64>,
//         n: &Vector3<f64>,
//         v: &Vector3<f64>,
//         p: &Point3<f64>,
//         brdf: &Box<dyn BRDF>,
//     ) -> Vector3<f64> {
//         let lp = m * self.pos;
//         let l = (lp - p).normalize();
//         let d = (lp - p).norm_squared();
//         let lc = self.color * self.power;
//         let lf = brdf.f(&BRDFInput::new(&n, &l, &v));
// 
//         let attenuation = 1.0 / d;
// 
//         let c = lc * attenuation;
// 
//         lf.component_mul(&c) * n.dot(&l).max(0.0).min(1.0)
//     }
// }

pub struct DiskLight {
    pub pos: Point3<f64>,
    pub color: Vector3<f64>,
    pub power: f64,
    pub radius: f64,
    pub normal: Vector3<f64>,
}

impl Light for DiskLight {
    fn sample_point(&self) -> Point3<f64> {

        let sm = sample::onb(&self.pos, &self.normal);

        let theta = 2.0 * PI * random::<f64>();
        let r = self.radius * random::<f64>();

        sm.inverse_transform_point(&Point3::<f64>::new(
            r * theta.cos(),
            r * theta.sin(),
            0.0
        ))
    }

    fn shade(
        &self,
        m: &Isometry3<f64>,
        lp: &Point3<f64>,
        n: &Vector3<f64>,
        v: &Vector3<f64>,
        p: &Point3<f64>,
        brdf: &Box<dyn BRDF>,
    ) -> Vector3<f64> {
        let l = (lp - p).normalize();
        let lc = self.color * self.power;
        let lf = brdf.f(&BRDFInput::new(&n, &l, &v));


        if (m * self.normal).dot(&l) < 0.0 {
            lf.component_mul(&lc) / (PI * self.radius * self.radius)
        } else {
            Vector3::zeros()
        }
    }
}
