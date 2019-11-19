extern crate nalgebra as na;
use na::Vector3;

use crate::brdf::BRDF;

pub struct Ray {
    pub origin: Vector3<f32>,
    pub direction: Vector3<f32>,
}

pub struct IntersectionRecord<'a> {
    pub t: f32,
    pub pos: Vector3<f32>,
    pub normal: Vector3<f32>,
    pub brdf: &'a Box<dyn BRDF>,
}

pub trait Intersect {
    fn intersect(&self, ray: &Ray) -> Option<IntersectionRecord>;
}
