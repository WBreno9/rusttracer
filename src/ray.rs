extern crate nalgebra as na;
use na::Point3;
use na::Vector3;

pub struct Ray {
    pub origin: Point3<f32>,
    pub direction: Vector3<f32>,
}

