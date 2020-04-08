extern crate nalgebra as na;
use na::Point3;
use na::Vector3;

#[derive(Clone, Copy)]
pub struct Ray {
    pub origin: Point3<f64>,
    pub direction: Vector3<f64>,
}
