extern crate nalgebra as na;
use na::{Isometry3, Point3, Vector3};

use crate::object;
use crate::ray::Ray;

lazy_static! {
    pub static ref WORLD_UP: Vector3<f64> = Vector3::<f64>::new(0.0, 1.0, 0.0);
}

pub fn onb(origin: &Point3<f64>, z: &Vector3<f64>) -> Isometry3<f64> {
    Isometry3::face_towards(&origin, &(origin + z), &WORLD_UP).inverse()
}

pub fn init_vectors(
    ray: &Ray,
    record: &object::IntersectionRecord,
) -> (Isometry3<f64>, Point3<f64>, Vector3<f64>, Vector3<f64>) {
    let o: Point3<f64> = (ray.origin.coords + record.t * ray.direction).into();
    let m = onb(&o, &record.normal);

    let p = m * o;
    let d = m * -ray.direction;
    let n = m * record.normal;

    (m, p, d, n)
}

pub fn reflect_onb(v: &Vector3<f64>) -> Vector3<f64> {
    Vector3::new(-v.x, -v.y, v.z)
}
