extern crate nalgebra as na;
use na::{Isometry3, Point3, Vector3};

use crate::object;
use crate::ray::Ray;

use std::f64::consts::PI;

lazy_static! {
    pub static ref WORLD_UP: Vector3<f64> = Vector3::<f64>::new(0.0, 1.0, 0.0);
    pub static ref WORLD_RIGHT: Vector3<f64> = Vector3::<f64>::new(1.0, 0.0, 0.0);
}

pub fn onb(origin: &Point3<f64>, z: &Vector3<f64>) -> Isometry3<f64> {
    let v = (origin + z) - origin;
    let dot = WORLD_UP.dot(&v);
    let is_collinear = dot.abs() < 1.000001 && dot.abs() > 0.999999;

    if is_collinear {
        Isometry3::face_towards(&origin, &(origin + z), &WORLD_RIGHT).inverse()
    } else {
        Isometry3::face_towards(&origin, &(origin + z), &WORLD_UP).inverse()
    }
}

pub fn init_vectors(
    ray: &Ray,
    record: &object::IntersectionRecord,
) -> (
    Isometry3<f64>,
    Point3<f64>,
    Vector3<f64>,
    Vector3<f64>,
    Point3<f64>,
) {
    let o: Point3<f64> = (ray.origin.coords + record.t * ray.direction).into();
    let m = onb(&o, &record.normal);

    let p = m * o;
    let d = m * -ray.direction;
    let n = m * record.normal;

    (m, p, d, n, o)
}

pub fn reflect_onb(v: &Vector3<f64>) -> Vector3<f64> {
    Vector3::new(-v.x, -v.y, v.z)
}

pub fn uniform_hemisphere() -> Vector3<f64> {
    use rand::random;

    let theta = random::<f64>() * PI * 2.0;
    let phi = (1.0 - 1.0 * random::<f64>()).acos();

    Vector3::<f64>::new(phi.sin() * theta.cos(), phi.sin() * theta.sin(), phi.cos()).normalize()
}

pub fn sample_ray(m: &Isometry3<f64>, o: &Point3<f64>, l: &Vector3<f64>) -> Ray {
    Ray {
        origin: *o,
        direction: m.inverse_transform_vector(l),
    }
}
