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

pub struct SampleRecord {
    pub o: Point3<f64>,
    pub on: Vector3<f64>,
    pub m: Isometry3<f64>,
    pub n: Vector3<f64>,
    pub v: Vector3<f64>,
    pub p: Point3<f64>,
}

impl SampleRecord {
    pub fn new(ray: &Ray, record: &object::IntersectionRecord) -> SampleRecord {
        let o: Point3<f64> = (ray.origin.coords + record.t * ray.direction).into();
        let m = onb(&o, &record.normal);

        let p = m * o;
        let v = m * -ray.direction;
        let n = Vector3::new(0.0, 0.0, 1.0);

        SampleRecord {
            o,
            on: record.normal,
            m,
            n,
            v,
            p,
        }
    }
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
