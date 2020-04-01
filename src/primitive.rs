extern crate nalgebra as na;
use na::Point3;
use na::Vector3;

use crate::ray::Ray;

pub struct IntersectionRecord {
    pub t: f64,
    pub normal: Vector3<f64>,
}

pub trait Primitive {
    fn intersect(&self, ray: &Ray) -> Option<IntersectionRecord>;
}

pub struct AggregatePrimitive<T: Primitive> {
    pub primitives: Vec<T>,
}

impl<T: Primitive> AggregatePrimitive<T> {
    pub fn new() -> AggregatePrimitive<T> {
        AggregatePrimitive::<T> {
            primitives: Vec::<T>::new(),
        }
    }
}

impl<T: Primitive> Primitive for AggregatePrimitive<T> {
    fn intersect(&self, ray: &Ray) -> Option<IntersectionRecord> {
        let mut closest: Option<IntersectionRecord> = None;

        for primitive in self.primitives.iter() {
            if let Some(record) = primitive.intersect(&ray) {
                closest = match &closest {
                    Some(old_record) if old_record.t > record.t => Some(record),
                    Some(_) => closest,
                    None => Some(record),
                }
            }
        }

        closest
    }
}

pub struct Sphere {
    pub pos: Point3<f64>,
    pub radius: f64,
}

impl Sphere {
    pub fn new(pos: Point3<f64>, radius: f64) -> Sphere {
        Sphere { pos, radius }
    }
}

impl Primitive for Sphere {
    fn intersect(&self, ray: &Ray) -> Option<IntersectionRecord> {
        let l = self.pos - ray.origin;
        let tca = l.dot(&ray.direction);
        let d2 = l.dot(&l) - tca * tca;

        let radius2 = self.radius * self.radius;

        let thc = (radius2 - d2).sqrt();

        let t0 = tca - thc;
        let t1 = tca + thc;

        let mut t = t1.min(t0);

        if t0 < std::f32::EPSILON.into() && t1 > std::f32::EPSILON.into() {
            t = t1;

            let pos = ray.origin.coords + t * ray.direction;
            let normal = (self.pos.coords - pos).normalize();

            return Some(IntersectionRecord { t, normal });
        }

        if d2 > radius2 || t < std::f32::EPSILON.into() {
            None
        } else {
            let pos = ray.origin.coords + t * ray.direction;
            let normal = (pos - self.pos.coords).normalize();

            Some(IntersectionRecord { t, normal })
        }
    }
}

pub struct Plane {
    pub pos: Point3<f64>,
    pub nrm: Vector3<f64>,
}

impl Primitive for Plane {
    fn intersect(&self, ray: &Ray) -> Option<IntersectionRecord> {
        let denom = (-self.nrm).dot(&ray.direction);
        if denom > 1e-6 {
            let v = self.pos - ray.origin;
            let t = v.dot(&-self.nrm) / denom;

            if t >= 0.0 {
                return Some(IntersectionRecord { t, normal: self.nrm });
            }
        }
        None
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Vertex {
    pub pos: Vector3<f64>,
    pub nrm: Vector3<f64>,
    // pub tcd: Vector3<f64>,
}

pub struct Triangle {
    pub vert: Vec<Vertex>,
}

impl Triangle {
    pub fn new(v: &Vec<Vertex>) -> Triangle {
        debug_assert!(v.len() == 3);

        Triangle { vert: v.clone() }
    }
}

impl Primitive for Triangle {
    fn intersect(&self, ray: &Ray) -> Option<IntersectionRecord> {
        let oa = ray.origin.coords - self.vert[0].pos;
        let e1 = self.vert[1].pos - self.vert[0].pos;
        let e2 = self.vert[2].pos - self.vert[0].pos;

        let p = ray.direction.cross(&e2);
        let q = oa.cross(&e1);

        let inv = 1.0 / p.dot(&e1);

        let t = inv * q.dot(&e2);
        let u = inv * p.dot(&oa);
        let v = inv * q.dot(&ray.direction);

        if u < 0.0 || u > 1.0 || v < 0.0 || u + v > 1.0 {
            None
        } else {
            // let normal = e1.cross(&e2).normalize();
            let w = 1.0 - u - v;

            let normal =
                (w * self.vert[0].nrm + u * self.vert[1].nrm + v * self.vert[2].nrm).normalize();

            Some(IntersectionRecord { t, normal })
        }
    }
}
