extern crate nalgebra as na;
use na::Vector3;

use crate::ray::*;

pub struct IntersectionPrimitive {
    pub t: f32,
    pub pos: Vector3<f32>,
    pub normal: Vector3<f32>,
}

pub trait IntersectPrimitive {
    fn intersect(&self, ray: &Ray) -> Option<IntersectionPrimitive>;
}

pub struct AggregatePrimitive<T: IntersectPrimitive> {
    pub primitives: Vec<T>,
}

impl<T: IntersectPrimitive> AggregatePrimitive<T> {
    pub fn new() -> AggregatePrimitive<T> {
        AggregatePrimitive::<T> {
            primitives: Vec::<T>::new(),
        }
    }
}

impl<T: IntersectPrimitive> IntersectPrimitive for AggregatePrimitive<T> {
    fn intersect(&self, ray: &Ray) -> Option<IntersectionPrimitive> {
        let mut closest: Option<IntersectionPrimitive> = None;

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
    pub pos: Vector3<f32>,
    pub radius: f32,
}

impl Sphere {
    pub fn new(pos: Vector3<f32>, radius: f32) -> Sphere {
        Sphere {
            pos, 
            radius,
        }
    }
}

impl IntersectPrimitive for Sphere {
    fn intersect(&self, ray: &Ray) -> Option<IntersectionPrimitive> {
        let l = self.pos - ray.origin;
        let tca = l.dot(&ray.direction);
        let d2 = l.dot(&l) - tca*tca;

        let radius2 = self.radius*self.radius;

        let thc = (radius2 - d2).sqrt();
        let mut t = tca - thc;
        let t1 = tca + thc;

        t = t.min(t1);

        if d2 > radius2 || t < 0.0 {
            None
        } else {
            let pos = ray.origin + t*ray.direction;
            let normal = (pos - self.pos).normalize();

            Some(IntersectionPrimitive {
                t,
                pos,
                normal,
            })
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Vertex {
    pub pos: Vector3<f32>,
    // pub nrm: Vector3<f32>,
    // pub tcd: Vector3<f32>,
}

pub struct Triangle {
    pub vert: Vec<Vertex>,
}

impl Triangle {
    pub fn new(v: &Vec<Vertex>) -> Triangle {
        debug_assert!(v.len() == 3);

        Triangle {
            vert: v.clone(),
        }
    }
}

impl IntersectPrimitive for Triangle {
    fn intersect(&self, ray: &Ray) -> Option<IntersectionPrimitive> {
        let oa = ray.origin - self.vert[0].pos;
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
            let normal = e1.cross(&e2).normalize();
            Some(IntersectionPrimitive {
                t,
                pos: ray.direction + t*ray.origin,
                normal,
            })
        }
    }
}
