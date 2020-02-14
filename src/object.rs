extern crate nalgebra as na;
use na::Vector3;

use crate::primitive;
use crate::ray::Ray;
use crate::brdf::*;

pub struct IntersectionRecord<'a> {
    pub t: f32,
    pub normal: Vector3<f32>,
    pub brdf: &'a Box<dyn BRDF>,
}

pub trait Intersect {
    fn intersect(&self, ray: &Ray) -> Option<IntersectionRecord>;
}

pub struct Object<T: primitive::Primitive> {
    primitive: T,
    brdf: Box<dyn BRDF>,
}

impl<T: primitive::Primitive> Object<T> {
    pub fn new(primitive: T) -> Object<T> {
        Object {
            primitive,
            brdf: Box::new(MicrofacetBRDF::new(
                    Vector3::<f32>::new(0.9, 0.3, 0.2),
                    Vector3::<f32>::new(0.58, 0.58, 0.58),
                    0.27,
                    0.8,
                    ))}
    }
}

impl<T: primitive::Primitive> Intersect for Object<T> {
    fn intersect(&self, ray: &Ray) -> Option<IntersectionRecord> {
        if let Some(intersect_prim) = self.primitive.intersect(&ray) {
            Some(IntersectionRecord {
                t: intersect_prim.t,
                normal: intersect_prim.normal,
                brdf: &self.brdf,
            })
        } else {
            None
        }
    }
}

pub struct AggregateObject {
    pub primitives: Vec<Box<dyn Intersect>>,
}

impl AggregateObject {
    pub fn new() -> AggregateObject {
        AggregateObject {
            primitives: Vec::<Box<dyn Intersect>>::new(),
        }
    }
}

impl Intersect for AggregateObject {
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
