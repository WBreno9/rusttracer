extern crate nalgebra as na;
use na::Vector3;

use crate::primitive::{AggregatePrimitive, IntersectionRecord, Primitive, Triangle};

use crate::ray::Ray;

#[derive(Clone)]
struct Bounds {
    max: Vector3<f64>,
    min: Vector3<f64>,
}

impl Bounds {
    fn intersect(&self, ray: &Ray) -> bool {
        let inv = Vector3::repeat(1.000001).component_div(&ray.direction);

        let t0 = (self.min - ray.origin.coords).component_mul(&inv);
        let t1 = (self.max - ray.origin.coords).component_mul(&inv);

        let vmin = Vector3::new(t0.x.min(t1.x), t0.y.min(t1.y), t0.z.min(t1.z));

        let vmax = Vector3::new(t0.x.max(t1.x), t0.y.max(t1.y), t0.z.max(t1.z));

        let tmin = vmin.x.max(vmin.y.max(vmin.z));
        let tmax = vmax.x.min(vmax.y.min(vmax.z));

        tmin < tmax
    }
}

enum Node {
    Internal(InternalNode),
    Leaf(LeafNode),
}

impl Node {
    fn intersect(&self, ray: &Ray) -> Option<IntersectionRecord> {
        match self {
            Node::Internal(node) => {
                let left_hit = if node.left_bounds.intersect(&ray) {
                    node.left.intersect(&ray)
                } else {
                    None
                };

                let right_hit = if node.right_bounds.intersect(&ray) {
                    node.right.intersect(&ray)
                } else {
                    None
                };

                if left_hit.is_some() && right_hit.is_some() {
                    let l = left_hit.unwrap();
                    let r = right_hit.unwrap();

                    if l.t < r.t {
                        Some(l)
                    } else {
                        Some(r)
                    }
                } else {
                    if left_hit.is_some() {
                        left_hit
                    } else {
                        right_hit
                    }
                }
            }

            Node::Leaf(leaf) => {
                let mut closest: Option<IntersectionRecord> = None;

                for tri_ref in leaf.refs.iter() {
                    if let Some(record) = tri_ref.tri_ref.intersect(ray) {
                        closest = match closest {
                            Some(old_record) if old_record.t > record.t => Some(record),
                            Some(_) => closest,
                            None => Some(record),
                        };
                    }
                }

                closest
            }
        }
    }

    fn intersect_debug(&self, ray: &Ray) -> Vector3<f64> {
        match self {
            Node::Internal(node) => {
                let mut res: Vector3<f64> = Vector3::repeat(0.0);

                if node.left_bounds.intersect(&ray) {
                    res += Vector3::new(0.0, 0.0, 0.01) + node.left.intersect_debug(&ray);
                };

                if node.right_bounds.intersect(&ray) {
                    res += Vector3::new(0.0, 0.0, 0.01) + node.right.intersect_debug(&ray);
                };

                res
            }

            Node::Leaf(leaf) => {
                let mut res: Vector3<f64> = Vector3::repeat(0.0);

                for tri_ref in leaf.refs.iter() {
                    if let Some(_) = tri_ref.tri_ref.intersect(ray) {
                        res += Vector3::new(0.1, 0.0, 0.0);
                    }

                    if tri_ref.bounds.intersect(ray) {
                        res += Vector3::new(0.0, 0.1, 0.0);
                    }
                }

                res
            }
        }
    }
}

pub struct Tree {
    root: Node,
}

impl Tree {
    pub fn new(mesh: AggregatePrimitive<Triangle>) -> Tree {
        Tree {
            root: build_node(build_refs(mesh)),
        }
    }

    pub fn debug(&self, ray: &Ray) -> Vector3<f64> {
        self.root.intersect_debug(ray)
    }
}

impl Primitive for Tree {
    fn intersect(&self, ray: &Ray) -> Option<IntersectionRecord> {
        self.root.intersect(ray)
    }
}

struct InternalNode {
    left_bounds: Bounds,
    right_bounds: Bounds,
    left: Box<Node>,
    right: Box<Node>,
}

#[derive(Clone)]
struct TriangleRef {
    bounds: Bounds,
    tri_ref: Triangle,
}

struct LeafNode {
    refs: Vec<TriangleRef>,
}

#[derive(Clone, Copy)]
enum SortAxis {
    X = 0,
    Y = 1,
    Z = 2,
}

fn triangle_bounds(tri: &Triangle) -> Bounds {
    Bounds {
        max: Vector3::new(
            tri.vert[0]
                .pos
                .x
                .max(tri.vert[1].pos.x.max(tri.vert[2].pos.x)),
            tri.vert[0]
                .pos
                .y
                .max(tri.vert[1].pos.y.max(tri.vert[2].pos.y)),
            tri.vert[0]
                .pos
                .z
                .max(tri.vert[1].pos.z.max(tri.vert[2].pos.z)),
        ) + Vector3::repeat(0.00000001),
        min: Vector3::new(
            tri.vert[0]
                .pos
                .x
                .min(tri.vert[1].pos.x.min(tri.vert[2].pos.x)),
            tri.vert[0]
                .pos
                .y
                .min(tri.vert[1].pos.y.min(tri.vert[2].pos.y)),
            tri.vert[0]
                .pos
                .z
                .min(tri.vert[1].pos.z.min(tri.vert[2].pos.z)),
        ) - Vector3::repeat(0.00000001),
    }
}

fn refs_bounds(refs: &[TriangleRef]) -> Bounds {
    let mut a = refs[0].bounds.clone();

    for tri_ref in refs[1..refs.len()].iter() {
        let b = &tri_ref.bounds;

        a.min = Vector3::new(
            a.min.x.min(b.min.x),
            a.min.y.min(b.min.y),
            a.min.z.min(b.min.z),
        );

        a.max = Vector3::new(
            a.max.x.max(b.max.x),
            a.max.y.max(b.max.y),
            a.max.z.max(b.max.z),
        );
    }

    a
}

fn build_refs(mesh: AggregatePrimitive<Triangle>) -> Vec<TriangleRef> {
    let mut refs: Vec<TriangleRef> = vec![];

    for prim in mesh.primitives {
        refs.push(TriangleRef {
            bounds: triangle_bounds(&prim),
            tri_ref: prim,
        })
    }

    refs
}

fn sort_refs(refs: &mut [TriangleRef], axis: SortAxis) {
    refs.sort_unstable_by(|a, b| {
        let c0 = (a.bounds.max + a.bounds.min) / 2.0;
        let c1 = (b.bounds.max + b.bounds.min) / 2.0;

        c0[axis as usize].partial_cmp(&c1[axis as usize]).unwrap()
    });
}

fn build_node<'a>(mut refs: Vec<TriangleRef>) -> Node {
    if refs.len() <= 3 {
        return Node::Leaf(LeafNode { refs });
    }

    let bounds = refs_bounds(&refs);
    let v = bounds.max - bounds.min;

    let sort_axis = if v.x > v.y && v.x > v.z {
        SortAxis::X
    } else if v.y > v.x && v.y > v.z {
        SortAxis::Y
    } else {
        SortAxis::Z
    };

    sort_refs(&mut refs[..],  sort_axis);

    let middle = (refs.len() as f64 / 2.0).floor() as usize;

    let left_refs = refs[0..middle].to_vec();
    let right_refs = refs[middle..refs.len()].to_vec();

    return Node::Internal(InternalNode {
        left_bounds: refs_bounds(&left_refs),
        right_bounds: refs_bounds(&right_refs),
        left: Box::new(build_node(left_refs)),
        right: Box::new(build_node(right_refs)),
    });
}
