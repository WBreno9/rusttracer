extern crate image;

extern crate nalgebra as na;
use na::Point3;
use na::Vector3;
use na::Vector2;

pub mod camera;
use crate::camera::{PerspectiveCamera};

mod brdf;
use crate::brdf::{BRDFInput, MicrofacetBRDF};

pub mod object;
use crate::object::*;

pub mod primitive;
use crate::primitive::*;

pub mod ray;

extern crate obj;

pub mod mesh;

use std::time::Instant;

use na::Isometry3;

use crate::ray::Ray;

#[macro_use]
extern crate lazy_static;

lazy_static! {
    static ref WORLD_UP: Vector3<f32> = Vector3::<f32>::new(0.0, 1.0, 0.0);
}

fn onb(origin: &Point3<f32>, z: &Vector3<f32>) -> Isometry3<f32> {
        Isometry3::face_towards(&origin, &(origin+z), &WORLD_UP).inverse()
}

fn init_vectors(ray: &Ray, record: &object::IntersectionRecord) ->
    (Isometry3<f32>, Point3<f32>, Vector3<f32>, Vector3<f32>) {
        let o: Point3<f32> = (ray.origin.coords + record.t*ray.direction).into();
        let m = onb(&o, &record.normal);

        let p = m * o;
        let d = m * -ray.direction;
        let n = m * record.normal;

        (m, p.into(), d, n)
}

fn radiance(ray: &Ray, scene: &dyn object::Intersect) -> Vector3<f32> {
    if let Some(record) = scene.intersect(&ray) {
        let (m, p, d, n) = init_vectors(&ray, &record);

        let lp = m * Point3::new(0.0, 1.8, 0.0);
        let lv = lp - p;
        let dist2 = lv.dot(&lv);

        let attenuation = 4.0 / dist2;

        let input = BRDFInput {
            l: &lv.normalize(),
            n: &n,
            v: &d,
        };

        record.brdf.f(&input) * attenuation
    } else {
        Vector3::repeat(0.0)
    }
}

fn main() {
    let width = 128;
    let height = width;

    let mut im = image::RgbImage::new(width, height);
    let (im_width, im_height) = im.dimensions();

    let ratio = width as f32/height as f32;

    let mut camera = PerspectiveCamera::new();
    camera.isometry = na::geometry::Isometry3::look_at_lh(
        &Vector3::new(0.0, 0.0, 2.0).into(),
        &Point3::origin(),
        &Vector3::y_axis(),
        );

    // let mut mesh = Object::<AggregatePrimitive<Triangle>>::new(
    //     mesh::load_mesh_aggregate("tests/bunny.obj").unwrap());

    let box_size = 2.0;
    let mut scene = cornell_box(box_size);

    scene.primitives.push(Box::new(Object::new(Sphere::new(
        Point3::new(-0.55, -box_size+0.6, -box_size+0.5), 0.6))));

    let start = Instant::now();

    for i in 0..im_width {
        for j in 0..im_height {
            let pixel = im.get_pixel_mut(i, j);

            let screen_space = Vector2::<f32>::new(
                ((i as f32/width as f32) - 0.5) * 2.0 * ratio,
                -((j as f32/height as f32) - 0.5) * 2.0,
            );

            let ray = camera.get_ray(screen_space);

            let c = radiance(&ray, &scene);

            pixel[0] = (c[0].min(1.0) * 255.0) as u8;
            pixel[1] = (c[1].min(1.0) * 255.0) as u8;
            pixel[2] = (c[2].min(1.0) * 255.0) as u8;
        }
    }

    println!("Execution time: {:?}", start.elapsed());

    im.save("test_image.ppm").unwrap();
}

fn cornell_box(box_size: f32) -> AggregateObject {
    let mut scene = AggregateObject::new();

    let plane_constant : f32 = 1000.0;

    let ceiling_floor_brdf = Box::new(MicrofacetBRDF {
         albedo: Vector3::<f32>::new(1.0, 1.0, 1.0),
         f0: Vector3::<f32>::new(1.0, 1.0, 1.0),
         roughness: 1.0, 
         specular: 0.0,
    });

    let mut ground = Object::new(Sphere::new(
        Point3::new(0.0, -plane_constant-box_size, 0.0), plane_constant));
    ground.brdf = ceiling_floor_brdf.clone();
    let mut ceiling = Object::new(Sphere::new(
        Point3::new(0.0, plane_constant+box_size, 0.0), plane_constant));
    ceiling.brdf = ceiling_floor_brdf.clone();
    let mut back_wall = Object::new(Sphere::new(
        Point3::new(0.0, 0.0, -plane_constant-box_size), plane_constant));
    back_wall.brdf = ceiling_floor_brdf.clone();

    let mut left_wall = Object::new(Sphere::new(
        Point3::new(plane_constant+box_size, 0.0, 0.0), plane_constant));
    left_wall.brdf = Box::new(MicrofacetBRDF {
         albedo: Vector3::<f32>::new(0.0, 1.0, 0.0),
         f0: Vector3::<f32>::new(1.0, 1.0, 1.0),
         roughness: 1.0, 
         specular: 0.0,
    });

    let mut right_wall = Object::new(Sphere::new(
        Point3::new(-plane_constant-box_size, 0.0, 0.0), plane_constant));
    right_wall.brdf = Box::new(MicrofacetBRDF {
         albedo: Vector3::<f32>::new(1.0, 0.0, 0.0),
         f0: Vector3::<f32>::new(1.0, 1.0, 1.0),
         roughness: 1.0, 
         specular: 0.0,
    });

    scene.primitives.push(Box::new(ground));
    scene.primitives.push(Box::new(ceiling));
    scene.primitives.push(Box::new(back_wall));
    scene.primitives.push(Box::new(left_wall));
    scene.primitives.push(Box::new(right_wall));

    scene
}
