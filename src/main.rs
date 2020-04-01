extern crate image;

extern crate nalgebra as na;
use na::Point3;
use na::Vector2;
use na::Vector3;

pub mod camera;
use crate::camera::PerspectiveCamera;

mod brdf;
use crate::brdf::{BRDFInput, DiffuseBRDF, MicrofacetBRDF};

pub mod object;
use crate::object::*;

pub mod primitive;
use crate::primitive::*;

pub mod ray;

extern crate obj;

pub mod mesh;

use std::time::Instant;

use crate::ray::Ray;

#[macro_use]
extern crate lazy_static;

pub mod sample;

pub mod light;

use crate::light::*;

fn radiance(depth: i32, ray: &Ray, scene: &dyn object::Intersect) -> Vector3<f64> {
    if depth >= 3 {
        return Vector3::repeat(0.0);
    }

    if let Some(record) = scene.intersect(&ray) {
        let (m, p, v, n, o) = sample::init_vectors(&ray, &record);

        let (l, pdf) = record.brdf.p();

        let r = sample::sample_ray(&m, &o, &l);

        let f = record.brdf.f(&BRDFInput::new(&n, &l, &v));
        let e = record.brdf.e();

        let light = light::PointLight {
            pos: Point3::<f64>::new(0.0, 1.7, 0.0),
            color:  Vector3::<f64>::new(1.0, 1.0, 1.0),
            power: 3.0,
        };

        e + (f * pdf).component_mul(&radiance(depth + 1, &r, scene))
            * n.dot(&l)
            + light.sample(&m, &n, &v, &p, &record.brdf)
    } else {
        Vector3::repeat(0.0)
    }
}

fn main() {
    let width = 256;
    let height = width;

    let mut im = image::RgbImage::new(width, height);
    let (im_width, im_height) = im.dimensions();

    let mut camera = PerspectiveCamera::new(Vector2::<u32>::new(width, height));
    camera.isometry = na::geometry::Isometry3::look_at_lh(
        &Vector3::new(0.5, 0.6, 2.7).into(),
        &Point3::origin(),
        &Vector3::y_axis(),
    );

    let box_size = 2.0;
    let mut scene = cornell_box(box_size);

    let mut ball = Object::new(Sphere::new(
        Point3::new(-0.55, -box_size + 0.6, -box_size + 0.5),
        0.6,
    ));
    ball.brdf = Box::new(MicrofacetBRDF {
        albedo: Vector3::<f64>::new(0.0, 0.5, 0.5),
        f0: Vector3::<f64>::repeat(0.8),
        roughness: 0.6,
        specular: 0.5
    });

    scene.primitives.push(Box::new(ball));

    let start = Instant::now();

    for i in 0..im_width {
        for j in 0..im_height {
            let ray = camera.get_ray(i, j);

            let mut c = Vector3::<f64>::repeat(0.0);

            let spp = 1000;
            for _ in 0..spp {
                c += radiance(0, &ray, &scene);
            }
            c /= spp as f64;

            let pixel = im.get_pixel_mut(i, j);

            pixel[0] = (c[0].min(1.0) * 255.0) as u8;
            pixel[1] = (c[1].min(1.0) * 255.0) as u8;
            pixel[2] = (c[2].min(1.0) * 255.0) as u8;
        }
    }

    println!("Execution time: {:?}", start.elapsed());

    im.save("test_image.ppm").unwrap();
}

fn cornell_box(box_size: f64) -> AggregateObject {
    let mut scene = AggregateObject::new();

    let ceiling_floor_brdf = Box::new(DiffuseBRDF{
        color: Vector3::<f64>::new(1.0, 1.0, 1.0),
    });

    let mut floor = Object::new(Plane {
        pos: Point3::new(0.0, -box_size, 0.0),
        nrm: Vector3::new(0.0, 1.0, 0.0)
    });
    floor.brdf = ceiling_floor_brdf.clone();

    let mut ceiling = Object::new(Plane {
        pos: Point3::new(0.0, box_size, 0.0),
        nrm: Vector3::new(0.0, -1.0, 0.0)
    });
    ceiling.brdf = ceiling_floor_brdf.clone();

    let mut back_wall = Object::new(Plane {
        pos: Point3::new(0.0, 0.0, -box_size),
        nrm: Vector3::new(0.0, 0.0, 1.0)
    });
    back_wall.brdf = ceiling_floor_brdf.clone();

    let mut left_wall = Object::new(Plane {
        pos: Point3::new(box_size, 0.0, 0.0),
        nrm: Vector3::new(-1.0, 0.0, 0.0)
    });
    left_wall.brdf = Box::new(DiffuseBRDF {
        color: Vector3::<f64>::new(0.0, 1.0, 0.0),
    });

    let mut right_wall = Object::new(Plane {
        pos: Point3::new(-box_size, 0.0, 0.0),
        nrm: Vector3::new(1.0, 0.0, 0.0)
    });
    right_wall.brdf = Box::new(DiffuseBRDF {
        color: Vector3::<f64>::new(1.0, 0.0, 0.0),
    });

    // let mut light = Object::new(Sphere::new(
    //     Point3::new(0.0, box_size, 0.0),
    //     0.5,
    //     ));
    // light.brdf = Box::new(EmissiveBRDF::new(
    //     Vector3::<f64>::new(1.0, 1.0, 1.0),
    //     22.0,
    // ));

    scene.primitives.push(Box::new(floor));
    scene.primitives.push(Box::new(ceiling));
    scene.primitives.push(Box::new(back_wall));
    scene.primitives.push(Box::new(left_wall));
    scene.primitives.push(Box::new(right_wall));
    // scene.primitives.push(Box::new(light));

    scene
}
