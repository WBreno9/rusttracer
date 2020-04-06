extern crate image;

extern crate nalgebra as na;
use na::Point3;
use na::Vector2;
use na::Vector3;

pub mod camera;
use crate::camera::PerspectiveCamera;

mod brdf;
use crate::brdf::{BRDFInput, DiffuseBRDF};

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
    if depth >= 4 {
        return Vector3::repeat(0.0);
    }

    if let Some(record) = scene.intersect(&ray) {
        let (m, p, v, n, o) = sample::init_vectors(&ray, &record);

        let (l, pdf) = record.brdf.p();

        let r = sample::sample_ray(&m, &o, &l);

        let f = record.brdf.f(&BRDFInput::new(&n, &l, &v));
        let e = record.brdf.e();

        let light = light::DiskLight {
            pos: Point3::<f64>::new(0.0, 1.5, 0.0),
            color: Vector3::<f64>::new(1.0, 1.0, 1.0),
            power: 1.0,
            radius: 0.6,
            normal: Vector3::<f64>::new(0.0, -1.0, 0.0)
        };

        let lp = light.sample_point();

        let sr = Ray {
            origin: o,
            direction: (lp - o).normalize()
        };

        let mut lc = Vector3::<f64>::zeros();

        if let Some(srecord) = scene.intersect(&sr) {
            if srecord.t > (lp - o).norm() {
                let lp2 = m * lp;
                let lv = (lp2 - p).normalize();
                let ld = (lp2 - p).norm_squared();
        
                let c = light.color * light.power;
        
                let dot = n.dot(&lv).min(1.0).max(0.0);
                let dot2 = (m * light.normal).dot(&-lv).max(0.0).min(1.0);
        
                let lf = record.brdf.f(&BRDFInput::new(&n, &lv, &v));

                let lpdf = std::f64::consts::PI * light.radius * light.radius;

                lc = lf * dot * (dot2 / (lpdf * ld));
                lc = lc.component_mul(&c);
            }
        }

        e + lc + (f * pdf).component_mul(&radiance(depth + 1, &r, scene)) * n.dot(&l)
    } else {
        Vector3::repeat(0.0)
    }
}

fn main() {
    let width = 128;
    let height = width;

    let mut im = image::RgbImage::new(width, height);
    let (im_width, im_height) = im.dimensions();

    let mut camera = PerspectiveCamera::new(Vector2::<u32>::new(width, height));
    camera.isometry = na::geometry::Isometry3::look_at_lh(
        &Vector3::new(0.0, 0.0, 2.7269).into(),
        &Point3::origin(),
        &Vector3::y_axis(),
    );

    let box_size = 2.0;
    let mut scene = cornell_box(box_size);

    let mut ball = Object::new(Sphere::new(
        Point3::new(0.6, -box_size + 0.7, -box_size + 0.7),
        0.7,
    ));
    ball.brdf = Box::new(DiffuseBRDF {
        color: Vector3::<f64>::repeat(1.0),
    });

    scene.primitives.push(Box::new(ball));

    let spp = 512;

    use indicatif::{ProgressBar, ProgressStyle};

    let pb = ProgressBar::new((im_width * im_height) as u64);
    pb.set_style(ProgressStyle::default_bar()
        .template("{spinner:.green} [{elapsed}] {bar:40.cyan/blue} {msg} ({eta})")
        .progress_chars("##-"));

    let start = Instant::now();

    for i in 0..im_width {
        for j in 0..im_height {
            let ray = camera.get_ray(i, j);

            let mut c = Vector3::<f64>::repeat(0.0);

            for _ in 0..spp {
                c += radiance(0, &ray, &scene);
            }
            c /= spp as f64;

            let pixel = im.get_pixel_mut(i, j);

            pixel[1] = (c[1].powf(0.4545).min(1.0) * 255.0) as u8;
            pixel[2] = (c[2].powf(0.4545).min(1.0) * 255.0) as u8;
            pixel[0] = (c[0].powf(0.4545).min(1.0) * 255.0) as u8;

            pb.set_message(&format!("W:[{}, {}] H:[{}, {}]",
                i, im_width, j, im_height));
            pb.inc(1);
        }
    }

    println!("Execution time: {:?}", start.elapsed());

    im.save("test_image.ppm").unwrap();
}

fn cornell_box(box_size: f64) -> AggregateObject {
    let mut scene = AggregateObject::new();

    let ceiling_floor_brdf = Box::new(DiffuseBRDF {
        color: Vector3::<f64>::new(1.0, 1.0, 1.0),
    });

    let mut floor = Object::new(Plane {
        pos: Point3::new(0.0, -box_size, 0.0),
        nrm: Vector3::new(0.0, 1.0, 0.0),
    });
    floor.brdf = ceiling_floor_brdf.clone();

    let mut ceiling = Object::new(Plane {
        pos: Point3::new(0.0, box_size, 0.0),
        nrm: Vector3::new(0.0, -1.0, 0.0),
    });
    ceiling.brdf = ceiling_floor_brdf.clone();

    let mut back_wall = Object::new(Plane {
        pos: Point3::new(0.0, 0.0, -box_size),
        nrm: Vector3::new(0.0, 0.0, 1.0),
    });
    back_wall.brdf = ceiling_floor_brdf.clone();

    let mut left_wall = Object::new(Plane {
        pos: Point3::new(box_size, 0.0, 0.0),
        nrm: Vector3::new(-1.0, 0.0, 0.0),
    });
    left_wall.brdf = Box::new(DiffuseBRDF {
        color: Vector3::<f64>::new(0.0, 1.0, 0.0),
    });

    let mut right_wall = Object::new(Plane {
        pos: Point3::new(-box_size, 0.0, 0.0),
        nrm: Vector3::new(1.0, 0.0, 0.0),
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
