extern crate image;

extern crate nalgebra as na;
use na::Point3;
use na::Vector2;
use na::Vector3;

pub mod camera;
use crate::camera::PerspectiveCamera;

mod brdf;
use crate::brdf::{BRDFInput, BRDF};

pub mod object;
use crate::object::*;

pub mod primitive;

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

pub mod bvh;

struct Scene {
    obj: Box<dyn Intersect>,
    lights: Vec<Box<dyn Light>>,
}

use rand::random;

impl Scene {
    fn get_light(&self) -> &Box<dyn Light> {
        let i = (random::<f64>() * self.lights.len() as f64).floor() as usize;
        &self.lights[i]
    }
}

fn direct_light(s: &sample::SampleRecord, brdf: &Box<dyn BRDF>, scene: &Scene) -> Vector3<f64> {
    let light = scene.get_light();

    let (lp, lpdf) = light.sample_point();

    let lpo = lp - s.o;

    let sr = Ray {
        origin: s.o + s.on * 0.000001,
        direction: lpo.normalize(),
    };

    let mut direct = Vector3::<f64>::zeros();

    let res = scene.obj.intersect(&sr);

    if res.is_none() || res.unwrap().t > lpo.norm() {
        let lp2 = s.m * lp;
        let lp2s = lp2 - s.p;
        let lv = lp2s.normalize();
        let ld = lp2s.norm_squared();

        let dot = s.n.dot(&lv).min(1.0).max(0.0);

        let lf = brdf.f(&BRDFInput {
            n: &s.n,
            l: &lv,
            v: &s.v,
        });

        direct = lf * dot * (1.0 / (lpdf * ld));
        direct = direct.component_mul(&light.shade(&s.m, &s.o, &lv));
    }

    direct * scene.lights.len() as f64
}

fn radiance(depth: i32, mut ray: Ray, scene: &Scene) -> Vector3<f64> {
    let mut color = Vector3::<f64>::zeros();
    let mut b = Vector3::<f64>::repeat(1.0);

    for _ in 0..depth {
        if let Some(record) = scene.obj.intersect(&ray) {
            let s = sample::SampleRecord::new(&ray, &record);

            let (l, pdf) = record.brdf.p();
            let e = record.brdf.e();

            let f = record.brdf.f(&BRDFInput {
                n: &s.n,
                l: &l,
                v: &s.v,
            });

            let lc = direct_light(&s, &record.brdf, &scene);

            ray = Ray {
                origin: s.o + l * 0.000001,
                direction: s.m.inverse_transform_vector(&l),
            };

            color += (e + lc).component_mul(&b);
            b = b.component_mul(&(f * pdf * s.n.dot(&l)));
        }
    }

    color
}

fn main() {
    let width = 128;
    let height = width;

    let mut im = image::RgbImage::new(width, height);
    let (im_width, im_height) = im.dimensions();

    let mut camera = PerspectiveCamera::new(Vector2::<u32>::new(width, height));
    camera.isometry = na::geometry::Isometry3::look_at_rh(
        &Vector3::new(0.000000001, 0.0000001, 2.7269).into(),
        &Point3::origin(),
        &Vector3::y_axis(),
    );
    let scene = Scene {
        obj: Box::new(Object::new(bvh::Tree::new(
            mesh::load_mesh_aggregate("suzanne.obj").unwrap(),
        ))),
        lights: vec![Box::new(DiskLight {
            pos: Point3::<f64>::new(0.0, 1.5, 0.0),
            color: Vector3::<f64>::new(1.0, 1.0, 1.0),
            power: 1.0,
            radius: 0.6,
            normal: Vector3::<f64>::new(0.0, -1.0, 0.0),
        })],
    };

    let spp = 512;

    use indicatif::{ProgressBar, ProgressStyle};

    let pb = ProgressBar::new((im_width * im_height) as u64);
    pb.set_style(
        ProgressStyle::default_bar()
            .template("{spinner:.green} [{elapsed}] [{bar:40.cyan/blue}] {msg:.blue} ({eta:.red})")
            .progress_chars("=> "),
    );

    let w_string_len = width.to_string().len();
    let h_string_len = height.to_string().len();

    let start = Instant::now();

    for i in 0..im_width {
        for j in 0..im_height {
            let mut c = Vector3::<f64>::repeat(0.0);

            for _ in 0..spp {
                let ray = camera.get_ray(i, j);
                c += radiance(4, ray, &scene);
            }
            c /= spp as f64;

            let pixel = im.get_pixel_mut(i, j);

            pixel[0] = (c[0].powf(0.4545).min(1.0) * 255.0) as u8;
            pixel[1] = (c[1].powf(0.4545).min(1.0) * 255.0) as u8;
            pixel[2] = (c[2].powf(0.4545).min(1.0) * 255.0) as u8;

            pb.set_message(&format!(
                "W:[{:w$}, {}] H:[{:h$}, {}]",
                i,
                im_width,
                j,
                im_height,
                w = w_string_len,
                h = h_string_len
            ));
            pb.inc(1);
        }
    }

    println!("Execution time: {:?}", start.elapsed());

    im.save("test_image.ppm").unwrap();
}

// fn cornell_box(box_size: f64) -> AggregateObject {
//     let mut scene = AggregateObject::new();

//     let ceiling_floor_brdf = Box::new(DiffuseBRDF {
//         color: Vector3::<f64>::new(1.0, 1.0, 1.0),
//     });

//     let mut floor = Object::new(Plane {
//         pos: Point3::new(0.0, -box_size, 0.0),
//         nrm: Vector3::new(0.0, 1.0, 0.0),
//     });
//     floor.brdf = ceiling_floor_brdf.clone();

//     let mut ceiling = Object::new(Plane {
//         pos: Point3::new(0.0, box_size, 0.0),
//         nrm: Vector3::new(0.0, -1.0, 0.0),
//     });
//     ceiling.brdf = ceiling_floor_brdf.clone();

//     let mut back_wall = Object::new(Plane {
//         pos: Point3::new(0.0, 0.0, -box_size),
//         nrm: Vector3::new(0.0, 0.0, 1.0),
//     });
//     back_wall.brdf = ceiling_floor_brdf.clone();

//     let mut left_wall = Object::new(Plane {
//         pos: Point3::new(box_size, 0.0, 0.0),
//         nrm: Vector3::new(-1.0, 0.0, 0.0),
//     });
//     left_wall.brdf = Box::new(DiffuseBRDF {
//         color: Vector3::<f64>::new(0.0, 1.0, 0.0),
//     });

//     let mut right_wall = Object::new(Plane {
//         pos: Point3::new(-box_size, 0.0, 0.0),
//         nrm: Vector3::new(1.0, 0.0, 0.0),
//     });
//     right_wall.brdf = Box::new(DiffuseBRDF {
//         color: Vector3::<f64>::new(1.0, 0.0, 0.0),
//     });

//     // let mut light = Object::new(Sphere::new(
//     //     Point3::new(0.0, box_size, 0.0),
//     //     0.5,
//     //     ));
//     // light.brdf = Box::new(EmissiveBRDF::new(
//     //     Vector3::<f64>::new(1.0, 1.0, 1.0),
//     //     22.0,
//     // ));

//     scene.primitives.push(Box::new(floor));
//     scene.primitives.push(Box::new(ceiling));
//     scene.primitives.push(Box::new(back_wall));
//     scene.primitives.push(Box::new(left_wall));
//     scene.primitives.push(Box::new(right_wall));
//     // scene.primitives.push(Box::new(light));

//     scene
// }
