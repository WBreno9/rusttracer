extern crate image;

extern crate nalgebra as na;
use na::Point3;
use na::Vector2;
use na::Vector3;

pub mod camera;
use crate::camera::Camera;

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
        &self.lights[0]
    }
}

fn direct_light(s: &sample::SampleRecord, brdf: &Box<dyn BRDF>, scene: &Scene) -> Vector3<f64> {
    let light = scene.get_light();

    let (lp, lpdf) = light.sample_point();

    let lpo = lp - s.o;

    let sr = Ray {
        origin: s.o + s.on * 0.0000000001,
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

        direct = (lf * dot).component_mul(&(light.shade(&s.m, &s.o, &lv) / (lpdf * ld)));
    }

    direct * scene.lights.len() as f64
}

fn radiance(depth: i32, mut ray: Ray, scene: &Scene) -> Vector3<f64> {
    let mut color = Vector3::<f64>::zeros();
    let mut b = Vector3::<f64>::repeat(1.0);

    for _ in 0..depth {
        if let Some(record) = scene.obj.intersect(&ray) {
            let s = sample::SampleRecord::new(&ray, &record);

            let (l, pdf) = record.brdf.p(&s.v);
            let e = record.brdf.e();

            let f = record.brdf.f(&BRDFInput {
                n: &s.n,
                l: &l,
                v: &s.v,
            });

            let lc = direct_light(&s, &record.brdf, &scene);

            ray = Ray {
                origin: s.o + l * 0.0000000001,
                direction: s.m.inverse_transform_vector(&l),
            };

            color += (e + lc).component_mul(&b);
            b = b.component_mul(&((f / pdf) * s.n.dot(&l)));
        }
    }

    color
}

fn main() {
    let width = 512;
    let height = width;

    let mut im = image::RgbImage::new(width, height);
    let (im_width, im_height) = im.dimensions();

    let camera = Camera::new(
        &Vector3::new(0.000000001, 1.2891, 5.873).into(),
        &-(Vector3::new(0.000000001, 1.2891, 5.873) - Vector3::new(0.0, 1.2891, 0.0)).normalize(),
        Vector2::<u32>::new(width, height),
        45.0,
    );

    // let camera = Camera::new(
    //     &Vector3::new(0.000000001, 0.000000001, 7.0001).into(),
    //     &-Vector3::new(0.000000001, 0.000000001, 7.0001).normalize(),
    //     Vector2::<u32>::new(width, height),
    //     33.3,
    // );

    let scene = Scene {
        obj: Box::new(mesh::load_model_bvh("tests/roots.obj").unwrap()),
        lights: vec![
            Box::new(DiskLight {
                pos: Point3::<f64>::new(0.0, 3.0, 0.0),
                color: Vector3::<f64>::new(1.0, 1.0, 1.0),
                power: 1.0,
                radius: 0.6,
                normal: (Point3::origin() - Point3::<f64>::new(0.0, 4.0, 0.0)).normalize(),
            }),
            // Box::new(DiskLight {
            //     pos: Point3::<f64>::new(0.0, 1.0, 4.0),
            //     color: Vector3::<f64>::new(1.0, 1.0, 1.0),
            //     power: 1.6,
            //     radius: 0.5,
            //     normal: (Point3::origin() - Point3::<f64>::new(0.0, 1.0, 4.0)).normalize(),
            // }),
        ],
    };

    let spp = 32;

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
                c += radiance(3, ray, &scene);
            }
            c /= spp as f64;

            // let ray = camera.get_ray(i, j);
            // for mesh in model.iter() {
            //     c += mesh.primitive.debug(&ray)
            // }

            let pixel = im.get_pixel_mut(i, j);

            pixel[0] = (c[0].powf(1.0/2.2).min(1.0) * 255.0) as u8;
            pixel[1] = (c[1].powf(1.0/2.2).min(1.0) * 255.0) as u8;
            pixel[2] = (c[2].powf(1.0/2.2).min(1.0) * 255.0) as u8;

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

    println!(" ");
    println!("Execution time: {:?}", start.elapsed());

    im.save("output.png").unwrap();
}
