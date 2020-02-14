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

fn main() {
    let width = 800;
    let height = 600;

    let mut im = image::RgbImage::new(width, height);
    let (im_width, im_height) = im.dimensions();

    let ratio = width as f32/height as f32;

    let mut camera = PerspectiveCamera::new();
    camera.isometry = na::geometry::Isometry3::look_at_lh(
        &Vector3::repeat(0.7).into(),
        &Point3::origin(),
        &Vector3::y_axis(),
        );

    let aggregate = mesh::load_mesh_aggregate("tests/bunny.obj").unwrap();
    let mut scene = Object::<AggregatePrimitive<Triangle>>::new(aggregate);
    scene.brdf = Box::new(MicrofacetBRDF {
        albedo: Vector3::<f32>::new(1.0, 1.0, 1.0),
        f0: Vector3::<f32>::new(1.0, 1.0, 1.0),
        roughness: 1.0, 
        specular: 0.5,
    });

    for i in 0..im_width {
        for j in 0..im_height {
            let pixel = im.get_pixel_mut(i, j);

            let screen_space = Vector2::<f32>::new(
                ((i as f32/width as f32) - 0.5) * 2.0 * ratio,
                -((j as f32/height as f32) - 0.5) * 2.0,
            );

            let ray = camera.get_ray(screen_space);

            if let Some(record) = scene.intersect(&ray) {
                let pos = ray.origin.coords + record.t*ray.direction;

                let lp = Vector3::<f32>::new(3.0, 5.0, -1.5).normalize()*5.0;
                let lv = lp - pos;
                let dist2 = lv.dot(&lv);

                let attenuation = 25.0 / dist2;

                let input = BRDFInput {
                    l: &lv.normalize(),
                    n: &record.normal,
                    v: &-ray.direction,
                };
                let c = record.brdf.f(&input) * attenuation;

                pixel[0] = (c[0].min(1.0) * 255.0) as u8;
                pixel[1] = (c[1].min(1.0) * 255.0) as u8;
                pixel[2] = (c[2].min(1.0) * 255.0) as u8;
            } else {
                pixel[0] = 0;
                pixel[1] = 0;
                pixel[2] = 0;
            }

        }
    }

    im.save("test_image.ppm").unwrap();
}
