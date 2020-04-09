extern crate nalgebra as na;
use na::{Isometry3, Point3};
use na::{Vector2, Vector3};
use rand::random;

use crate::ray::Ray;

pub struct Camera {
    pub isometry: Isometry3<f64>,
    pub img_dimensions: Vector2<u32>,
    pub fov: f64,
}

impl Camera {
    pub fn new(
        origin: &Point3<f64>,
        direction: &Vector3<f64>,
        img_dimensions: Vector2<u32>,
        fov: f64,
    ) -> Camera {
        Camera {
            isometry: Isometry3::look_at_rh(origin, &(origin + direction), &Vector3::y_axis()),
            img_dimensions: img_dimensions,
            fov,
        }
    }

    pub fn get_ray(&self, i: u32, j: u32) -> Ray {
        let p = self.to_screen_space(i, j);

        let v = Vector3::<f64>::new(
            p.x + random::<f64>() / (self.img_dimensions.x as f64),
            p.y + random::<f64>() / (self.img_dimensions.y as f64),
            -1.0 / (self.fov / 2.0).to_radians().tan(),
        )
        .normalize();

        // let v = Vector3::<f64>::new(
        //     p.x,
        //     p.y,
        //     -1.0 / (self.fov / 2.0).to_radians().tan(),
        // )
        // .normalize();

        Ray {
            origin: self.isometry.inverse_transform_point(&Point3::origin()),
            direction: self.isometry.inverse_transform_vector(&v),
        }
    }

    pub fn img_ratio(&self) -> f64 {
        self.img_dimensions.x as f64 / self.img_dimensions.y as f64
    }

    pub fn to_screen_space(&self, i: u32, j: u32) -> Vector2<f64> {
        let ratio = self.img_ratio();

        Vector2::<f64>::new(
            ((i as f64 / self.img_dimensions.x as f64) - 0.5) * 2.0 * ratio,
            -((j as f64 / self.img_dimensions.y as f64) - 0.5) * 2.0,
        )
    }
}
