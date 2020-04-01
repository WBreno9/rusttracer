extern crate nalgebra as na;
use na::Vector3;

use crate::sample;

use std::f64::consts::PI;

fn ggx_chi(a: f64) -> f64 {
    if a > 0.0 {
        1.0
    } else {
        0.0
    }
}

fn ggx_ndf(alpha: f64, n: &Vector3<f64>, m: &Vector3<f64>) -> f64 {
    let alpha2 = alpha * alpha;
    let dot = n.dot(&m);
    let denom = 1.0 + dot * dot * (alpha2 - 1.0);
    (ggx_chi(dot) * alpha2) / (PI * denom * denom)
}

fn ggx_g1(alpha: f64, n: &Vector3<f64>, s: &Vector3<f64>) -> f64 {
    let dot = n.dot(&s);
    (2.0 * dot * ggx_chi(dot)) / ((2.0 - alpha) + alpha)
}

fn fresnel_schlick_scalar(f0: f64, n: &Vector3<f64>, l: &Vector3<f64>) -> f64 {
    f0 + (1.0 - f0) * (1.0 - n.dot(&l)).powf(5.0)
}

fn fresnel_schlick(f0: &Vector3<f64>, n: &Vector3<f64>, l: &Vector3<f64>) -> Vector3<f64> {
    Vector3::<f64>::new(
        fresnel_schlick_scalar(f0[0], &n, &l),
        fresnel_schlick_scalar(f0[1], &n, &l),
        fresnel_schlick_scalar(f0[2], &n, &l),
    )
}

pub trait BRDF {
    fn f(&self, input: &BRDFInput) -> Vector3<f64>;
    fn p(&self) -> (Vector3<f64>, f64);
    fn e(&self) -> Vector3<f64>;
}

pub struct BRDFInput<'a> {
    pub n: &'a Vector3<f64>,
    pub l: &'a Vector3<f64>,
    pub v: &'a Vector3<f64>,
}

impl<'a> BRDFInput<'a> {
    pub fn new(n: &'a Vector3<f64>, 
        l: &'a Vector3<f64>, 
        v: &'a Vector3<f64>) -> BRDFInput<'a> {
        BRDFInput {
            n, l, v
        }
    }
}

#[derive(Clone)]
pub struct EmissiveBRDF {
    pub color: Vector3<f64>,
    pub power: f64,
}

impl EmissiveBRDF {
    pub fn new(
        color: Vector3<f64>,
        power: f64,
    ) -> EmissiveBRDF {
        EmissiveBRDF {
            color: color.normalize(),
            power,
        }
    }
}

impl BRDF for EmissiveBRDF {
    fn f(&self, _: &BRDFInput) -> Vector3<f64> {
        Vector3::zeros()
    }
    fn p(&self) -> (Vector3<f64>, f64) {
        (Vector3::zeros(), 0.0)
    }
    fn e(&self) -> Vector3<f64> {
        self.color * self.power
    }
}

#[derive(Clone)]
pub struct DiffuseBRDF {
    pub color: Vector3<f64>,
}


impl BRDF for DiffuseBRDF {
    fn f(&self, _: &BRDFInput) -> Vector3<f64> {
        self.color
    }

    fn p(&self) -> (Vector3<f64>, f64) {
        (sample::uniform_hemisphere(), 1.0 / 2.0*PI)
    }

    fn e(&self) -> Vector3<f64> {
        Vector3::zeros()
    }
}

#[derive(Clone)]
pub struct MicrofacetBRDF {
    pub albedo: Vector3<f64>,
    pub f0: Vector3<f64>,
    pub roughness: f64,
    pub specular: f64,
}

impl MicrofacetBRDF {
    pub fn new(
        albedo: Vector3<f64>,
        f0: Vector3<f64>,
        roughness: f64,
        specular: f64,
    ) -> MicrofacetBRDF {
        MicrofacetBRDF {
            albedo,
            f0,
            roughness,
            specular,
        }
    }
}

impl BRDF for MicrofacetBRDF {
    fn f(&self, input: &BRDFInput) -> Vector3<f64> {
        let h = (input.l + input.v).normalize();

        let num = ggx_ndf(self.roughness, &input.n, &h)
            * ggx_g1(self.roughness, &input.n, &input.v)
            * ggx_g1(self.roughness, &input.n, &input.l)
            * fresnel_schlick(&self.f0, &input.n, &input.l);

        let den = 4.0 * (input.n.dot(&input.l) * input.n.dot(&input.v));

        let s = num / den;

        let d = input.n.dot(&input.l).max(0.0).min(1.0);

        (self.albedo * d) + (s * self.specular)
    }

    fn p(&self) -> (Vector3<f64>, f64) {
        (sample::uniform_hemisphere(), 1.0 / 2.0*std::f64::consts::PI)
    }

    fn e(&self) -> Vector3<f64> {
        Vector3::zeros()
    }
}
