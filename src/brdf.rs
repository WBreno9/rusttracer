extern crate nalgebra as na;
use na::Vector3;

use std::f32::consts::PI;

fn ggx_chi(a: f32) -> f32 {
    if a > 0.0 { 1.0 } else { 0.0 }
}

fn ggx_ndf(alpha: f32, n: &Vector3<f32>, m: &Vector3<f32>) -> f32 {
    let alpha2 = alpha*alpha;
    let dot = n.dot(&m);
    let denom = 1.0 + dot*dot*(alpha2 - 1.0);
    (ggx_chi(dot)*alpha2) / (PI*denom*denom)
}

fn ggx_g1(alpha: f32, n: &Vector3<f32>, s: &Vector3<f32>) -> f32 {
    let dot = n.dot(&s);
    (2.0*dot*ggx_chi(dot)) / ((2.0 - alpha) + alpha)
}

fn fresnel_schlick_scalar(f0: f32, n: &Vector3<f32>, l: &Vector3<f32>) -> f32 {
    f0 + (1.0 - f0)*(1.0 - n.dot(&l)).powf(5.0)
}

fn fresnel_schlick(f0: &Vector3<f32>, n: &Vector3<f32>, l: &Vector3<f32>) 
    -> Vector3<f32> {
    Vector3::<f32>::new(
        fresnel_schlick_scalar(f0[0], &n, &l),
        fresnel_schlick_scalar(f0[1], &n, &l),
        fresnel_schlick_scalar(f0[2], &n, &l)
    )
}

pub trait BRDF {
    fn f(&self, input: &BRDFInput) -> Vector3<f32>;
}

pub struct BRDFInput<'a> {
    pub n: &'a Vector3<f32>,
    pub l: &'a Vector3<f32>,
    pub v: &'a Vector3<f32>,
}

#[derive(Clone)]
pub struct MicrofacetBRDF {
    pub albedo: Vector3<f32>,
    pub f0: Vector3<f32>,
    pub roughness: f32,
    pub specular: f32,
}

impl MicrofacetBRDF {
    pub fn new(albedo: Vector3<f32>, 
           f0: Vector3<f32>, 
           roughness: f32,
           specular: f32)
        -> MicrofacetBRDF {
            MicrofacetBRDF {
                albedo,
                f0,
                roughness,
                specular,
            }
    }
}

impl BRDF for MicrofacetBRDF {
    fn f(&self, input: &BRDFInput) -> Vector3<f32> {
        let h = (input.l + input.v).normalize();

        let num = ggx_ndf(self.roughness, &input.n, &h) *
            ggx_g1(self.roughness, &input.n, &input.v) * 
            ggx_g1(self.roughness, &input.n, &input.l) *
            fresnel_schlick(&self.f0, &input.n, &input.l);

        let den = 4.0*(input.n.dot(&input.l)*input.n.dot(&input.v));

        let s = num / den;

        let d = input.n.dot(&input.l).max(0.0).min(1.0);

        ((self.albedo*d) + (s*self.specular))
    }
}

