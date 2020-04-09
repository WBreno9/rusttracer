# Trabalho Final

## Microfacet BRDF

$$
  f_{spec} = \frac{F(h,l)G_{2}(l,v,h)D(h)}{4|n \cdot l||n \cdot v|}
$$

```rust
fn f(&self, input: &BRDFInput) -> Vector3<f64> {
        let h = (input.l + input.v).normalize();

        let num = ggx_ndf(self.roughness, &input.n, &h)
            * ggx_g1(self.roughness, &input.n, &input.v)
            * ggx_g1(self.roughness, &input.n, &input.l)
            * fresnel_schlick(&self.f0, &input.n, &input.l);

        let den = 4.0 * (input.n.dot(&input.l) * input.n.dot(&input.v));

        let s = num / den;

        (self.albedo / PI) + (s * self.specular)
    }
```

```rust
fn ggx_ndf(alpha: f64, n: &Vector3<f64>, m: &Vector3<f64>) -> f64 {
    let alpha2 = alpha * alpha;
    let dot = n.dot(&m);
    let denom = 1.0 + dot * dot * (alpha2 - 1.0);
    (ggx_chi(dot) * alpha2) / (PI * denom * denom)
}
```

```rust
fn ggx_g1(alpha: f64, n: &Vector3<f64>, s: &Vector3<f64>) -> f64 {
    let dot = n.dot(&s);
    (2.0 * dot * ggx_chi(dot)) / ((2.0 - alpha) + alpha)
}
```



```rust
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
```

## Amostragem Explicita da Luz

$$
  L(x, \omega_{o}) = \int_{x' \in R} g(x, x')f(x, \omega_{i}, \omega_{o})L_{e}(x', \omega_{i})\cos\theta \frac{\cos\theta'}{p(x')\|x' - x\|^2}
$$

```rust
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

        direct = (lf * dot).component_mul(&(light.shade(&s.m, &s.o, &lv) 
          / (lpdf * ld)));
    }

    direct * scene.lights.len() as f64
}
```

## Estrutura de aceleração: BVH

## Gamma Correction
