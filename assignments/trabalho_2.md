# Atividade II - Teste de Intersecção Raio-Triângulo

```rust
#[derive(Clone)]
pub struct IntersectionRecord {
    pub t: f64,
    pub normal: Vector3<f64>,
}

pub trait Primitive {
    fn intersect(&self, ray: &Ray) -> Option<IntersectionRecord>;
}
```


```rust
#[derive(Debug, Clone, Copy)]
pub struct Vertex {
    pub pos: Vector3<f64>,
    pub nrm: Vector3<f64>,
}
```

```rust
#[derive(Clone)]
pub struct Triangle {
    pub vert: Vec<Vertex>,
}
```


```rust
fn intersect(&self, ray: &Ray) -> Option<IntersectionRecord> {
        let oa = ray.origin.coords - self.vert[0].pos;
        let e1 = self.vert[1].pos - self.vert[0].pos;
        let e2 = self.vert[2].pos - self.vert[0].pos;

        let p = ray.direction.cross(&e2);
        let q = oa.cross(&e1);

        let inv = 1.0 / p.dot(&e1);

        let t = inv * q.dot(&e2);
        let u = inv * p.dot(&oa);
        let v = inv * q.dot(&ray.direction);

        if u < 0.0 || u > 1.0 || v < 0.0 || u + v > 1.0 || t < 0.0 {
            None
        } else {
            let w = 1.0 - u - v;

            let normal =
                (w * self.vert[0].nrm 
                + u * self.vert[1].nrm 
                + v * self.vert[2].nrm)
                .normalize();

            Some(IntersectionRecord { t, normal })
        }
    }
```