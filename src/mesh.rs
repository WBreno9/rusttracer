extern crate nalgebra as na;
extern crate obj;
extern crate serde_json;

use serde_json::json;

use na::Vector3;

use std::error::Error;

use crate::object;
use crate::primitive::{AggregatePrimitive, Triangle, Vertex};

use crate::brdf::*;

use crate::bvh;

type Mesh = object::Object<AggregatePrimitive<Triangle>>;
type Model = object::AggregateObject;

type BVHMesh = object::Object<bvh::Tree>;

fn create_material(meta_data: &serde_json::Value, name: &str) -> Box<dyn BRDF> {
    let data = &meta_data["groups"]
        .as_array()
        .unwrap()
        .iter()
        .find(|x| x["name"] == json!(name))
        .unwrap()["material"];

    match data["name"].as_str().unwrap() {
        "diffuse" => {
            let color = Vector3::<f64>::new(
                data["color"]["r"].as_f64().unwrap(),
                data["color"]["g"].as_f64().unwrap(),
                data["color"]["b"].as_f64().unwrap(),
            );

            Box::new(DiffuseBRDF { color })
        }
        _ => Box::new(DiffuseBRDF {
            color: Vector3::repeat(1.0),
        }),
    }
}

fn load_mesh_group(
    obj_mesh: &obj::Obj<obj::SimplePolygon>,
    index: usize,
    meta_data: &serde_json::Value,
) -> Result<Mesh, Box<dyn Error>> {
    let group = &obj_mesh.objects[0].groups[index];

    let mut has_normal = true;
    let mut aggregate = AggregatePrimitive::<Triangle>::new();

    for poly in group.polys.clone().into_iter() {
        let mut vert = Vec::<Vertex>::new();

        let mut v_pos = Vec::<Vector3<f64>>::new();
        let mut v_nrm = Vec::<Vector3<f64>>::new();

        for obj::IndexTuple(pos_index, _, nrm_index) in poly {
            let pos_v = obj_mesh.position[pos_index];
            let pos = Vector3::<f64>::new(pos_v[0] as f64, pos_v[1] as f64, pos_v[2] as f64);
            v_pos.push(pos);

            if has_normal {
                if let Some(nrm_index_uwraped) = nrm_index {
                    let nrm_v = obj_mesh.normal[nrm_index_uwraped];
                    let nrm =
                        Vector3::<f64>::new(nrm_v[0] as f64, nrm_v[1] as f64, nrm_v[2] as f64);
                    v_nrm.push(nrm);
                } else {
                    has_normal = false;
                }
            }
        }

        if !has_normal {
            v_nrm.clear();

            let e1 = v_pos[1] - v_pos[0];
            let e2 = v_pos[2] - v_pos[0];
            let nrm = e1.cross(&e2).normalize();

            for _ in 0..3 {
                v_nrm.push(nrm);
            }
        }

        for (pos, nrm) in v_pos.into_iter().zip(v_nrm.into_iter()) {
            vert.push(Vertex { pos, nrm });
        }

        let triangle = Triangle::new(&vert);
        aggregate.primitives.push(triangle);
    }

    let mut mesh = Mesh::new(aggregate);
    mesh.brdf = create_material(meta_data, &group.name);

    Ok(mesh)
}

pub fn load_mesh(path: &str) -> Result<Mesh, Box<dyn Error>> {
    use std::path::Path;
    let obj_mesh = obj::Obj::<obj::SimplePolygon>::load(&Path::new(path))?;

    if obj_mesh.objects.len() != 1 {
        return Err("Failed to load obj, file needs to \
                    have at least one object"
            .into());
    } else if obj_mesh.objects[0].groups.len() != 1 {
        return Err("Failed to load obj, object needs to \
                    have at least one group"
            .into());
    }

    let meta_path = path.to_owned() + ".json";

    let meta_data = serde_json::from_str(&std::fs::read_to_string(meta_path)?)?;

    load_mesh_group(&obj_mesh, 0, &meta_data)
}

pub fn load_model(path: &str) -> Result<Model, Box<dyn Error>> {
    use std::path::Path;
    let obj_mesh = obj::Obj::<obj::SimplePolygon>::load(&Path::new(path))?;

    if obj_mesh.objects.len() < 1 {
        return Err("Failed to load obj, file needs to \
                    have at least one object"
            .into());
    } else if obj_mesh.objects[0].groups.len() < 1 {
        return Err("Failed to load obj, object needs to \
                    have at least one group"
            .into());
    }

    let meta_path = path.to_owned() + ".json";

    let meta_data = serde_json::from_str(&std::fs::read_to_string(meta_path)?)?;

    let mut model = Model::new();

    for (index, _) in obj_mesh.objects[0].groups.iter().enumerate() {
        model
            .primitives
            .push(Box::new(load_mesh_group(&obj_mesh, index, &meta_data)?));
    }

    Ok(model)
}

pub fn load_mesh_bvh(path: &str) -> Result<BVHMesh, Box<dyn Error>> {
    let mesh = load_mesh(path)?;

    Ok(BVHMesh {
        primitive: bvh::Tree::new(mesh.primitive),
        brdf: mesh.brdf,
    })
}

pub fn load_model_bvh(path: &str) -> Result<Model, Box<dyn Error>> {
    use std::path::Path;
    let obj_mesh = obj::Obj::<obj::SimplePolygon>::load(&Path::new(path))?;

    if obj_mesh.objects.len() < 1 {
        return Err("Failed to load obj, file needs to \
                    have at least one object"
            .into());
    } else if obj_mesh.objects[0].groups.len() < 1 {
        return Err("Failed to load obj, object needs to \
                    have at least one group"
            .into());
    }

    let meta_path = path.to_owned() + ".json";

    let meta_data = serde_json::from_str(&std::fs::read_to_string(meta_path)?)?;

    let mut model = Model::new();

    for (index, _) in obj_mesh.objects[0].groups.iter().enumerate() {
        let mesh = load_mesh_group(&obj_mesh, index, &meta_data)?;

        model.primitives.push(Box::new(BVHMesh {
            primitive: bvh::Tree::new(mesh.primitive),
            brdf: mesh.brdf,
        }));
    }

    Ok(model)
}

pub fn load_mesh_aggregate(path: &str) -> Result<AggregatePrimitive<Triangle>, Box<dyn Error>> {
    use std::path::Path;
    let obj_mesh = obj::Obj::<obj::SimplePolygon>::load(&Path::new(path))?;

    if obj_mesh.objects.len() != 1 {
        return Err("Failed to load obj, file needs to \
                    have at least one object"
            .into());
    } else if obj_mesh.objects[0].groups.len() != 1 {
        return Err("Failed to load obj, object needs to \
                    have at least one group"
            .into());
    }

    let mut has_normal = true;

    let mut aggregate = AggregatePrimitive::<Triangle>::new();
    for poly in obj_mesh.objects[0].groups[0].polys.clone().into_iter() {
        let mut vert = Vec::<Vertex>::new();

        let mut v_pos = Vec::<Vector3<f64>>::new();
        let mut v_nrm = Vec::<Vector3<f64>>::new();

        for obj::IndexTuple(pos_index, _, nrm_index) in poly {
            let pos_v = obj_mesh.position[pos_index];
            let pos = Vector3::<f64>::new(pos_v[0] as f64, pos_v[1] as f64, pos_v[2] as f64);
            v_pos.push(pos);

            if has_normal {
                if let Some(nrm_index_uwraped) = nrm_index {
                    let nrm_v = obj_mesh.normal[nrm_index_uwraped];
                    let nrm =
                        Vector3::<f64>::new(nrm_v[0] as f64, nrm_v[1] as f64, nrm_v[2] as f64);
                    v_nrm.push(nrm);
                } else {
                    has_normal = false;
                }
            }
        }

        if !has_normal {
            v_nrm.clear();

            let e1 = v_pos[1] - v_pos[0];
            let e2 = v_pos[2] - v_pos[0];
            let nrm = e1.cross(&e2).normalize();

            for _ in 0..3 {
                v_nrm.push(nrm);
            }
        }

        for (pos, nrm) in v_pos.into_iter().zip(v_nrm.into_iter()) {
            vert.push(Vertex { pos, nrm });
        }

        let triangle = Triangle::new(&vert);
        aggregate.primitives.push(triangle);
    }

    Ok(aggregate)
}
