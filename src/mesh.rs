extern crate nalgebra as na;
use na::Vector3;

extern crate obj;

use std::error::Error;

use crate::primitive::{AggregatePrimitive, Triangle, Vertex};

pub fn load_mesh_aggregate(path: &str) -> 
Result<AggregatePrimitive<Triangle>, Box<dyn Error>> {
    use std::path::Path;
    let obj_mesh = obj::Obj::<obj::SimplePolygon>::load(&Path::new(path))?;

    if obj_mesh.objects.len() != 1 {
        return Err("Failed to load obj, file needs to \
            have at least one object".into());
    } else if obj_mesh.objects[0].groups.len() != 1 {
        return Err("Failed to load obj, object needs to \
            have at least one group".into());
    }

    let mut has_normal = true;

    let mut aggregate = AggregatePrimitive::<Triangle>::new(); 
    for poly in obj_mesh.objects[0].groups[0].polys.clone().into_iter() {
        let mut vert = Vec::<Vertex>::new();

        let mut v_pos = Vec::<Vector3<f32>>::new();
        let mut v_nrm = Vec::<Vector3<f32>>::new();

        for obj::IndexTuple(pos_index, _, nrm_index) in poly {
            let pos_v = obj_mesh.position[pos_index];
            let pos = Vector3::<f32>::new(pos_v[0], pos_v[1], pos_v[2]);
            v_pos.push(pos);
            
            if has_normal {
                if let Some(nrm_index_uwraped) = nrm_index {
                    let nrm_v = obj_mesh.normal[nrm_index_uwraped];
                    let nrm = Vector3::<f32>::new(nrm_v[0], nrm_v[1], nrm_v[2]);
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
            vert.push(Vertex {
                pos,
                nrm,
            });

        }

        let triangle = Triangle::new(&vert);
        aggregate.primitives.push(triangle);
    }

    Ok(aggregate)
}
