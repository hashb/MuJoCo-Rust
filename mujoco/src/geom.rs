use crate::Mesh;

use crate::re_exports::GeomType;

pub fn geom_type_from(val: usize) -> GeomType {
    match val {
        0 => GeomType::PLANE,
        1 => GeomType::HFIELD,
        2 => GeomType::SPHERE,
        3 => GeomType::CAPSULE,
        4 => GeomType::ELLIPSOID,
        5 => GeomType::CYLINDER,
        6 => GeomType::BOX,
        7 => GeomType::MESH,
        1001 => GeomType::NONE,
        _ => panic!("Invalid value for GeomType"),
    }
}

#[derive(Debug, Clone)]
pub struct Geom {
    pub id: i32,
    pub name: String,
    pub geom_type: GeomType,
    pub body_id: i32,
    pub pos: [f64; 3],
    pub quat: [f64; 4],
    pub size: [f64; 3],
    pub color: [f32; 4],
    pub mesh: Option<Mesh>,
    pub geom_group: i32,
    pub geom_contype: i32,
}
