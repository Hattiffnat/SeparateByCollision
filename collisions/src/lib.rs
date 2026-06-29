use std::{collections::HashMap, hash::Hash};

use bvh::{
    aabb::{Aabb, Bounded},
    bounding_hierarchy::{BHShape, BHValue, BoundingHierarchy},
    bvh::{Bvh, BvhNode, Shapes},
};
use nalgebra::{Point3, SimdComplexField, Vector2, Vector3};

#[repr(C)]
#[derive(Debug)]
pub struct CMesh {
    verts: *mut f32,
    verts_len: u32,

    edges: *mut u32,
    edges_len: u32,

    tris: *mut u32,
    tris_len: u32,
}

#[derive(Debug)]
struct Mesh<'a> {
    points: Vec<Point3<f32>>,
    edges: &'a [u32],
    tris: &'a [u32],
}

impl<'a> Mesh<'a> {
    fn get_point(&self, index: usize) -> &Point3<f32> {
        &self.points[index]
    }

    fn get_edge(&self, index: usize) -> Vector2<usize> {
        let i_o = index * 2;
        Vector2::new(self.edges[i_o] as usize, self.edges[i_o + 1] as usize)
    }

    fn get_tri(&self, index: usize) -> Vector3<usize> {
        let i_o = index * 3;
        Vector3::new(
            self.tris[i_o] as usize,
            self.tris[i_o + 1] as usize,
            self.tris[i_o + 3] as usize,
        )
    }

    fn get_islands(
        &'a self,
        verts_union_find: &UnionFind<usize>,
        radius: f32,
    ) -> HashMap<usize, Island<'a>> {
        let mut islands: HashMap<usize, Island> = HashMap::new();

        for (root_v, verts) in verts_union_find.groups().drain() {
            islands.insert(root_v, Island::new(&self, verts, radius));
        }

        for e in self.edges.chunks_exact(2) {
            let edge = Edge::new(self, e[0] as usize, e[1] as usize, radius);

            islands
                .get_mut(&edge.verts_inds[0])
                .map(|island| island.edges.push(edge));
        }

        for t in self.tris.chunks_exact(3) {
            let tri = Tri::new(self, t[0] as usize, t[1] as usize, t[2] as usize, radius);

            islands
                .get_mut(&tri.verts_inds[0])
                .map(|island| island.tris.push(tri));
        }

        islands
    }
}

impl<'a> TryFrom<*const CMesh> for Mesh<'a> {
    type Error = String;

    fn try_from(c_mesh: *const CMesh) -> Result<Self, Self::Error> {
        if c_mesh.is_null() {
            return Err("mesh pointer is null".into());
        }
        let c_mesh = unsafe { &*c_mesh };

        let points: Vec<_> = if (*c_mesh).verts.is_null() {
            Vec::new()
        } else {
            (unsafe { std::slice::from_raw_parts((*c_mesh).verts, (*c_mesh).verts_len as usize) })
                .chunks_exact(3)
                .map(|co| Point3::new(co[0], co[1], co[2]))
                .collect()
        };

        let edges = if (*c_mesh).edges.is_null() {
            &[]
        } else {
            unsafe { std::slice::from_raw_parts((*c_mesh).edges, (*c_mesh).edges_len as usize) }
        };

        let tris = if (*c_mesh).tris.is_null() {
            &[]
        } else {
            unsafe { std::slice::from_raw_parts((*c_mesh).tris, (*c_mesh).tris_len as usize) }
        };

        Ok(Self {
            points,
            edges,
            tris,
        })
    }
}

// EDGE

#[derive(Debug)]
struct Edge<'a> {
    mesh: &'a Mesh<'a>,
    verts_inds: Vector2<usize>,
    radius: f32,
    bvh_node_index: usize,
}

impl<'a> Edge<'a> {
    fn new(mesh: &'a Mesh, v1: usize, v2: usize, radius: f32) -> Self {
        Self {
            mesh,
            verts_inds: Vector2::new(v1, v2),
            radius,
            bvh_node_index: 0,
        }
    }
}

impl<'a> Bounded<f32, 3> for Edge<'a> {
    fn aabb(&self) -> Aabb<f32, 3> {
        let fat = Vector3::new(self.radius, self.radius, self.radius);

        let p1 = self.mesh.get_point(self.verts_inds[0]);
        let p2 = self.mesh.get_point(self.verts_inds[1]);

        Aabb::with_bounds(p1.inf(&p2) - fat, p1.sup(&p2) + fat)
    }
}

impl<'a> BHShape<f32, 3> for Edge<'a> {
    fn set_bh_node_index(&mut self, index: usize) {
        self.bvh_node_index = index
    }

    fn bh_node_index(&self) -> usize {
        self.bvh_node_index
    }
}

// TRI

#[derive(Debug)]
struct Tri<'a> {
    mesh: &'a Mesh<'a>,
    verts_inds: Vector3<usize>,
    radius: f32,
    bvh_node_index: usize,
}

impl<'a> Tri<'a> {
    fn new(mesh: &'a Mesh, v1: usize, v2: usize, v3: usize, radius: f32) -> Self {
        Self {
            mesh,
            verts_inds: Vector3::new(v1, v2, v3),
            radius,
            bvh_node_index: 0,
        }
    }
}

impl<'a> Bounded<f32, 3> for Tri<'a> {
    fn aabb(&self) -> Aabb<f32, 3> {
        let fat = Vector3::new(self.radius, self.radius, self.radius);

        let p1 = self.mesh.get_point(self.verts_inds[0]);
        let p2 = self.mesh.get_point(self.verts_inds[1]);
        let p3 = self.mesh.get_point(self.verts_inds[2]);

        Aabb::with_bounds(p1.inf(&p2).inf(&p3) - fat, p1.sup(&p2).sup(&p3) + fat)
    }
}

impl<'a> BHShape<f32, 3> for Tri<'a> {
    fn set_bh_node_index(&mut self, index: usize) {
        self.bvh_node_index = index
    }

    fn bh_node_index(&self) -> usize {
        self.bvh_node_index
    }
}

#[derive(Debug)]
struct Island<'a> {
    mesh: &'a Mesh<'a>,
    verts_inds: Vec<usize>,
    edges: Vec<Edge<'a>>, // [[vertex_index_1, vertex_index_2], ... ]
    tris: Vec<Tri<'a>>,   // [[v1, v2, v3], ... ]
    radius: f32,
    bvh_node_index: usize,
}

impl<'a> Island<'a> {
    fn new(mesh: &'a Mesh, verts_inds: Vec<usize>, radius: f32) -> Self {
        Self {
            mesh,
            verts_inds,
            edges: Vec::new(),
            tris: Vec::new(),
            radius,
            bvh_node_index: 0,
        }
    }
}

impl<'a> PartialEq for &Island<'a> {
    fn eq(&self, other: &Self) -> bool {
        std::ptr::addr_eq(self, other)
    }
}

impl<'a> Eq for &Island<'a> {}

impl<'a> Hash for &Island<'a> {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        std::ptr::from_ref(*self).hash(state);
    }
}

impl<'a> Bounded<f32, 3> for Island<'a> {
    fn aabb(&self) -> Aabb<f32, 3> {
        let fat = Vector3::new(self.radius, self.radius, self.radius);

        let (min, max) = self
            .verts_inds
            .iter()
            .map(|v_i| self.mesh.get_point(*v_i))
            .fold(
                (
                    Point3::new(f32::INFINITY, f32::INFINITY, f32::INFINITY),
                    Point3::new(f32::NEG_INFINITY, f32::NEG_INFINITY, f32::NEG_INFINITY),
                ),
                |(p_min, p_max), p| (p_min.inf(&p), p_max.sup(&p)),
            );

        Aabb::with_bounds(min - fat, max + fat)
    }
}

impl<'a> BHShape<f32, 3> for Island<'a> {
    fn set_bh_node_index(&mut self, index: usize) {
        self.bvh_node_index = index
    }

    fn bh_node_index(&self) -> usize {
        self.bvh_node_index
    }
}

#[repr(C)]
#[derive(Debug, Default)]
pub struct CGroups {
    pub verts_inds_ptr: *mut u32,
    pub verts_inds_len: u32,

    pub offsets_ptr: *mut u32,
    pub offsets_len: u32,
}

// impl Drop for CGroups {
//     fn drop(&mut self) {
//         unsafe {
//             if !self.verts_inds.is_null() {
//                 let layout = Layout::array::<u32>(self.verts_inds_len as usize).unwrap();
//                 dealloc(self.verts_inds as *mut u8, layout);
//             }
//             if !self.offsets_ptr.is_null() {
//                 let layout = Layout::array::<u32>(self.offsets_len as usize).unwrap();
//                 dealloc(self.offsets_ptr as *mut u8, layout);
//             }
//         }
//     }
// }

pub extern "C" fn drop_cgroups(c_groups: CGroups) {
    drop(c_groups);
}

struct UnionFind<T: Hash + Eq + Copy> {
    parents: HashMap<T, T>,
}

impl<T: Hash + Eq + Copy> UnionFind<T> {
    fn new<'a>(items: &'a [T]) -> Self {
        Self {
            parents: items.iter().map(|v| (*v, *v)).collect(),
        }
    }

    fn union(&mut self, a: T, b: T) {
        let r_a = self.find(a);
        let r_b = self.find(b);
        if r_a != r_b {
            self.parents.insert(b, r_a);
        }
    }

    fn find<'l>(&'l self, a: T) -> T {
        let mut x = a;
        while self.parents.get(&x).map_or(false, |v| v != v) {
            x = *self.parents.get(&x).expect("a should be in parents");
        }
        x
    }

    fn groups(&self) -> HashMap<T, Vec<T>> {
        let mut groups: HashMap<T, Vec<T>> = HashMap::new();

        for v in self.parents.values() {
            let r = self.find(*v);
            groups.entry(r).or_default().push(*v);
        }

        groups
    }
}

fn bvh_vs_bvh<'a, F, const D: usize, T>(
    bvh_1: &'a Bvh<F, D>,
    bvh_2: &'a Bvh<F, D>,
    shapes_1: &'a [T], //
    shapes_2: &'a [T],
    // ) -> Vec<(&'a BvhNode<F, D>, &'a BvhNode<F, D>)>
) -> Vec<(&'a T, &'a T)>
where
    F: BHValue,
    T: BHShape<F, D>,
{
    let mut result = Vec::new();
    let mut stack = vec![(&bvh_1.nodes[0], &bvh_2.nodes[0])];

    while let Some((node_1, node_2)) = stack.pop() {
        if !node_1
            .get_node_aabb(shapes_1)
            .intersects_aabb(&node_2.get_node_aabb(shapes_2))
        {
            continue;
        }

        match (node_1, node_2) {
            (
                BvhNode::Leaf {
                    shape_index: sh1, ..
                },
                BvhNode::Leaf {
                    shape_index: sh2, ..
                },
            ) => result.push((&shapes_1[*sh1], &shapes_2[*sh2])),
            (
                BvhNode::Leaf { .. },
                BvhNode::Node {
                    child_l_index,
                    child_r_index,
                    ..
                },
            ) => {
                stack.push((node_1, &bvh_2.nodes[*child_l_index]));
                stack.push((node_1, &bvh_2.nodes[*child_r_index]));
            }
            (
                BvhNode::Node {
                    child_l_index,
                    child_r_index,
                    ..
                },
                BvhNode::Leaf { .. },
            ) => {
                stack.push((&bvh_1.nodes[*child_l_index], node_2));
                stack.push((&bvh_1.nodes[*child_r_index], node_2));
            }
            (
                BvhNode::Node {
                    child_l_index: ch_l_1,
                    child_r_index: ch_r_1,
                    ..
                },
                BvhNode::Node {
                    child_l_index: ch_l_2,
                    child_r_index: ch_r_2,
                    ..
                },
            ) => {
                stack.push((&bvh_1.nodes[*ch_l_1], &bvh_2.nodes[*ch_l_2]));
                stack.push((&bvh_1.nodes[*ch_l_1], &bvh_2.nodes[*ch_r_2]));
                stack.push((&bvh_1.nodes[*ch_r_1], &bvh_2.nodes[*ch_l_2]));
                stack.push((&bvh_1.nodes[*ch_r_1], &bvh_2.nodes[*ch_r_2]));
            }
        }
    }

    result
}

fn island_vs_island<'a>(
    isl_1: &'a Island,
    isl_2: &'a Island,
    edge_bvhs: &'a mut HashMap<&'a Island<'a>, Bvh<f32, 3>>,
    tri_bvhs: &'a mut HashMap<&'a Island<'a>, Bvh<f32, 3>>,
) -> bool {
    let x = edge_bvhs.entry(isl_1);

    // fn get_edge_bvh(isl: &Island, edge_bvhs: &mut HashMap<&Island, Bvh<f32, 3>>) -> &Bvh<f32, 3> {
    //     todo!();
    // }

    // SINGLE VERT ISLANDS CASE
    if isl_1.verts_inds.len() == 1 && isl_2.verts_inds.len() == 1 {
        let p1 = isl_1.mesh.get_point(isl_1.verts_inds[0]);
        let p2 = isl_2.mesh.get_point(isl_2.verts_inds[0]);

        return (p1 - p2).magnitude_squared() <= (isl_1.radius + isl_2.radius).simd_powi(2);
    }

    let (isl_1, isl_2) = if isl_1.verts_inds.len() > isl_2.verts_inds.len() {
        (isl_2, isl_1)
    } else {
        (isl_1, isl_2)
    };

    todo!()
}

/// Calculate collisions between mesh islands.
/// Panic if c_mesh pointer is null.
#[unsafe(no_mangle)]
pub extern "C" fn calculate_groups(
    c_mesh: *const CMesh,
    inflation_radius: f32,
    surface_collisions: bool,
) -> CGroups {
    let mesh = Mesh::try_from(c_mesh).expect("should converts");

    let points_len_u32 = mesh.points.len() as u32;

    let points_indices: Vec<_> = (0..mesh.points.len()).collect();
    let mut verts_union_find = UnionFind::new(&points_indices);
    mesh.edges.chunks_exact(2).for_each(|e| {
        verts_union_find.union(e[0] as usize, e[1] as usize);
    });

    let islands_inds = verts_union_find.groups();

    if islands_inds.len() < 2 {
        let verts_inds: Box<[u32]> = (0..points_len_u32).collect();
        let verts_inds_ptr = Box::into_raw(verts_inds) as *mut u32;

        return CGroups {
            verts_inds_ptr,
            verts_inds_len: points_len_u32,
            offsets_ptr: [points_len_u32].as_mut_ptr(),
            offsets_len: 1,
        };
    }

    // BROAD PHASE
    println!("BROAD PHASE");

    let mut islands = mesh.get_islands(&verts_union_find, inflation_radius);

    let islands_bvh = Bvh::build_par(islands.values_mut().collect::<Vec<_>>().as_mut_slice());

    let islands_vec = islands.values().collect::<Vec<_>>();
    let mut broad_collisions = Vec::new();

    for island_1 in islands.values() {
        for island_2 in islands_bvh.traverse(&island_1.aabb(), islands_vec.as_slice()) {
            if !std::ptr::addr_eq(island_1, *island_2) {
                broad_collisions.push((island_1, *island_2));
            }
        }
    }

    if !surface_collisions {
        let verts_inds: Box<_> = islands_vec
            .iter()
            .map(|isl| &isl.verts_inds)
            .flatten()
            .map(|v_i| *v_i as u32)
            .collect();

        let offsets: Box<_> = islands_vec
            .iter()
            .map(|isl| isl.verts_inds.len() as u32)
            .collect();
        let offsets_len = offsets.len() as u32;

        return CGroups {
            verts_inds_ptr: Box::into_raw(verts_inds) as *mut u32,
            verts_inds_len: points_len_u32,
            offsets_ptr: Box::into_raw(offsets) as *mut u32,
            offsets_len,
        };
    }

    // NARROW PHASE

    let mut tri_bvhs: HashMap<&Island, Bvh<f32, 3>> = HashMap::new();
    let mut edge_bvhs: HashMap<&Island, Bvh<f32, 3>> = HashMap::new();

    let mut islands_union_find = UnionFind::new(&islands_vec);

    // let narrow_collisions = broad_collisions
    //     .iter()
    //     .filter(|(isl_1, isl_2)| island_vs_island(isl_1, isl_2, &mut edge_bvhs, &mut tri_bvhs));

    todo!()
}

#[cfg(test)]
mod tests {
    // use super::*;

    #[test]
    fn it_works() {}
}
