use std::{
    collections::{HashMap, HashSet},
    time::Instant,
};

use bvh::{
    aabb::{Aabb, Bounded},
    bounding_hierarchy::{BHShape, BoundingHierarchy},
    bvh::Bvh,
};
use nalgebra::{Point3, Vector2, Vector3};

mod bvhs_comp;
mod islands_comp;
mod shapes_comp;
mod union_find;

type P3 = Point3<f32>;

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
    points: Vec<P3>,
    edges_flat: &'a [u32],
    tris_flat: &'a [u32],
}

impl<'a> Mesh<'a> {
    #[inline(always)]
    fn get_point(&self, index: &usize) -> &P3 {
        &self.points[*index]
    }

    fn get_islands(&'a self, radius: f32) -> HashMap<usize, Island<'a>> {
        let mut verts_union_find = union_find::UnionFindManager::new(self.points.len());

        // println!("edges_flat = {:?}", self.edges_flat);
        self.edges_flat.chunks_exact(2).for_each(|e| {
            verts_union_find.union(e[0] as usize, e[1] as usize);
        });

        let verts_groups = verts_union_find.groups();
        // println!("verts_groups = {:?}", verts_groups);

        let mut islands: HashMap<usize, Island> = verts_groups
            .into_iter()
            .map(|(root_v, verts)| {
                (
                    root_v,
                    Island::new(&self, verts, Vec::new(), Vec::new(), radius),
                )
            })
            .collect();

        let mut visited_edges: HashSet<Vector2<usize>> = HashSet::new();
        for t in self.tris_flat.chunks_exact(3) {
            let tri = Tri::new(self, t[0] as usize, t[1] as usize, t[2] as usize, radius);

            visited_edges.insert(Vector2::new(t[0] as usize, t[1] as usize));
            visited_edges.insert(Vector2::new(t[1] as usize, t[2] as usize));
            visited_edges.insert(Vector2::new(t[0] as usize, t[2] as usize));

            let rv = verts_union_find.find(tri.verts_inds[0]);

            islands.get_mut(&rv).map(|island| island.tris.push(tri));
        }

        for e in self.edges_flat.chunks_exact(2) {
            let edge = Edge::new(self, e[0] as usize, e[1] as usize, radius);

            if !visited_edges.contains(&edge.verts_inds) {
                let rv = verts_union_find.find(edge.verts_inds[0]);
                islands.get_mut(&rv).map(|island| island.edges.push(edge));
            } else {
                // println!("edge {} not included", edge.verts_inds.transpose());
            }
        }

        // dbg!(visited_edges.len());
        // for isl in islands.values() {
        //     dbg!(isl.edges.len());
        //     dbg!(isl.tris.len());
        // }

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
        // dbg!(c_mesh);

        let points_co_flat = if (*c_mesh).verts.is_null() {
            &[]
        } else {
            unsafe { std::slice::from_raw_parts((*c_mesh).verts, (*c_mesh).verts_len as usize) }
        };
        dbg!(points_co_flat.len());
        // println!("points_co_flat = {:?}", &points_co_flat);

        let edges_flat = if (*c_mesh).edges.is_null() {
            &[]
        } else {
            unsafe { std::slice::from_raw_parts((*c_mesh).edges, (*c_mesh).edges_len as usize) }
        };
        dbg!(edges_flat.len());
        // println!("edges_flat = {:?}", &edges_flat);

        let tris_flat = if (*c_mesh).tris.is_null() {
            &[]
        } else {
            unsafe { std::slice::from_raw_parts((*c_mesh).tris, (*c_mesh).tris_len as usize) }
        };
        dbg!(tris_flat.len());
        // println!("tris_flat = {:?}", &tris_flat);

        Ok(Self {
            points: points_co_flat
                .chunks_exact(3)
                .map(|co| Point3::new(co[0], co[1], co[2]))
                .collect(),
            edges_flat,
            tris_flat,
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

    #[inline(always)]
    fn get_points_co(&self) -> (&P3, &P3) {
        (
            &self.mesh.points[self.verts_inds[0]],
            &self.mesh.points[self.verts_inds[1]],
        )
    }
}

impl<'a> Bounded<f32, 3> for Edge<'a> {
    fn aabb(&self) -> Aabb<f32, 3> {
        let fat = Vector3::new(self.radius, self.radius, self.radius);

        let p1 = self.mesh.get_point(&self.verts_inds[0]);
        let p2 = self.mesh.get_point(&self.verts_inds[1]);

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

    #[inline(always)]
    fn get_points_co(&self) -> (&P3, &P3, &P3) {
        (
            &self.mesh.points[self.verts_inds[0]],
            &self.mesh.points[self.verts_inds[1]],
            &self.mesh.points[self.verts_inds[2]],
        )
    }

    #[inline(always)]
    fn get_segments_co(&self) -> [(&P3, &P3); 3] {
        let (a, b, c) = self.get_points_co();
        [(a, b), (b, c), (a, c)]
    }
}

impl<'a> Bounded<f32, 3> for Tri<'a> {
    fn aabb(&self) -> Aabb<f32, 3> {
        let fat = Vector3::new(self.radius, self.radius, self.radius);

        let p1 = self.mesh.get_point(&self.verts_inds[0]);
        let p2 = self.mesh.get_point(&self.verts_inds[1]);
        let p3 = self.mesh.get_point(&self.verts_inds[2]);

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
    fn new(
        mesh: &'a Mesh,
        verts_inds: Vec<usize>,
        edges: Vec<Edge<'a>>,
        tris: Vec<Tri<'a>>,
        radius: f32,
    ) -> Self {
        Self {
            mesh,
            verts_inds,
            edges,
            tris,
            radius,
            bvh_node_index: 0,
        }
    }
}

impl<'a> Bounded<f32, 3> for Island<'a> {
    fn aabb(&self) -> Aabb<f32, 3> {
        let fat = Vector3::new(self.radius, self.radius, self.radius);

        let (min, max) = self
            .verts_inds
            .iter()
            .map(|v_i| self.mesh.get_point(v_i))
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
    verts_inds_ptr: *mut u32,
    verts_inds_len: u32,

    offsets_ptr: *mut u32,
    offsets_len: u32,
}

impl CGroups {
    fn new(verts_flat: Vec<u32>, offsets: Vec<u32>) -> Self {
        // println!("verts_inds_flat = {:?}", verts_flat);
        // println!("offsets = {:?}", offsets);

        let verts_flat_len = verts_flat.len() as u32;
        let offsets_len = offsets.len() as u32;

        let verts_inds_flat: Box<_> = verts_flat.into_boxed_slice();
        let offsets: Box<_> = offsets.into_boxed_slice();
        Self {
            verts_inds_ptr: Box::into_raw(verts_inds_flat) as *mut u32,
            verts_inds_len: verts_flat_len,
            offsets_ptr: Box::into_raw(offsets) as *mut u32,
            offsets_len,
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn free_cgroups(c_groups: CGroups) {
    let _ = unsafe {
        Vec::from_raw_parts(
            c_groups.verts_inds_ptr,
            c_groups.verts_inds_len as usize,
            c_groups.verts_inds_len as usize,
        )
    };

    let _ = unsafe {
        Vec::from_raw_parts(
            c_groups.offsets_ptr,
            c_groups.offsets_len as usize,
            c_groups.offsets_len as usize,
        )
    };
}

fn flatten_islands_groups(
    groups: HashMap<usize, Vec<usize>>,
    islands: Vec<Island>,
) -> (Vec<u32>, Vec<u32>) {
    let mut verts_inds_flat = Vec::new();
    let mut offsets = Vec::new();

    for (_, islands_subset) in groups.into_iter() {
        let offset: u32 = islands_subset
            .into_iter()
            .map(|isl_i| {
                verts_inds_flat.extend(islands[isl_i].verts_inds.iter().map(|i| *i as u32));
                (islands[isl_i]).verts_inds.len() as u32
            })
            .sum();

        offsets.push(offset);
    }

    (verts_inds_flat, offsets)
}

/// Calculate collisions between mesh islands.
/// Panic if c_mesh pointer is null.
#[unsafe(no_mangle)]
pub extern "C" fn calculate_groups(
    c_mesh: *const CMesh,
    inflation_radius: f32,
    surface_collisions: bool,
) -> CGroups {
    println!("inflation_radius = {}", inflation_radius);
    println!("surface_collisions = {}", surface_collisions);

    let mesh = Mesh::try_from(c_mesh).expect("should converts");
    let islands = mesh.get_islands(inflation_radius);

    let points_len_u32 = mesh.points.len() as u32;
    if islands.len() < 2 {
        let verts_inds: Vec<u32> = (0..points_len_u32).collect();
        let offsets = vec![verts_inds.len() as u32];
        return CGroups::new(verts_inds, offsets);
    }

    // BROAD PHASE
    println!("BROAD PHASE");
    let broad_timer = Instant::now();

    let mut islands: Vec<_> = islands.into_iter().map(|(_, isl)| isl).collect();
    let islands_bvh = Bvh::build_par(&mut islands);

    type BvhIdx = usize;
    let mut broad_collisions: HashSet<(BvhIdx, BvhIdx)> = HashSet::new();

    for island_1 in islands.iter() {
        for island_2 in islands_bvh.traverse(&island_1.aabb(), &islands) {
            if island_1.bvh_node_index != island_2.bvh_node_index {
                let (a, b) = if island_1.bvh_node_index < island_2.bvh_node_index {
                    (island_1.bvh_node_index, island_2.bvh_node_index)
                } else {
                    (island_2.bvh_node_index, island_1.bvh_node_index)
                };

                broad_collisions.insert((a, b));
            }
        }
    }

    let bvh_idx_to_vec_idx: HashMap<BvhIdx, usize> = islands
        .iter()
        .enumerate()
        .map(|(vec_idx, isl)| (isl.bvh_node_index, vec_idx))
        .collect();

    println!("broad phase collisions: {}", broad_collisions.len());

    if !surface_collisions {
        let mut islands_union_find = union_find::UnionFindManager::new(islands.len());

        for (isl_bvh_idx_1, isl_bvh_idx_2) in broad_collisions.iter() {
            let isl_1_i = bvh_idx_to_vec_idx[isl_bvh_idx_1];
            let isl_2_i = bvh_idx_to_vec_idx[isl_bvh_idx_2];

            islands_union_find.union(isl_1_i, isl_2_i);
        }

        let (verts_inds_flat, offsets) =
            flatten_islands_groups(islands_union_find.groups(), islands);
        return CGroups::new(verts_inds_flat, offsets);
    }

    println!("broad phase finished in {:?}", broad_timer.elapsed());

    // NARROW PHASE
    println!("NARROW PHASE");
    let narrow_timer = Instant::now();

    // Hash maps for lazy Bvh creation
    let mut tri_bvhs: HashMap<usize, Bvh<f32, 3>> = HashMap::new();
    let mut edge_bvhs: HashMap<usize, Bvh<f32, 3>> = HashMap::new();

    let mut islands_union_find = union_find::UnionFindManager::new(islands.len());

    broad_collisions
        .iter()
        .for_each(|(isl_bvh_idx_1, isl_bvh_idx_2)| {
            let isl_i_1 = bvh_idx_to_vec_idx[isl_bvh_idx_1];
            let isl_i_2 = bvh_idx_to_vec_idx[isl_bvh_idx_2];

            if islands_union_find.find(isl_i_1) != islands_union_find.find(isl_i_2) {
                if islands_comp::island_vs_island(
                    isl_i_1,
                    isl_i_2,
                    &mut islands,
                    &mut edge_bvhs,
                    &mut tri_bvhs,
                ) {
                    islands_union_find.union(isl_i_1, isl_i_2);
                };
            };
        });

    println!("narrow phase finished in {:?}", narrow_timer.elapsed());

    let (verts_inds_flat, offsets) = flatten_islands_groups(islands_union_find.groups(), islands);
    CGroups::new(verts_inds_flat, offsets)
}

#[cfg(test)]
mod tests {

    use super::*;

    struct SimpleEdge {
        co: (P3, P3),
        index: usize,
    }

    impl Bounded<f32, 3> for SimpleEdge {
        fn aabb(&self) -> Aabb<f32, 3> {
            Aabb {
                min: self.co.0.inf(&self.co.1),
                max: self.co.0.sup(&self.co.1),
            }
        }
    }

    impl BHShape<f32, 3> for SimpleEdge {
        fn set_bh_node_index(&mut self, index: usize) {
            self.index = index;
        }

        fn bh_node_index(&self) -> usize {
            self.index
        }
    }

    #[test]
    fn zero_bvh() {
        let mut shapes: Vec<SimpleEdge> = vec![
            SimpleEdge {
                co: Default::default(),
                index: 0,
            },
            SimpleEdge {
                co: Default::default(),
                index: 0,
            },
        ];

        let bvh: Bvh<f32, 3> = Bvh::build_par(shapes.as_mut_slice());

        dbg!(bvh);
    }

    #[test]
    fn empty_bvh() {
        let mut shapes: Vec<SimpleEdge> = vec![];

        let bvh: Bvh<f32, 3> = Bvh::build_par(shapes.as_mut_slice());

        dbg!(bvh);
    }
}
