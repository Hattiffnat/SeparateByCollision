import logging
from collections import defaultdict
from collections.abc import Iterable
from itertools import chain, combinations, permutations, product
from typing import Generic, Literal, Self, TypeVar, final

from bmesh.types import BMesh
from bpy.types import Mesh, MeshEdge, MeshLoopTriangle, MeshPolygon
from mathutils import Vector
from mathutils.geometry import (
    closest_point_on_tri,
    intersect_point_line,
    intersect_ray_tri,
)

log = logging.getLogger(__name__)


class AABB:
    __slots__ = ("min", "max", "dimensions")

    def __init__(self, points: Iterable[Vector], radius):
        points = tuple(points)

        self.min = Vector()
        self.max = Vector()
        for axis in range(3):
            self.min[axis] = min(p[axis] for p in points) - radius
            self.max[axis] = max(p[axis] for p in points) + radius

        self.dimensions = self.max - self.min

    def __str__(self):
        return f"AABB(min {self.min}, max {self.max})"

    def longest_axis(self):
        m = max(self.dimensions)

        for axis in range(3):
            if m == self.dimensions[axis]:
                return axis

    def intersects(self, other: Self) -> bool:
        return (
            True
            and all(self.max[axis] >= other.min[axis] for axis in range(3))
            and all(other.max[axis] >= self.min[axis] for axis in range(3))
        )

    def visualize(self, bm: BMesh):
        verts = list(product(*zip(self.min, self.max)))

        for i, v1_co in enumerate(verts):
            v1 = bm.verts.new(v1_co)
            for v2_co in verts[i + 1 :]:
                v2 = bm.verts.new(v2_co)
                diff = sum(c1 != c2 for c1, c2 in zip(v1_co, v2_co))
                if diff == 1:
                    bm.edges.new((v1, v2))


class Island:
    __slots__ = ("id_data", "vertices", "edges", "faces", "tris", "_aabb", "radius")

    def __init__(self, mesh, radius):
        self.id_data = mesh
        self.radius = radius

        self.vertices: list[int] = []
        self.edges: list[MeshEdge] = []
        self.faces: list[MeshPolygon] = []
        self.tris: list[MeshLoopTriangle] = []

        self._aabb = None

    def __str__(self):
        return f"Island( v {len(self.vertices)} | e {len(self.edges)} | f {len(self.faces)} )"

    def aabb(self):
        if self._aabb is None:
            self._aabb = AABB((self.id_data.vertices[v_i].co for v_i in self.vertices), 0.0)

        return self._aabb


T = TypeVar("T")


@final
class UnionFindManager(Generic[T]):
    def __init__(self, values: Iterable[T]):
        self.parent = {v: v for v in values}
        self.rank = {v: 0 for v in values}

    def find(self, v: T) -> T:
        while self.parent[v] != v:
            self.parent[v] = self.parent[self.parent[v]]
            v = self.parent[v]
        return v

    def union(self, a: T, b: T):
        ra, rb = self.find(a), self.find(b)
        if ra == rb:
            return

        if self.rank[ra] < self.rank[rb]:
            self.parent[ra] = rb
        else:
            self.parent[rb] = ra
            if self.rank[ra] == self.rank[rb]:
                self.rank[ra] += 1

    def groups(self) -> dict[T, list[T]]:
        groups = defaultdict(list)

        for k in self.parent:
            r = self.find(k)
            groups[r].append(k)

        return groups


@final
class MeshElement:
    __slots__ = ("element", "aabb", "centroid")

    def __init__(self, element: MeshEdge | MeshLoopTriangle | MeshPolygon | Island):
        self.element = element
        self.aabb = AABB((element.id_data.vertices[v].co for v in self.element.vertices), 0.0)
        self.centroid = (self.aabb.min + self.aabb.max) * 0.5

    def __str__(self):
        return f"MeshElement(el = {self.element})"


@final
class BVHNode:
    """
    Lazy BVH Tree.
    I couldn't get mathutils.bvhtree to work properly with edges and radius.
    """

    __slots__ = ("_left", "_right", "elements", "aabb", "radius", "depth")

    MAX_LEAF_SIZE = 1
    MAX_DEPTH = 24

    def __init__(self, elements: Iterable[MeshElement], radius, depth=0):
        self.radius = radius
        self._left = None
        self._right = None
        self.elements = elements

        self.depth = depth

        self.aabb = AABB(chain(*((el.aabb.min, el.aabb.max) for el in elements)), radius)

    def __str__(self):
        if self.elements is not None:
            return "BVHNode( " + "|".join(map(str, self.elements)) + " )"
        return "BVHNode()"

    def _split(self):
        if not self._is_splitable():
            return

        axis = self.aabb.longest_axis()
        self.elements.sort(key=lambda e: e.centroid[axis])

        mid = len(self.elements) // 2
        self._left = BVHNode(self.elements[:mid], self.radius, self.depth + 1)
        self._right = BVHNode(self.elements[mid:], self.radius, self.depth + 1)

        self.elements = None

    def _is_splitable(self) -> bool:
        return self.elements is not None and len(self.elements) > self.MAX_LEAF_SIZE and self.depth < self.MAX_DEPTH

    def left(self) -> Self | None:
        self._split()
        return self._left

    def right(self) -> Self | None:
        self._split()
        return self._right

    def visualize(self, bm):
        self.aabb.visualize(bm)

        if self._left is not None:
            self._left.visualize(bm)
        if self._right is not None:
            self._right.visualize(bm)


def line_line_distance_squared(a0: Vector, a1: Vector, b0: Vector, b1: Vector) -> float:
    """Unfortunately, the mathutils.geometry.intersect_line_line function does not work as expected."""
    EPS = 0.000000001

    d1 = a1 - a0
    d2 = b1 - b0
    r = a0 - b0

    a = d1.dot(d1)
    e = d2.dot(d2)
    f = d2.dot(r)

    if a <= EPS and e <= EPS:
        return (a0 - b0).length

    if a <= EPS:
        s = 0.0
        t = max(0.0, min(1.0, f / e))
    else:
        c = d1.dot(r)

        if e <= EPS:
            t = 0.0
            s = max(0.0, min(1.0, -c / a))
        else:
            b = d1.dot(d2)
            denom = a * e - b * b

            if denom != 0.0:
                s = max(0.0, min(1.0, (b * f - c * e) / denom))
            else:
                s = 0.0

            t = (b * s + f) / e

            if t < 0.0:
                t = 0.0
                s = max(0.0, min(1.0, -c / a))
            elif t > 1.0:
                t = 1.0
                s = max(0.0, min(1.0, (b - c) / a))

    c1 = a0 + d1 * s
    c2 = b0 + d2 * t

    return (c1 - c2).length_squared


def leafs(node: BVHNode) -> Iterable[BVHNode]:
    stack = [node]

    while stack:
        current = stack.pop()

        if current.left() is None:
            yield current
            continue

        stack.append(current.right())
        stack.append(current.left())


def bvh_aabb_query(node: BVHNode, aabb: AABB) -> Iterable[BVHNode]:
    log.debug(f"aabb: {aabb}")
    stack = [node]

    while stack:
        current = stack.pop()

        if not current.aabb.intersects(aabb):
            continue

        left, right = current.left(), current.right()
        if left is None:
            yield current
            continue

        stack.append(left)
        stack.append(right)


def bvh_overlap(bvh_1: BVHNode, bvh_2: BVHNode) -> Iterable[tuple[BVHNode, BVHNode]]:
    stack = [(bvh_1, bvh_2)]
    while stack:
        node_1, node_2 = stack.pop()

        if not node_1.aabb.intersects(node_2.aabb):
            continue

        left_1, right_1 = node_1.left(), node_1.right()
        left_2, right_2 = node_2.left(), node_2.right()

        is_leaf_1 = left_1 is None
        is_leaf_2 = left_2 is None

        if is_leaf_1 and is_leaf_2:
            yield node_1, node_2
            continue

        if is_leaf_1:
            stack.append((node_1, left_2))
            stack.append((node_1, right_2))
        elif is_leaf_2:
            stack.append((node_2, left_1))
            stack.append((node_2, right_1))
        else:
            stack.append((left_1, left_2))
            stack.append((left_1, right_2))
            stack.append((right_1, left_2))
            stack.append((right_1, right_2))


def island_vs_island(
    mesh: Mesh,
    isl_1: Island,
    isl_2: Island,
    radius: float,
    island_edges_bvh: dict[Island, BVHNode],
    island_tris_bvh: dict[Island, BVHNode],
) -> bool:
    def get_edges_bvh(isl: Island) -> BVHNode:
        if isl not in island_edges_bvh:
            island_edges_bvh[isl] = BVHNode([MeshElement(e) for e in isl.edges], radius)
        return island_edges_bvh[isl]

    def get_tris_bvh(isl: Island) -> BVHNode:
        if isl not in island_tris_bvh:
            island_tris_bvh[isl] = BVHNode([MeshElement(t) for t in isl.tris], radius)
        return island_tris_bvh[isl]

    diameter = radius * 2
    diameter_squared = diameter**2
    rad_vec = Vector.Fill(3, radius)

    # VERT VS VERT
    if len(isl_1.vertices) == 1 and len(isl_2.vertices) == 1:
        v1, v2 = mesh.vertices[isl_1.vertices[0]], mesh.vertices[isl_2.vertices[0]]
        return (v1.co - v2.co).length_squared <= diameter_squared

    if len(isl_1.vertices) > len(isl_2.vertices):
        isl_1, isl_2 = isl_2, isl_1

    # VERT VS MESH
    if len(isl_1.vertices) == 1:
        vert = mesh.vertices[isl_1.vertices[0]]
        vert_aabb = AABB([vert.co - rad_vec, vert.co, vert.co + rad_vec], 0)

        # VERT VS TRIS
        if isl_2.tris:
            for node_2 in bvh_aabb_query(get_tris_bvh(isl_2), vert_aabb):
                log.debug("vert vs tris")
                for el in node_2.elements:
                    closest = closest_point_on_tri(vert.co, *(mesh.vertices[v_i].co for v_i in el.element.vertices))
                    log.debug(f"closest: {closest}")
                    if (vert.co - closest).length_squared <= diameter_squared:
                        return True

        # VERT VS EDGES
        for node_2 in bvh_aabb_query(get_edges_bvh(isl_2), vert_aabb):
            log.debug("vert vs edges")
            for el in node_2.elements:
                e = el.element
                e_v_1, e_v_2 = (
                    mesh.vertices[e.vertices[0]],
                    mesh.vertices[e.vertices[0]],
                )

                closest, factor = intersect_point_line(vert.co, e_v_1.co, e_v_2.co)

                if factor < 0.0:
                    dist_sq = (e_v_1.co - vert.co).length_squared
                elif factor > 1.0:
                    dist_sq = (e_v_2.co - vert.co).length_squared
                else:
                    dist_sq = (closest - vert.co).length_squared

                if dist_sq <= diameter_squared:
                    return True

        return False

    # EDGE VS EDGE
    for node_1, node_2 in bvh_overlap(get_edges_bvh(isl_1), get_edges_bvh(isl_2)):
        for el_1, el_2 in product(node_1.elements, node_2.elements):
            log.debug("edge vs edge")
            v1_i, v2_i = el_1.element.vertices
            v3_i, v4_i = el_2.element.vertices

            v1, v2 = mesh.vertices[v1_i], mesh.vertices[v2_i]
            v3, v4 = mesh.vertices[v3_i], mesh.vertices[v4_i]

            ds = line_line_distance_squared(v1.co, v2.co, v3.co, v4.co)
            if ds <= diameter_squared:
                return True

    # TRI VS TRI
    if len(isl_1.tris) == 0 or len(isl_2.tris) == 0:
        return False

    for node_1, node_2 in bvh_overlap(get_tris_bvh(isl_1), get_tris_bvh(isl_2)):
        for el_1, el_2 in product(node_1.elements, node_2.elements):
            for t_1, t_2 in permutations((el_1.element, el_2.element)):
                log.debug("tri vs tri")
                # Intersection test
                t_co = [mesh.vertices[v2_i].co for v2_i in t_2.vertices]
                for e in combinations(t_1.vertices, 2):
                    e_co: list[Vector] = [mesh.vertices[v1_i].co for v1_i in e]
                    normal = e_co[0] - e_co[1]
                    point: Vector = intersect_ray_tri(*t_co, normal.normalized(), e_co[1])
                    if point is not None:
                        normal_length_sq = normal.length_squared
                        if (e_co[1] - point).length_squared <= normal_length_sq:
                            return True

                # Closest point test
                for v1_i in t_1.vertices:
                    v1 = mesh.vertices[v1_i]
                    t_co = (mesh.vertices[v2_i].co for v2_i in t_2.vertices)

                    dist_sq = (v1.co - closest_point_on_tri(v1.co, *t_co)).length_squared
                    if dist_sq <= diameter_squared:
                        return True

    return False


def calculate_collisions(mesh: Mesh, radius: float, mode: Literal["BB", "SURFACE"]) -> dict[Island, list[Island]]:
    verts_union_find = UnionFindManager(range(len(mesh.vertices)))

    for e in mesh.edges:
        verts_union_find.union(e.vertices[0], e.vertices[1])

    islands: dict[int, Island] = {}

    for r_i, verts_indices in verts_union_find.groups().items():
        islands[r_i] = Island(mesh, radius)
        islands[r_i].vertices.extend(verts_indices)

    for e in mesh.edges:
        r = verts_union_find.find(e.vertices[0])
        islands[r].edges.append(e)

    for f in mesh.polygons:
        r = verts_union_find.find(f.vertices[0])
        islands[r].faces.append(f)

    log.info(f"islands: {len(islands)}")

    # BROAD PHASE
    log.info("# BROAD PHASE")

    islands_bvh = BVHNode([MeshElement(isl) for isl in islands.values()], radius)

    broad_collisions: set[frozenset[Island]] = set()

    if islands_bvh.left() is None:
        broad_collisions.update(map(frozenset, combinations(islands.values(), 2)))

    else:
        for node_1 in leafs(islands_bvh):
            for node_2 in bvh_aabb_query(islands_bvh, node_1.aabb):
                if node_1 == node_2:
                    continue

                broad_collisions.update(
                    map(
                        frozenset,
                        combinations(
                            (el.element for el in chain(node_1.elements, node_2.elements)),
                            2,
                        ),
                    )
                )

    log.info(f"broad phase collisions: {len(broad_collisions)}")

    if mode == "BB":
        collisions = broad_collisions
        collisions_union_find = UnionFindManager(islands.values())

        for isl_1, isl_2 in collisions:
            collisions_union_find.union(isl_1, isl_2)
    else:
        # NARROW PHASE
        log.info("# NARROW PHASE")

        for t in mesh.loop_triangles:
            r = verts_union_find.find(t.vertices[0])
            islands[r].tris.append(t)

        island_edges_bvh = {}
        island_tris_bvh = {}

        collisions: set[frozenset[Island]] = set()
        collisions_union_find = UnionFindManager(islands.values())

        for i, coll in enumerate(broad_collisions):
            isl_1, isl_2 = coll
            if collisions_union_find.find(isl_1) == collisions_union_find.find(isl_2):
                collisions.add(coll)
                continue

            if island_vs_island(mesh, isl_1, isl_2, radius, island_edges_bvh, island_tris_bvh):
                collisions_union_find.union(isl_1, isl_2)
                collisions.add(coll)

            progress_ratio = i / len(broad_collisions)
            progress_ratio_int = int(progress_ratio * 100)
            if progress_ratio % 3 == 0.0:
                log.info(f"{'#' * progress_ratio_int}| {progress_ratio * 100:.2f}%")

        log.info(f"narrow phase collisions: {len(collisions)}")

    return collisions_union_find.groups()
