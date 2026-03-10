import time
from collections import defaultdict
from itertools import chain, combinations, product
from math import inf
from typing import FrozenSet, Iterable, List, Optional, Self, Set, Tuple, Union

import bmesh
import bpy
from bpy.props import EnumProperty, FloatProperty
from bpy.types import MeshEdge, MeshLoopTriangle, MeshPolygon
from mathutils import Vector
from mathutils.geometry import (
    closest_point_on_tri,
    intersect_point_line_segment,
)


class AABB:
    __slots__ = ('min', 'max', 'dimensions')

    def __init__(self, points: Iterable[Vector], radius):
        points = tuple(points)

        self.min = Vector()
        self.max = Vector()
        for axis in range(3):
            self.min[axis] = min(p[axis] for p in points) - radius
            self.max[axis] = max(p[axis] for p in points) + radius

        self.dimensions = self.max - self.min

    def __str__(self):
        return f'AABB(min {self.min}, max {self.max})'

    def longest_axis(self):
        m = max(self.dimensions)

        for axis in range(3):
            if m == self.dimensions[axis]:
                return axis

    def intersects(self, other):
        return all(self.max[axis] >= other.min[axis] for axis in range(3)) and all(
            other.max[axis] >= self.min[axis] for axis in range(3)
        )

    def visualize(self, bm):
        verts = list(product(*zip(self.min, self.max)))

        for i, v1_co in enumerate(verts):
            v1 = bm.verts.new(v1_co)
            for v2_co in verts[i + 1 :]:
                v2 = bm.verts.new(v2_co)
                diff = sum(c1 != c2 for c1, c2 in zip(v1_co, v2_co))
                if diff == 1:
                    bm.edges.new((v1, v2))


class Island:
    __slots__ = ('id_data', 'vertices', 'edges', 'faces', 'tris', '_aabb', 'radius')

    def __init__(self, mesh, radius):
        self.id_data = mesh
        self.radius = radius

        self.vertices: List[int] = []
        self.edges: List[MeshEdge] = []
        self.faces: List[MeshPolygon] = []
        self.tris: List[MeshLoopTriangle] = []

        self._aabb = None

    def __str__(self):
        return f'Island( v {len(self.vertices)} | e {len(self.edges)} | f {len(self.faces)} )'

    #    def __repr__(self):
    #        return self.__str__()

    def aabb(self):
        if self._aabb is None:
            self._aabb = AABB((self.id_data.vertices[v_i].co for v_i in self.vertices), 0.0)

        return self._aabb


class UnionFindManager:
    def __init__(self, values):
        self.parent = {v: v for v in values}
        self.rank = {v: 0 for v in values}

    def find(self, v):
        while self.parent[v] != v:
            self.parent[v] = self.parent[self.parent[v]]
            v = self.parent[v]
        return v

    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        if ra == rb:
            return

        if self.rank[ra] < self.rank[rb]:
            self.parent[ra] = rb
        else:
            self.parent[rb] = ra
            if self.rank[ra] == self.rank[rb]:
                self.rank[ra] += 1

    def groups(self):
        groups = defaultdict(list)

        for k in self.parent:
            r = self.find(k)
            groups[r].append(k)

        return groups


class MeshElement:
    __slots__ = ('element', 'aabb', 'centroid')

    def __init__(self, element: Union[MeshEdge, MeshLoopTriangle, MeshPolygon, Island]):
        self.element = element
        self.aabb = AABB((element.id_data.vertices[v].co for v in self.element.vertices), 0.0)
        self.centroid = (self.aabb.min + self.aabb.max) * 0.5

    def __str__(self):
        return f'MeshElement(el = {self.element})'


class BVHNode:
    """
    I couldn't get mathutils.bvhtree to work properly with edges and radius.
    """

    __slots__ = ('_left', '_right', 'elements', 'aabb', 'radius', 'depth')

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
            return 'BVHNode( ' + '|'.join(map(str, self.elements)) + ' )'
        return 'BVHNode()'

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

    def left(self) -> Optional[Self]:
        self._split()
        return self._left

    def right(self) -> Optional[Self]:
        self._split()
        return self._right

    def visualize(self, bm):
        self.aabb.visualize(bm)

        if self._left is not None:
            self._left.visualize(bm)
        if self._right is not None:
            self._right.visualize(bm)


def line_line_distance_squared(a0: Vector, a1: Vector, b0: Vector, b1: Vector):
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


def leafs(node) -> Iterable[BVHNode]:
    stack = [node]

    while stack:
        current = stack.pop()

        if current.left() is None:
            yield current
            continue

        stack.append(current.right())
        stack.append(current.left())


def bvh_aabb_query(node, aabb, radius) -> Iterable[BVHNode]:
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


def bvh_overlap(bvh_1: BVHNode, bvh_2: BVHNode, radius) -> Iterable[Tuple[BVHNode, BVHNode]]:
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


def island_vs_island(mesh, isl_1, isl_2, radius, island_edges_bvh, island_tris_bvh) -> bool:
    # print(isl_1, "VS", isl_2)

    def get_edges_bvh(isl):
        if isl not in island_edges_bvh:
            island_edges_bvh[isl] = BVHNode([MeshElement(e) for e in isl.edges], radius)

        return island_edges_bvh[isl]

    def get_tris_bvh(isl):
        if isl not in island_tris_bvh:
            island_tris_bvh[isl] = BVHNode([MeshElement(t) for t in isl.tris], radius)

        return island_tris_bvh[isl]

    diameter = radius * 2
    diameter_squared = diameter**2

    # VERT VS VERT
    if len(isl_1.vertices) == 1 and len(isl_2.vertices) == 1:
        v1, v2 = mesh.vertices[isl_1.vertices[0]], mesh.vertices[isl_2.vertices[0]]
        return (v1.co - v2.co).length_squared <= diameter_squared

    if len(isl_1.vertices) > len(isl_2.vertices):
        isl_1, isl_2 = isl_2, isl_1

    # VERT VS MESH
    if len(isl_1.vertices) == 1:
        vert = mesh.vertices[isl_1.vertices[0]]

        # VERT VS TRIS
        if isl_2.tris:
            #            print("DEBUG", "vert vs tris")
            for node_2 in bvh_aabb_query(get_tris_bvh(isl_2), isl_1.aabb(), 0.0):
                for el in node_2.elements:
                    closest = closest_point_on_tri(vert.co, *(mesh.vertices[v_i].co for v_i in el.element.vertices))
                    if (vert.co - closest).length_squared <= diameter_squared:
                        return True

        # VERT VS EDGES
        for node_2 in bvh_aabb_query(get_edges_bvh(isl_2), isl_1.aabb(), 0.0):
            #            print("DEBUG", "vert vs edges")
            for el in node_2.elements:
                e = el.element
                e_v_1, e_v_2 = (
                    mesh.vertices[e.vertices[0]],
                    mesh.vertices[e.vertices[0]],
                )

                _, distance = intersect_point_line_segment(vert.co, e_v_1.co, e_v_2.co)

                if distance <= diameter:
                    return True

        return False

    # EDGE VS EDGE
    for node_1, node_2 in bvh_overlap(get_edges_bvh(isl_1), get_edges_bvh(isl_2), 0.0):
        for el_1, el_2 in product(node_1.elements, node_2.elements):
            v1_i, v2_i = el_1.element.vertices
            v3_i, v4_i = el_2.element.vertices

            v1, v2 = mesh.vertices[v1_i], mesh.vertices[v2_i]
            v3, v4 = mesh.vertices[v3_i], mesh.vertices[v4_i]

            #            print("DEBUG", "edge vs edge")
            #            print(("{} " * 4).format(v1.co, v2.co, v3.co, v4.co))

            ds = line_line_distance_squared(v1.co, v2.co, v3.co, v4.co)
            #            print("distance squared:", ds, "distance:", ds ** 0.5)
            if ds <= diameter_squared:
                return True

    # TRI VS TRI
    if len(isl_1.tris) == 0 or len(isl_2.tris) == 0:
        return False

    for node_1, node_2 in bvh_overlap(get_tris_bvh(isl_1), get_tris_bvh(isl_2), 0.0):
        for el_1, el_2 in product(node_1.elements, node_2.elements):
            t_1, t_2 = el_1.element, el_2.element

            #            print("DEBUG", "tri vs tri")
            #            pp((t_1, t_2))

            for v1_i, t in chain(product(t_1.vertices, [t_2]), product(t_2.vertices, [t_1])):
                v1 = mesh.vertices[v1_i]
                t_co = (mesh.vertices[v2_i].co for v2_i in t.vertices)

                closest = closest_point_on_tri(v1.co, *t_co)
                if (v1.co - closest).length_squared <= diameter_squared:
                    return True

    return False


class SeparateByCollisionOperator(bpy.types.Operator):
    """Separate mesh by loose parts. Parts will be grouped if they collide"""

    bl_idname = 'mesh.separate_by_collision'
    bl_label = 'Separate by Collision'

    radius: FloatProperty(min=0.0, max=inf, default=0.0, description='Collision detection radius')

    mode: EnumProperty(
        items=(
            ('SURFACE', 'Surface', '⚠️ Can be slow on large meshes'),
            ('BB', 'Bounding Box', 'Fast'),
        ),
        default=0,
        description='Collision type',
    )

    @classmethod
    def poll(cls, context):
        return context.mode == 'OBJECT'

    def execute(self, context):
        start = time.perf_counter()

        print(f'radius: {self.radius:.5}\tmode {self.mode}')

        for obj in context.selected_objects:
            print(obj.name)
            if obj.type != 'MESH':
                continue

            verts_union_find = UnionFindManager(range(len(obj.data.vertices)))

            for e in obj.data.edges:
                verts_union_find.union(e.vertices[0], e.vertices[1])

            islands = {}

            for r_i, verts_indices in verts_union_find.groups().items():
                islands[r_i] = Island(obj.data, self.radius)
                islands[r_i].vertices.extend(verts_indices)

            print('islands:', len(islands))

            # BROAD PHASE
            print('# BROAD PHASE')

            islands_bvh = BVHNode([MeshElement(isl) for isl in islands.values()], self.radius)

            broad_collisions: Set[FrozenSet[Island]] = set()

            if islands_bvh.left() is None:
                broad_collisions.update(map(frozenset, combinations(islands.values(), 2)))

            else:
                for node_1 in leafs(islands_bvh):
                    #                    print("DEBUG", "node_1.aabb.dimensions:", node_1.aabb.dimensions)
                    for node_2 in bvh_aabb_query(islands_bvh, node_1.aabb, 0.0):
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

            print('broad phase collisions:', len(broad_collisions))

            #######
            #            bvh_bm = bmesh.new()
            #
            #            islands_bvh.visualize(bvh_bm)
            #
            #            bvh_mesh = bpy.data.meshes.new("bvh_tree")
            #            bvh_bm.to_mesh(bvh_mesh)
            #
            #            bvh_obj = bpy.data.objects.new("bvh_tree", bvh_mesh)
            #            bvh_obj.matrix_local = obj.matrix_local
            #            context.collection.objects.link(bvh_obj)
            #            return
            #######

            if self.mode == 'BB':
                collisions = broad_collisions
            else:
                # NARROW PHASE
                print('# NARROW PHASE')

                for e in obj.data.edges:
                    r = verts_union_find.find(e.vertices[0])
                    islands[r].edges.append(e)
                island_edges_bvh = {}

                for t in obj.data.loop_triangles:
                    r = verts_union_find.find(t.vertices[0])
                    islands[r].tris.append(t)
                island_tris_bvh = {}

                collisions = {
                    coll
                    for coll in broad_collisions
                    if island_vs_island(obj.data, *coll, self.radius, island_edges_bvh, island_tris_bvh)
                }
                print('narrow phase collisions:', len(collisions))

            collision_union_find = UnionFindManager(islands.values())

            for isl_1, isl_2 in collisions:
                collision_union_find.union(isl_1, isl_2)

            groups = collision_union_find.groups()
            print('collision groups:', len(groups))

            if len(groups) < 2:
                continue

            # TODO:
            # Unfortunately, bmesh.ops.separate and bmesh.ops.duplicate currently cannot
            # extract elements to another bmesh. However, this separation method is
            # guaranteed to preserve all attributes, vertex color, vertex groups, etc.

            bm = bmesh.new()
            bm.from_mesh(obj.data)
            bm.verts.ensure_lookup_table()

            verts_to_delete = set()

            groups_iter = iter(groups.values())
            next(groups_iter)
            for group in groups_iter:
                mesh = obj.data.copy()
                group_bm = bmesh.new()
                group_bm.from_mesh(mesh)

                group_verts = frozenset(chain(*(isl.vertices for isl in group)))

                bmesh.ops.delete(group_bm, geom=[v for v in group_bm.verts if v.index not in group_verts])
                verts_to_delete.update(group_verts)

                group_obj = obj.copy()
                group_obj.data = mesh
                group_bm.to_mesh(mesh)
                group_bm.free()

                context.collection.objects.link(group_obj)
                print('object', group_obj.name, 'created')

            bmesh.ops.delete(bm, geom=[v for v in bm.verts if v.index in verts_to_delete])
            bm.to_mesh(obj.data)
            bm.free()

        print(f'finished in: {time.perf_counter() - start:.4} sec')
        return {'FINISHED'}


# UI


def separate_by_collision_menu(self, context):
    self.layout.operator(SeparateByCollisionOperator.bl_idname)
    self.layout.separator()


classes = (SeparateByCollisionOperator,)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
        print(cls.__name__, 'registred')

    bpy.types.VIEW3D_MT_object_context_menu.prepend(separate_by_collision_menu)
    bpy.types.VIEW3D_MT_view.append(separate_by_collision_menu)


def unregister():
    bpy.types.VIEW3D_MT_view.remove(separate_by_collision_menu)
    bpy.types.VIEW3D_MT_object_context_menu.remove(separate_by_collision_menu)

    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
        print(cls.__name__, 'unregistred')


if __name__ == '__main__':
    bpy.utils.register_class(SeparateByCollisionOperator)

    # bpy.ops.mesh.separate_by_collision(radius=0.3, mode="BB")
    bpy.ops.mesh.separate_by_collision(radius=0.3, mode='SURFACE')
