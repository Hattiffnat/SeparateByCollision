import ctypes
import logging
import platform
import time
from collections.abc import Iterable
from itertools import chain
from math import inf
from pathlib import Path
from typing import final

import blf
import bmesh
import bpy
import numpy as np
from bmesh.types import BMesh, BMVert
from bpy.props import EnumProperty, FloatProperty
from bpy.types import Context, Event, Menu, Mesh, MeshPolygon, Object
from bpy_extras import view3d_utils
from gpu_extras.presets import draw_circle_2d
from mathutils import Vector

from .py_collisions import Island, MeshEdge, UnionFindManager

logging.basicConfig(
    level=logging.INFO,
    # format="%(levelname)s|separate_by_collision:%(lineno)d %(message)s"
)
log = logging.getLogger(__name__)

log.debug(__file__)

# DETECT NATIVE DLL

system = platform.system()
collisions_backend = None
try:
    if system == "Linux":
        lib_collisions = ctypes.CDLL(Path(__file__).parent / "bin/libcollisions.so")
        # lib_collisions = ctypes.CDLL(Path(__file__).parent / "bin/collisions.dll")
        collisions_backend = "Rust"
    elif system == "Windows":
        lib_collisions = ctypes.CDLL(Path(__file__).parent / "bin/collisions.dll")
        collisions_backend = "Rust"
    elif system == "Darwin":
        lib_collisions = ctypes.CDLL(Path(__file__).parent / "bin/libcollisions.dylib")
        collisions_backend = "Rust"
    else:
        log.warning(f"⚠️ No collision DLL for your system ({system}). 🐢Python🐢 code will be used instead")
except (IOError, OSError) as err:
    log.error(err)
    log.error("❗ Unable to load collisions DLL. 🐢Python🐢 code will be used instead")
    collisions_backend = "Python"
else:
    log.debug(f"system: {system}")
    if collisions_backend is None:
        from . import py_collisions

        collisions_backend = "Python"
    else:
        collisions_backend = "Rust"


# collisions_backend = "Python"

if collisions_backend == "Rust":
    log.info("🚀 collisions DLL loaded 🚀")

    @final
    class CMesh(ctypes.Structure):
        _fields_ = [
            ("verts", ctypes.POINTER(ctypes.c_float)),
            ("verts_len", ctypes.c_uint32),
            ("edges", ctypes.POINTER(ctypes.c_uint32)),
            ("edges_len", ctypes.c_uint32),
            ("tris", ctypes.POINTER(ctypes.c_uint32)),
            ("tris_len", ctypes.c_uint32),
        ]

    @final
    class CGroups(ctypes.Structure):
        _fields_ = [
            ("verts_inds", ctypes.POINTER(ctypes.c_uint32)),
            ("verts_inds_len", ctypes.c_uint32),
            ("offsets", ctypes.POINTER(ctypes.c_uint32)),
            ("offsets_len", ctypes.c_uint32),
        ]

    lib_collisions.calculate_groups.argtypes = [ctypes.POINTER(CMesh), ctypes.c_float, ctypes.c_bool]
    lib_collisions.calculate_groups.restype = CGroups

    lib_collisions.free_cgroups.argtypes = [CGroups]
    lib_collisions.free_cgroups.restype = None

    def cgroups_to_islands(cgroups: CGroups, mesh: Mesh) -> dict[Island, list[Island]]:
        verts_inds_flat = [cgroups.verts_inds[i] for i in range(cgroups.verts_inds_len)]
        offsets = [cgroups.offsets[i] for i in range(cgroups.offsets_len)]
        log.debug(f"len(offsets) = {len(offsets)}")

        verts_union_find = UnionFindManager(verts_inds_flat)
        left = 0
        for offset in offsets:
            right = left + offset

            vertices = verts_inds_flat[left:right]

            r = vertices[0]
            for v in vertices:
                verts_union_find.union(r, v)

            left = right

        islands: dict[int, Island] = {}

        for rv, verts in verts_union_find.groups().items():
            islands[rv] = Island(mesh, 0)
            islands[rv].vertices = verts

        for edge in mesh.edges:
            edge: MeshEdge
            rv = verts_union_find.find(edge.vertices[0])
            islands[rv].edges.append(edge)

        for poly in mesh.polygons:
            poly: MeshPolygon
            rv = verts_union_find.find(poly.vertices[0])
            islands[rv].faces.append(poly)

        return {isl: [isl] for isl in islands.values()}


ELEMENTS = ("verts", "edges", "loops", "faces")
VAL_TYPES = frozenset(
    ("float", "shape", "skin", "deform", "color", "string", "int", "uv", "float_color", "bool", "float_vector")
)


def copy_bmesh_geom(
    source_bm: BMesh,
    dist_bm: BMesh,
    vert_indices: Iterable[int],
    edge_indices: Iterable[int],
    face_indices: Iterable[int],
) -> dict[BMVert, BMVert]:
    """
    Unfortunately, bmesh.ops.separate and bmesh.ops.duplicate currently cannot
    extract elements to another bmesh.
    """
    vert_map: dict[BMVert, BMVert] = {}

    def get_vert(s_v):
        if s_v not in vert_map:
            vert_map[s_v] = dist_bm.verts.new(s_v.co, v)
        return vert_map[s_v]

    # --- create layers ---

    for elem in ELEMENTS:
        for value_type in VAL_TYPES:
            lays = getattr(getattr(source_bm, elem).layers, value_type, None)
            if lays is None:
                continue

            for layer in lays:
                getattr(getattr(dist_bm, elem).layers, value_type).new(layer.name)

    # --- copy geometry ---

    # verts
    for nv_i in vert_indices:
        v: BMVert = source_bm.verts[nv_i]

        nv: BMVert = dist_bm.verts.new(v.co, v)
        vert_map[v] = nv

    # edges
    for e_i in edge_indices:
        e = source_bm.edges[e_i]
        _ = dist_bm.edges.new((get_vert(e.verts[0]), get_vert(e.verts[1])), e)

    # faces
    for f_i in face_indices:
        f = source_bm.faces[f_i]

        # face_verts = frozenset(vert_map[v] for v in f.verts)
        face_verts = list(vert_map[v] for v in f.verts)
        # if len(face_verts) < 3:
        #     continue

        nf = dist_bm.faces.new(face_verts, f)

        # copy loop data
        for l_src, l_dst in zip(f.loops, nf.loops):
            for layer in source_bm.loops.layers.uv:
                dst_layer = dist_bm.loops.layers.uv.get(layer.name)
                l_dst[dst_layer].uv = l_src[layer].uv

            for value_type in ("color", "int", "float", "float_vector", "float_color", "bool"):
                lays = getattr(source_bm.loops.layers, value_type, None)
                if lays is None:
                    continue

                for layer in lays:
                    dst_layer = getattr(dist_bm.loops.layers, value_type).get(layer.name)
                    l_dst[dst_layer] = l_src[layer]

    return vert_map


def copy_shape_keys(mesh_1: Mesh, dst_obj: Object, vert_index_map: dict[int, int]):
    if mesh_1.shape_keys is None:
        return

    mesh_2 = dst_obj.data

    for shape_key in mesh_1.shape_keys.key_blocks:
        new_shape_key = dst_obj.shape_key_add(name=shape_key.name, from_mix=False)
        for nv_i, _ in enumerate(mesh_2.vertices):
            v_i = vert_index_map[nv_i]
            new_shape_key.points[nv_i].co = shape_key.points[v_i].co

        new_shape_key.mute = shape_key.mute
        new_shape_key.lock_shape = shape_key.lock_shape

        new_shape_key.value = shape_key.value
        new_shape_key.slider_min = shape_key.slider_min
        new_shape_key.slider_max = shape_key.slider_max
        new_shape_key.vertex_group = shape_key.vertex_group
        new_shape_key.relative_key = shape_key.relative_key

        new_shape_key.interpolation = shape_key.interpolation

    mesh_2.shape_keys.use_relative = mesh_1.shape_keys.use_relative


def copy_vertex_groups(source_obj: Object, dst_obj: Object, vert_index_map: dict[int, int]):
    for group in source_obj.vertex_groups:
        new_group = dst_obj.vertex_groups.new(name=group.name)
        for nv_i, _ in enumerate(dst_obj.data.vertices):
            v_i = vert_index_map[nv_i]
            nv = dst_obj.data.vertices[nv_i]
            if nv.groups.find(group.name) >= 0:
                weight = group.weight(v_i)
                new_group.add([nv_i], weight, "REPLACE")


@final
class SeparateByCollisionOperator(bpy.types.Operator):
    """Separate mesh by loose parts. Parts will be grouped if they collide"""

    bl_idname = "mesh.separate_by_collision"
    bl_label = "Separate by Collision"

    radius: FloatProperty(
        min=0.0,
        max=inf,
        default=0.0,
        description="Collision detection radius",
    )
    mode: EnumProperty(
        items=(
            ("SURFACE", "Surface", "⚠️ Can be slow on large meshes"),
            ("BB", "Bounding Box", "Fast"),
        ),
        default=0,
        description="Collision type",
    )

    @classmethod
    def poll(cls, context: Context):
        return context.mode == "OBJECT"

    def invoke(self, context: Context, event: Event) -> set[str]:
        self._mouse_co = [0.0, 0.0]

        def circle_draw():
            origin = view3d_utils.location_3d_to_region_2d(
                context.region, context.region_data, context.scene.cursor.location, default=0.0
            )

            radius = (origin - Vector(self._mouse_co)).length

            draw_circle_2d(origin, [1.0, 0.5, 0.5, 1.0], radius, segments=64)

        def text_draw():
            blf.position(0, context.region.width / 2, 50, 0)
            blf.size(0, 14.0)
            blf.draw(0, f"radius {self.radius:.5}")

        self.circle_draw_handler = bpy.types.SpaceView3D.draw_handler_add(circle_draw, (), "WINDOW", "POST_PIXEL")
        self.text_draw_handler = bpy.types.SpaceView3D.draw_handler_add(text_draw, (), "WINDOW", "POST_PIXEL")
        context.area.tag_redraw()
        context.window_manager.modal_handler_add(self)
        return {"RUNNING_MODAL"}

    def _remove_handlers(self):
        bpy.types.SpaceView3D.draw_handler_remove(self.circle_draw_handler, "WINDOW")
        bpy.types.SpaceView3D.draw_handler_remove(self.text_draw_handler, "WINDOW")

    @staticmethod
    def _get_radius(region, rv3d, cursor_co, mouse_co) -> float:
        loc = view3d_utils.region_2d_to_location_3d(region, rv3d, mouse_co, cursor_co)
        return (cursor_co - loc).length

    def modal(self, context: Context, event: Event) -> set[str]:
        match event.type:
            case "MOUSEMOVE":
                self._mouse_co[0], self._mouse_co[1] = event.mouse_region_x, event.mouse_region_y
                self.radius = self._get_radius(
                    context.region,
                    context.region_data,
                    context.scene.cursor.location,
                    self._mouse_co,
                )
            case "LEFTMOUSE":
                self._remove_handlers()
                context.area.tag_redraw()
                return self.execute(context)
            case "RIGHTMOUSE" | "ESC":
                self._remove_handlers()
                context.area.tag_redraw()
                return {"CANCELLED"}
            case _:
                pass

        context.area.tag_redraw()
        return {"RUNNING_MODAL"}

    def execute(self, context: Context) -> set[str]:
        start = time.perf_counter()

        log.info(f"radius: {self.radius:.5}\tmode: {self.mode}")

        processed_meshes = set()
        for obj in context.selected_objects:
            if obj.type != "MESH":
                continue

            if obj.data in processed_meshes:
                continue
            processed_meshes.add(obj.data)

            log.info(f"object {obj.name} mesh {obj.data.name}")

            if len(obj.data.vertices) < 2:
                log.warning(f"mesh {obj.data.name} has {len(obj.data.vertices)} vertices")
                continue

            # USING BACKEND

            if collisions_backend == "Rust":
                verts = np.array([v.co for v in obj.data.vertices], dtype=np.float32).flatten()
                log.debug(f"flat verts len: {len(verts)}")

                edges = np.array([e.vertices for e in obj.data.edges], dtype=np.uint32).flatten()
                tris = np.array([t.vertices for t in obj.data.loop_triangles], dtype=np.uint32).flatten()

                c_mesh = CMesh()
                c_mesh.verts = verts.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
                c_mesh.verts_len = len(verts)
                c_mesh.edges = edges.ctypes.data_as(ctypes.POINTER(ctypes.c_uint32))
                c_mesh.edges_len = len(edges)
                c_mesh.tris = tris.ctypes.data_as(ctypes.POINTER(ctypes.c_uint32))
                c_mesh.tris_len = len(tris)

                c_groups: CGroups = lib_collisions.calculate_groups(
                    ctypes.byref(c_mesh), self.radius, self.mode == "SURFACE"
                )

                groups: dict[Island, list[Island]] = cgroups_to_islands(c_groups, obj.data)

                lib_collisions.free_cgroups(c_groups)

            else:
                groups: dict[Island, list[Island]] = py_collisions.calculate_collisions(
                    obj.data, self.radius, self.mode
                )

            log.info(f"collision groups: {len(groups)}")

            if len(groups) < 2:
                continue

            # SEPARATION
            log.info("# SEPARATION")

            bm: BMesh = bmesh.new()
            bm.from_mesh(obj.data)
            bm.verts.ensure_lookup_table()
            bm.edges.ensure_lookup_table()
            bm.faces.ensure_lookup_table()

            verts_to_delete: set[int] = set()

            total_groups = len(groups) - 1
            wm = context.window_manager

            groups_iter = iter(groups.values())
            next(groups_iter)

            wm.progress_begin(0, total_groups)
            for group_i, group in enumerate(groups_iter):
                group_bm: BMesh = bmesh.new(use_operators=False)

                verts_to_delete.update(chain.from_iterable(isl.vertices for isl in group))

                vert_map = copy_bmesh_geom(
                    bm,
                    group_bm,
                    chain.from_iterable(isl.vertices for isl in group),
                    (e.index for e in chain.from_iterable(isl.edges for isl in group)),
                    (f.index for f in chain.from_iterable(isl.faces for isl in group)),
                )

                # --- create vert index map ---

                group_bm.verts.index_update()
                verts_index_map_rv = {nv.index: v.index for v, nv in vert_map.items()}

                # --- create new mesh ---

                new_mesh: Mesh = bpy.data.meshes.new(obj.data.name)
                group_bm.to_mesh(new_mesh)
                group_bm.free()

                # --- apply materials ---

                if obj.data.materials is not None:
                    [new_mesh.materials.append(mat) for mat in obj.data.materials]

                # --- set UV map ---

                for uv_i, uv_layer in enumerate(obj.data.uv_layers):
                    new_uv_layer = new_mesh.uv_layers[uv_i]
                    new_uv_layer.active = uv_layer.active
                    new_uv_layer.active_render = uv_layer.active_render
                    new_uv_layer.active_clone = uv_layer.active_clone

                # --- set vertex color ---

                for i, vertex_color in enumerate(obj.data.vertex_colors):
                    new_mesh.vertex_colors[i].active = vertex_color.active
                    new_mesh.vertex_colors[i].active_render = vertex_color.active_render

                # --- create new object ---

                new_obj = obj.copy()
                new_obj.data = new_mesh

                copy_shape_keys(obj.data, new_obj, verts_index_map_rv)
                copy_vertex_groups(obj, new_obj, verts_index_map_rv)

                context.collection.objects.link(new_obj)
                log.info(f"created {new_obj.name}")
                wm.progress_update(group_i)

            wm.progress_end()

            bmesh.ops.delete(bm, geom=[bm.verts[v_i] for v_i in verts_to_delete])
            bm.to_mesh(obj.data)
            bm.free()

        msg = f"finished in: {time.perf_counter() - start:.4f} sec"
        log.info(msg)
        return {"FINISHED"}


# UI


@final
class SeparateByCollisionMenu(Menu):
    bl_idname = "OBJECT_MT_separate_by_collision"
    bl_label = "Separate by Collision"

    def draw(self, context: Context):
        layout = self.layout

        layout.operator(SeparateByCollisionOperator.bl_idname, text="Surface Mode").mode = "SURFACE"
        layout.operator(SeparateByCollisionOperator.bl_idname, text="Bounding Box Mode").mode = "BB"


def separate_by_collision_menu(self, _):
    self.layout.menu(SeparateByCollisionMenu.bl_idname)
    self.layout.separator()


classes = (
    SeparateByCollisionOperator,
    SeparateByCollisionMenu,
)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
        log.debug(f"{cls.__name__} registred")

    bpy.types.VIEW3D_MT_object_context_menu.prepend(separate_by_collision_menu)
    bpy.types.VIEW3D_MT_view.append(separate_by_collision_menu)


def unregister():
    bpy.types.VIEW3D_MT_view.remove(separate_by_collision_menu)
    bpy.types.VIEW3D_MT_object_context_menu.remove(separate_by_collision_menu)

    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
        log.debug(f"{cls.__name__} unregistred")
