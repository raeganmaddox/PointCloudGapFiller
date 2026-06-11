bl_info = {
    "name": "Point Cloud Gap Filler",
    "author": "OpenAI",
    "version": (1, 0, 0),
    "blender": (4, 4, 0),
    "location": "3D Viewport > Sidebar > Point Cloud",
    "description": "Densify point clouds with adaptive nearest-neighbor interpolation",
    "category": "Object",
}

from array import array
from bisect import bisect_left
import math
import random

import bpy
from bpy.props import (
    BoolProperty,
    EnumProperty,
    FloatProperty,
    IntProperty,
    PointerProperty,
)
from bpy.types import Operator, Panel, PropertyGroup
from mathutils.kdtree import KDTree


def _read_points(obj):
    if obj.type == "MESH":
        return [vertex.co.copy() for vertex in obj.data.vertices]
    if obj.type == "POINTCLOUD":
        return [point.co.copy() for point in obj.data.points]
    raise TypeError("Select a Mesh or Point Cloud object.")


def _build_neighborhoods(points, neighbor_count):
    point_count = len(points)
    tree = KDTree(point_count)
    for index, point in enumerate(points):
        tree.insert(point, index)
    tree.balance()

    search_count = min(point_count, neighbor_count + 1)
    neighborhoods = []
    gap_sizes = []

    for index, point in enumerate(points):
        neighbors = []
        for _co, neighbor_index, distance in tree.find_n(point, search_count):
            if neighbor_index != index:
                neighbors.append((neighbor_index, distance))
        neighborhoods.append(neighbors)
        gap_sizes.append(
            sum(distance for _index, distance in neighbors) / len(neighbors)
            if neighbors
            else 0.0
        )

    return neighborhoods, gap_sizes


def _cumulative_weights(values):
    cumulative = []
    total = 0.0
    for value in values:
        total += value
        cumulative.append(total)
    return cumulative, total


def _weighted_index(cumulative, total, rng):
    if total <= 0.0:
        return rng.randrange(len(cumulative))
    return min(bisect_left(cumulative, rng.random() * total), len(cumulative) - 1)


def _generate_round(points, amount, neighbor_count, gap_bias, spread, rng):
    neighborhoods, gap_sizes = _build_neighborhoods(points, neighbor_count)

    if max(gap_sizes, default=0.0) <= 1.0e-12:
        raise ValueError(
            "The point cloud has no measurable gaps. It may contain only duplicate points."
        )

    if gap_bias == 0.0:
        anchor_weights = [1.0] * len(points)
    else:
        anchor_weights = [math.pow(max(gap, 1.0e-12), gap_bias) for gap in gap_sizes]
    anchor_cumulative, anchor_total = _cumulative_weights(anchor_weights)

    generated = []
    for _index in range(amount):
        anchor_index = _weighted_index(anchor_cumulative, anchor_total, rng)
        neighbors = neighborhoods[anchor_index]
        if not neighbors:
            continue

        # Favor the wider edges around an anchor so sparse regions fill first.
        edge_weights = [max(distance, 1.0e-12) for _neighbor, distance in neighbors]
        edge_cumulative, edge_total = _cumulative_weights(edge_weights)
        partner_slot = _weighted_index(edge_cumulative, edge_total, rng)
        partner_index = neighbors[partner_slot][0]

        # This remains a weighted average of two nearby points. At spread 0 every
        # sample is the midpoint; at spread 1 samples can occupy the whole edge.
        factor = 0.5 + (rng.random() - 0.5) * spread
        generated.append(points[anchor_index].lerp(points[partner_index], factor))

    return generated


def _densify(points, amount, neighbor_count, gap_bias, spread, seed, progress=None):
    result = list(points)
    rng = random.Random(seed)
    remaining = amount
    completed = 0

    # Doubling rounds make generated points participate in later refinement
    # without rebuilding the KD-tree once for every new sample.
    while remaining:
        round_amount = min(remaining, max(1, len(result)))
        generated = _generate_round(
            result,
            round_amount,
            neighbor_count,
            gap_bias,
            spread,
            rng,
        )
        if not generated:
            raise ValueError("No points could be generated from this object.")
        result.extend(generated)
        remaining -= len(generated)
        completed += len(generated)
        if progress:
            progress(completed)

    return result


def _flat_coordinates(points):
    coordinates = array("f")
    for point in points:
        coordinates.extend((point.x, point.y, point.z))
    return coordinates


def _create_mesh_object(name, points, collection):
    mesh = bpy.data.meshes.new(name)
    mesh.vertices.add(len(points))
    mesh.vertices.foreach_set("co", _flat_coordinates(points))
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    collection.objects.link(obj)
    return obj


def _create_point_cloud_object(name, points, collection):
    point_cloud = bpy.data.pointclouds.new(name)
    if not hasattr(point_cloud, "resize"):
        bpy.data.pointclouds.remove(point_cloud)
        return _create_mesh_object(name, points, collection)
    point_cloud.resize(len(points))
    point_cloud.points.foreach_set("co", _flat_coordinates(points))
    point_cloud.update_tag()
    obj = bpy.data.objects.new(name, point_cloud)
    collection.objects.link(obj)
    return obj


class PCGF_Settings(PropertyGroup):
    points_to_add: IntProperty(
        name="Points to Add",
        description="Exact number of interpolated points to generate",
        default=10_000,
        min=1,
        soft_max=1_000_000,
        max=100_000_000,
    )
    neighbor_count: IntProperty(
        name="Neighbors",
        description="Nearby points considered when measuring and filling gaps",
        default=6,
        min=1,
        max=64,
    )
    gap_bias: FloatProperty(
        name="Gap Bias",
        description="Higher values concentrate more points in wider gaps; zero is uniform",
        default=1.5,
        min=0.0,
        max=8.0,
    )
    spread: FloatProperty(
        name="Average Spread",
        description="Zero creates exact midpoints; one spreads weighted averages along each gap",
        default=0.85,
        min=0.0,
        max=1.0,
        subtype="FACTOR",
    )
    seed: IntProperty(
        name="Seed",
        description="Random seed for reproducible interpolation",
        default=0,
        min=0,
    )
    include_original: BoolProperty(
        name="Include Original Points",
        description="Include source points in the new output object",
        default=True,
    )
    output_type: EnumProperty(
        name="Output Type",
        description="Data type used for the new object",
        items=(
            ("AUTO", "Match Input", "Use the same object data type as the source"),
            ("POINTCLOUD", "Point Cloud", "Create a Blender Point Cloud object"),
            ("MESH", "Mesh Vertices", "Create a mesh containing vertices only"),
        ),
        default="AUTO",
    )


class OBJECT_OT_fill_point_cloud_gaps(Operator):
    bl_idname = "object.fill_point_cloud_gaps"
    bl_label = "Fill Point Cloud Gaps"
    bl_description = "Add points using adaptive nearest-neighbor weighted averages"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        obj = context.active_object
        return obj is not None and obj.type in {"MESH", "POINTCLOUD"}

    def execute(self, context):
        source = context.active_object
        settings = context.scene.pcgf_settings

        try:
            source_points = _read_points(source)
        except (TypeError, AttributeError) as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}

        if len(source_points) < 2:
            self.report({"ERROR"}, "At least two source points are required.")
            return {"CANCELLED"}

        output_type = settings.output_type
        if output_type == "AUTO":
            output_type = source.type

        window_manager = context.window_manager
        window_manager.progress_begin(0, settings.points_to_add)
        try:
            densified = _densify(
                source_points,
                settings.points_to_add,
                settings.neighbor_count,
                settings.gap_bias,
                settings.spread,
                settings.seed,
                progress=window_manager.progress_update,
            )
            output_points = (
                densified if settings.include_original else densified[len(source_points):]
            )

            output_name = f"{source.name}_Densified"
            collection = context.collection or source.users_collection[0]
            if output_type == "POINTCLOUD":
                result = _create_point_cloud_object(output_name, output_points, collection)
            else:
                result = _create_mesh_object(output_name, output_points, collection)
        except (MemoryError, OverflowError):
            self.report(
                {"ERROR"},
                "Blender ran out of memory. Try generating fewer points per operation.",
            )
            return {"CANCELLED"}
        except (RuntimeError, ValueError) as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        finally:
            window_manager.progress_end()

        result.matrix_world = source.matrix_world.copy()
        result["pcgf_source"] = source.name
        result["pcgf_generated_points"] = settings.points_to_add
        result["pcgf_seed"] = settings.seed

        if output_type == "POINTCLOUD" and result.type != "POINTCLOUD":
            self.report(
                {"WARNING"},
                "This Blender version cannot create Point Cloud data from Python; "
                "created a vertex-only Mesh instead.",
            )

        source.select_set(False)
        result.select_set(True)
        context.view_layer.objects.active = result

        self.report(
            {"INFO"},
            f"Created {len(output_points):,} points ({settings.points_to_add:,} new).",
        )
        return {"FINISHED"}


class VIEW3D_PT_point_cloud_gap_filler(Panel):
    bl_label = "Point Cloud Gap Filler"
    bl_idname = "VIEW3D_PT_point_cloud_gap_filler"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Point Cloud"

    def draw(self, context):
        layout = self.layout
        settings = context.scene.pcgf_settings
        obj = context.active_object

        if obj is None or obj.type not in {"MESH", "POINTCLOUD"}:
            layout.label(text="Select a Mesh or Point Cloud.", icon="INFO")
            return

        source_count = len(obj.data.vertices) if obj.type == "MESH" else len(obj.data.points)
        layout.label(text=f"Source: {source_count:,} points")

        column = layout.column(align=True)
        column.prop(settings, "points_to_add")
        column.prop(settings, "neighbor_count")
        column.prop(settings, "gap_bias")
        column.prop(settings, "spread")
        column.prop(settings, "seed")

        layout.separator()
        layout.prop(settings, "include_original")
        layout.prop(settings, "output_type")

        output_count = settings.points_to_add
        if settings.include_original:
            output_count += source_count
        layout.label(text=f"Output: {output_count:,} points")
        layout.operator(
            OBJECT_OT_fill_point_cloud_gaps.bl_idname,
            icon="PARTICLES",
        )


classes = (
    PCGF_Settings,
    OBJECT_OT_fill_point_cloud_gaps,
    VIEW3D_PT_point_cloud_gap_filler,
)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.pcgf_settings = PointerProperty(type=PCGF_Settings)


def unregister():
    del bpy.types.Scene.pcgf_settings
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
