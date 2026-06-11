# Point Cloud Gap Filler

A Blender add-on that densifies a point cloud by adding an exact number of
new points between nearby source points.

## Install

1. Zip the `point_cloud_gap_filler` folder, or use the provided
   `point_cloud_gap_filler.zip`.
2. In Blender, open **Edit > Preferences > Add-ons**.
3. Choose **Install from Disk**, select the ZIP, and enable the add-on.
4. In the 3D Viewport, press **N** and open the **Point Cloud** tab.

## Use

1. Select a Blender Point Cloud object or a mesh whose vertices are the points.
2. Set **Points to Add**.
3. Click **Fill Point Cloud Gaps**.

The source object is left unchanged. The result is created as a new object and
inherits the source object's transform.

## Controls

- **Points to Add**: Exact number of generated points.
- **Neighbors**: Number of nearby points used to measure local spacing.
- **Gap Bias**: Concentrates sampling in sparse areas. `0` samples uniformly;
  values around `1.0` to `2.0` work well for most clouds.
- **Average Spread**: `0` creates exact pairwise midpoints. Higher values place
  weighted averages along the gaps, reducing duplicate samples.
- **Seed**: Makes results repeatable.
- **Include Original Points**: Includes source points in the output object.
- **Output Type**: Match the source, create a Point Cloud, or create a
  vertex-only Mesh.

Native Point Cloud creation requires Blender 5.1 or newer. Blender 4.4 through
5.0 can read Point Cloud inputs, but the Python API cannot resize new Point
Cloud data; on those versions the add-on automatically creates a vertex-only
Mesh instead.

## Method

The add-on builds a KD-tree, measures each point's local nearest-neighbor gap,
and gives wider gaps a higher sampling probability. Every generated position is
a weighted average of two nearby points. Generation happens in doubling rounds,
so new points participate in later refinement and progressively subdivide gaps.

Only point positions are interpolated. Custom attributes such as color, radius,
normals, and classification labels are not copied in version 1.0.

Very large outputs can require substantial RAM and processing time. Save the
Blend file before generating millions of points.
