"""
RGB displacement of point set
"""

import open3d as o3d
import numpy as np

scanMesh = o3d.io.read_triangle_mesh("002.ply")
pcl = o3d.geometry.PointCloud()
pcl.points = scanMesh.vertices
print(pcl.points)
pcl.colors = scanMesh.vertex_colors

# making two arrays, with the points and with the colors of the point cloud
col = np.asarray(pcl.colors)
pts = np.asarray(pcl.points)

col = col * 250


def rgb_sorting(r1, g1, b1, r2, g2, b2, displacement):
    color_range = color_array = np.array([
        [r1, g1, b1],  # dark green
        [r2, g2, b2]])  # light green

    mask = np.all(
        np.logical_and(
            np.min(color_range, axis=0) < col,
            col < np.max(color_range, axis=0)
        ),
        axis=-1
    )

    newCol = col[mask] / 250
    newPts = pts[mask]

    displPts = newPts + displacement
    new_pcl = o3d.geometry.PointCloud()
    new_pcl.points = o3d.utility.Vector3dVector(displPts)
    new_pcl.colors = o3d.utility.Vector3dVector(newCol)
    return new_pcl

"""
[120, 120, 100],  # dark green
[130, 140, 120]])  # light green
"""

points01 = rgb_sorting(120,120,100,130,140,120, 20)
points02 = rgb_sorting(0, 85, 0,90, 255,73, 10)
#[0, 85, 0],  # dark green
#[90, 255, 73



#o3d.io.write_point_cloud('rgb_displacement.ply', new_pcl)

o3d.visualization.draw_geometries([pcl, points01,points02])
