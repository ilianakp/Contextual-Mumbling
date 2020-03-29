"""
RGB displacement of point set
"""


import open3d as o3d
import numpy as np

"""
IDEAS
- assigning the colors of the points from one pcl to another
  the two pcls need to have the same number of points
  but ok with cloud compare
"""

# if it is a mesh instead of a pcl
#scanMesh = o3d.io.read_point_cloud("candles.ply")
#pcl = o3d.geometry.PointCloud()
#pcl.points = scanMesh.vertices
#pcl.colors = scanMesh.vertex_colors

pcl = o3d.io.read_point_cloud("002.ply")

# making two arrays, with the points and with the colors of the point cloud
col = np.asarray(pcl.colors)
pts = np.asarray(pcl.points)

# that means that it's creating a new number from r+g+b and add that number to every point's coordinates
# next step--> movement based on normal
#new_col = col[:,0]+col[:,1]+col[:,2]

# you can change that
displacement = 4

# x + r*displacement, y + g*displacement, z + b*displacement
new_pts = pts + (col*displacement)
print(new_pts)

# it's assigning the displaced points with their colors to the point cloud
pcl_new = o3d.geometry.PointCloud()
pcl_new.points = o3d.utility.Vector3dVector(new_pts)
pcl_new.colors = o3d.utility.Vector3dVector(col)
pcl_new.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=0.1, max_nn=30))

# vizualization
o3d.visualization.draw_geometries([pcl])
o3d.visualization.draw_geometries([pcl_new, pcl])

# write a file with the displaced pcl
o3d.io.write_point_cloud("displaced_pcl.ply", pcl_new)


# makes a pcl from the initial and the displaced
pcl_joined = o3d.geometry.PointCloud()
pcl_joined.points = o3d.utility.Vector3dVector(new_pts+pts)
pcl_joined.colors = o3d.utility.Vector3dVector(col+col)
pcl_joined.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=0.1, max_nn=30))

# write a file with the displaced pcl
#o3d.io.write_point_cloud("joined_pcl.ply", pcl_joined)

# creates a mesh from the joined pcl
mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
    pcl_joined, depth=8)

# created a file with the mesh
#o3d.io.write_triangle_mesh("mesh02.obj",
#                           mesh,
#                           write_triangle_uvs=True)
o3d.visualization.draw_geometries([mesh])