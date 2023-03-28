import numpy as np
import open3d as o3d

pcd = o3d.io.read_point_cloud("../data/rvRansacModel/plane.ply")
plance_w, plance_index = pcd.fit_plane_ransac(distance_threshold=0.002, num_iterations=100)
plane = pcd.select_by_index(plance_index)
plane.paint_uniform_color([1, 0, 0])
o3d.visualization.draw_geometries([pcd, plane])

pcd = o3d.io.read_point_cloud("../data/rvRansacModel/sphere.ply")
pcd = pcd.uniform_down_sample(5)
sphere_w, sphere_index = pcd.fit_sphere_ransac(distance_threshold=0.0002, num_iterations=100)
sphere = pcd.select_by_index(sphere_index)
sphere.paint_uniform_color([1, 0, 0])
o3d.visualization.draw_geometries([pcd, sphere])

pcd = o3d.io.read_point_cloud("../data/rvRansacModel/circle3d.ply")
pcd = pcd.uniform_down_sample(5)
circle_w, circle_index = pcd.fit_circle3D_ransac(distance_threshold=0.0005, num_iterations=100)
circle = pcd.select_by_index(circle_index)
circle.paint_uniform_color([1, 0, 0])
o3d.visualization.draw_geometries([pcd, circle])
