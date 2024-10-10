# ----------------------------------------------------------------------------
# -                        Open3D: www.open3d.org                            -
# ----------------------------------------------------------------------------
# Copyright (c) 2018-2023 www.open3d.org
# SPDX-License-Identifier: MIT
# ----------------------------------------------------------------------------

import open3d as o3d

if __name__ == "__main__":

    pcd = o3d.io.read_point_cloud("/home/rvbust/Documents/data.ply")
    fbs = pcd.fit_b_spline(200, degree=3, is_equidistant=False, is_close=True)
    o3d.visualization.draw_geometries([fbs])

