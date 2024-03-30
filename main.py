import vedo
import trimesh
import glob
import open3d as o3d
import numpy as np
import math
from pypcd import pypcd


DATAPATH = "./CrownData/"
label = "6P"
PCLOUTPUTPATH = "../OpenPCDet/data/custom/points/"
LABELOUTPUTPATH = "../OpenPCDet/data/custom/labels/"
IMAGEOUTPUTPATH = "../OpenPCDet/data/custom/ImageSets/"
TRAINTESTSPLIT=(0.8, 0.1, 0.1)




TEMPPATH = "./Temp/temp."
ZFILLNUM = 6

axis_statistics = [math.inf, -math.inf, math.inf, -math.inf, math.inf, -math.inf] #-x x -y y -z z
num = 0
# for folder_name in glob.glob(DATAPATH+'**/'):
#
#     prep = glob.glob(folder_name+"*Preparation.stl")
#     jaw = glob.glob(folder_name + "*Jaw_scan.stl")
#     if len(prep) == 1 and len(jaw) == 1:
#         prep_mesh = o3d.io.read_triangle_mesh(prep[0])
#         jaw_mesh = o3d.io.read_triangle_mesh(jaw[0])
#         vertices = np.asarray(jaw_mesh.vertices)
#         pcd = o3d.geometry.PointCloud(o3d.cpu.pybind.utility.Vector3dVector(vertices))
#         o3d.io.write_point_cloud("result.pcd", pcd)
#
#         vertices =np.hstack([vertices, np.zeros((vertices.shape[0], 1), vertices.dtype)])
#
#         exit(0)
#         np.save(PCLOUTPUTPATH+str(num).zfill(3), vertices)
#         aabb = o3d.geometry.Geometry3D.get_axis_aligned_bounding_box(prep_mesh)
#         with open(LABELOUTPUTPATH+str(num).zfill(3)+".txt", "w") as of:
#             of.write(f"{aabb.get_center()[0]} {aabb.get_center()[1]} {aabb.get_center()[2]} {aabb.get_extent()[0]} "
#                      f"{aabb.get_extent()[1]} {aabb.get_extent()[2]} 0.0 {label}")
#         aabb_jaw = o3d.geometry.Geometry3D.get_axis_aligned_bounding_box(jaw_mesh)
#         maxb = aabb_jaw.get_max_bound()
#         minb = aabb_jaw.get_min_bound()
#         if maxb[0] > axis_statistics[1]:
#             axis_statistics[1] = maxb[0]
#         if maxb[1] > axis_statistics[3]:
#             axis_statistics[3] = maxb[1]
#         if maxb[2] > axis_statistics[5]:
#             axis_statistics[5] = maxb[2]
#
#         if minb[0] < axis_statistics[0]:
#             axis_statistics[0] = minb[0]
#         if minb[1] < axis_statistics[2]:
#             axis_statistics[2] = minb[1]
#         if minb[2] < axis_statistics[4]:
#             axis_statistics[4] = minb[2]
#
#         num = num + 1
#         print(f"the {num}th file is done.")

for folder_name in glob.glob(DATAPATH+'**/'):

    prep = glob.glob(folder_name+"*Preparation.stl")
    jaw = glob.glob(folder_name + "*Jaw_scan.stl")
    if len(prep) == 1 and len(jaw) == 1:
        prep_mesh = o3d.io.read_triangle_mesh(prep[0])
        jaw_mesh = o3d.io.read_triangle_mesh(jaw[0])
        vertices = np.asarray(jaw_mesh.vertices)
        pcd = o3d.geometry.PointCloud(o3d.cpu.pybind.utility.Vector3dVector(vertices))
        o3d.io.write_point_cloud("./Temp/temp.pcd", pcd)
        pcd_data = pypcd.PointCloud.from_path('./Temp/temp.pcd')

        points = np.zeros([pcd_data.width, 4], dtype=np.float32)
        points[:, 0] = pcd_data.pc_data['x'].copy()
        points[:, 1] = pcd_data.pc_data['y'].copy()
        points[:, 2] = pcd_data.pc_data['z'].copy()
        points[:, 3] = 0.0

        with open(PCLOUTPUTPATH+str(num).zfill(ZFILLNUM)+'.bin', 'wb') as f:
            f.write(points.tobytes())


        aabb = o3d.geometry.Geometry3D.get_axis_aligned_bounding_box(prep_mesh)
        with open(LABELOUTPUTPATH+str(num).zfill(ZFILLNUM)+".txt", "w") as of:
            of.write(f"{aabb.get_center()[0]} {aabb.get_center()[1]} {aabb.get_center()[2] - aabb.get_extent()[2]*0.5} {aabb.get_extent()[0]} "
                     f"{aabb.get_extent()[1]} {aabb.get_extent()[2]} 0.0 {label}")
        aabb_jaw = o3d.geometry.Geometry3D.get_axis_aligned_bounding_box(jaw_mesh)
        maxb = aabb_jaw.get_max_bound()
        minb = aabb_jaw.get_min_bound()
        if maxb[0] > axis_statistics[1]:
            axis_statistics[1] = maxb[0]
        if maxb[1] > axis_statistics[3]:
            axis_statistics[3] = maxb[1]
        if maxb[2] > axis_statistics[5]:
            axis_statistics[5] = maxb[2]

        if minb[0] < axis_statistics[0]:
            axis_statistics[0] = minb[0]
        if minb[1] < axis_statistics[2]:
            axis_statistics[2] = minb[1]
        if minb[2] < axis_statistics[4]:
            axis_statistics[4] = minb[2]

        num = num + 1
        print(f"the {num}th file is done.")


# num=156


print("[-x, x, -y, y, -z, z] : " + str(axis_statistics))
with open("./statistics.txt", "w") as of:
    of.write(str(axis_statistics))

# with open(IMAGEOUTPUTPATH+"train.txt", "w") as of:
#     for i in range(120):
#         of.write(str(i).zfill(ZFILLNUM)+"\n")
#
# with open(IMAGEOUTPUTPATH+"val.txt", "w") as of:
#     for i in range(121, 140):
#         of.write(str(i).zfill(ZFILLNUM)+"\n")
#
# with open(IMAGEOUTPUTPATH+"test.txt", "w") as of:
#     for i in range(140, num):
#         of.write(str(i).zfill(ZFILLNUM)+"\n")












