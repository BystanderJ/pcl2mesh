#!/usr/bin/env python3
"""
laz2mesh.py
    1) 从相对路径 LAZROI/bounding_box.txt 读取 ROI
    2) 裁剪 LAZ 点云指定包围盒 (PDAL Python API)
    3) Open3D 估计法向
    4) 将裁剪后的彩色 PLY 重建成带纹理网格 (PyMeshLab)

用法示例:
    python laz2mesh.py cloud.laz --out_dir ./subscene --radius 0.3 0.6 0.9
"""
import argparse
import json
import math
import os
import sys

import open3d as o3d
import pymeshlab as ml

def import_pdal():
    try:
        import pdal
        return pdal
    except ImportError:
        sys.exit("Error: pdal 模块未找到，请确保已安装 PDAL Python API")

def load_roi_from_file(txt_path):
    """
    读取 LAZROI/bounding_box.txt，其中每行格式为：
      X_min <value>
      X_max <value>
      Y_min <value>
      Y_max <value>
      Z_min <value>
      Z_max <value>
    返回 (xmin,xmax,ymin,ymax,zmin,zmax) 元组
    """
    if not os.path.isfile(txt_path):
        sys.exit(f"Error: ROI 文件未找到: {txt_path}")
    roi = {}
    with open(txt_path, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) != 2:
                continue
            key, val = parts
            roi[key] = float(val)
    try:
        return roi['X_min'], roi['X_max'], roi['Y_min'], roi['Y_max'], roi['Z_min'], roi['Z_max']
    except KeyError as e:
        sys.exit(f"Error: ROI 文件缺少字段 {e}")

def main():
    ap = argparse.ArgumentParser(description='LAZ to textured mesh')
    ap.add_argument("in_laz", help="输入 .laz 文件路径")
    ap.add_argument("--out_dir", default=".", help="输出目录")
    ap.add_argument("--radius", nargs='+', type=float,
                    help="Ball-Pivoting 半径 列表")
    ap.add_argument("--knn", type=int, default=10,
                    help="Open3D 法向估计邻域 K")
    args = ap.parse_args()

    in_laz = os.path.abspath(args.in_laz)
    out_dir = os.path.abspath(args.out_dir)
    os.makedirs(out_dir, exist_ok=True)

    # 读取 ROI
    roi_file = os.path.join('LAZROI', 'bounding_box.txt')
    xmin, xmax, ymin, ymax, zmin, zmax = load_roi_from_file(roi_file)

    ply_crop    = os.path.join(out_dir, "cropped_cloud.ply")
    ply_normals = os.path.join(out_dir, "cropped_with_normals.ply")
    obj_path    = os.path.join(out_dir, "mesh_textured.obj")

    # PDAL 裁剪
    pdal = import_pdal()
    bounds = f"([{xmin},{xmax}],[{ymin},{ymax}],[{zmin},{zmax}])"
    pipeline = {
        "pipeline": [
            {"type": "readers.las", "filename": in_laz},
            {"type": "filters.crop", "bounds": bounds},
            {"type": "writers.ply", "filename": ply_crop,
             "dims": "X,Y,Z,Red,Green,Blue",
             "storage_mode": "little endian"}
        ]
    }
    print("➤ PDAL 裁剪…")
    pdal.Pipeline(json.dumps(pipeline)).execute()
    print("   生成:", ply_crop)

    # Open3D 法向估计
    print("➤ Open3D 估计法向 (k =", args.knn, ") …")
    pcd = o3d.io.read_point_cloud(ply_crop)
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=args.knn))
    pcd.orient_normals_consistent_tangent_plane(args.knn*5)
    o3d.io.write_point_cloud(ply_normals, pcd, write_ascii=False)
    print("   写入", ply_normals)

    # PyMeshLab 网格重建与贴纹理
    ms = ml.MeshSet()
    ms.load_new_mesh(ply_normals)

    # Ball Pivoting
    bbox = ms.current_mesh().bounding_box()
    diag = math.dist(bbox.min(), bbox.max())
    radius = args.radius[0] if args.radius else diag * 0.01
    ms.apply_filter(
        'generate_surface_reconstruction_ball_pivoting',
        ballradius=ml.PercentageValue(radius),
        clustering=20.0,
        creasethr=90.0,
        deletefaces=True
    )

    # UV 展开与贴图
    ms.apply_filter(
        'compute_texcoord_parametrization_triangle_trivial_per_wedge',
        textdim=4096
    )
    ms.apply_filter('compute_texcoord_transfer_wedge_to_vertex')
    ms.apply_filter(
        'compute_texmap_from_color',
        textname='color_atlas',
        textw=4096,
        texth=4096
    )

    # 导出 OBJ
    ms.save_current_mesh(obj_path, save_vertex_color=False)
    print("✓ 网格和贴图已导出 →", obj_path)

if __name__ == "__main__":
    main()
