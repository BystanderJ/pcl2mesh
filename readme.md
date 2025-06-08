# LAZ-to-Mesh Pipeline

## 项目简介

本工程用于从 `.laz` 点云中交互式提取感兴趣区域 (ROI)，并生成带纹理的三角网格。  
主要包含两个阶段：

1. **ROI 提取 (MATLAB)**  
   - 在 `LAZROI/ROI_new.m` 脚本中，用户通过交互式界面点击若干边界点，脚本会计算并保存最小轴对齐包围盒坐标到 `LAZROI/bounding_box.txt`。  

2. **点云裁剪与网格重建 (Python)**  
   - 回到项目主目录，运行 `laz2mesh.py` 脚本：  
     - 自动读取 `LAZROI/bounding_box.txt`，  
     - 使用 PDAL 裁剪 `.laz` 点云为 PLY，  
     - 使用 Open3D 估计法向，  
     - 使用 PyMeshLab 进行 Ball Pivoting 重建并贴纹理，  
     - 输出带材质的 OBJ 网格文件。

---

## 目录结构

.
├── LAZROI
│ ├── ROI_new.m # 交互式提取 ROI
│ └── bounding_box.txt # 自动生成的包围盒坐标
├── laz2mesh.py # 点云裁剪 & 网格重建
└── README.md # 本说明文件


---

## 依赖
* **MATLAB** R2022a+ （需支持 `lasFileReader`、`pcshow` 等函数）  
* **Python** 3.8+，并安装  
  * `pdal`  
  * `open3d`  
  * `pymeshlab`  

---

## 使用步骤

### 1. 交互式提取 ROI
1. 在 MATLAB 中进入 `LAZROI` 目录。  
2. 运行 `ROI_new`。  
3. 在弹出的点云窗口，用「数据光标」点击至少 2 个边界点（推荐 4–8 个）。  
4. 点击窗口左下角 **Done**，即生成 `bounding_box.txt`。  

### 2. 裁剪点云并生成网格
1. 回到项目根目录。  
2. 运行  

   `python laz2mesh.py <your_cloud.laz> --out_dir <output_folder>`  

   可选参数  
   * `--radius r1 r2 …`  Ball-Pivoting 半径列表  
   * `--knn K`  Open3D 法向估计邻域大小 (默认 10)  

3. 处理完成后，输出目录将包含  
   * `cropped_cloud.ply`  
   * `cropped_with_normals.ply`  
   * `mesh_textured.obj` 及纹理 PNG  

---


