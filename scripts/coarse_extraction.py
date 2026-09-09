import os
import glob
import time
import numpy as np
import open3d as o3d
from scipy.ndimage import label, generate_binary_structure
from collections import defaultdict
import pyproj

# ==========================================
# 可调节参数 (Adjustable Parameters)
# ==========================================
INPUT_DIR = "../submap"            # 局部地图输入目录
OUTPUT_DIR = "../poles"            # 粗提取结果存放目录
FILE_PREFIX = "merged_map_"        # 局部地图命名前缀
START_MAP_IDX = 0                  # 开始处理的地图编号 (如 merged_map_2.pcd)
END_MAP_IDX = 0                    # 结束处理的地图编号

# 1. 粗提取参数
GRID_RES = 0.5                        # 2D栅格分辨率 (m)
N_THRESH = 60                         # 栅格内最小点数阈值（根据点云稠密程度调）
DZ_THRESH = 8.0                       # 栅格内最小高度跨度阈值 (m)
N_LAYERS_THRESH = 40                  # 栅格内最大连续占据层数阈值 (要求必须有一段连续层数 >= N_LAYERS_THRESH)
LAYER_RES = 0.2                       # 高度分层分辨率 (m)

# 2. ROI 与 分块 参数
USE_ROI = True                         # 是否使用 ROI 过滤
ROI_Y_MIN = -10.0                      # 限制只取右侧10米
ROI_Y_MAX = 10.0                       # 限制只取左侧10米
ROI_Z_MAX = 10.0                       # 相对于轨迹的最大高度限制 (m)
ROI_Z_MIN = -5.0                       # 相对于轨迹的最小高度限制 (m)，取决于雷达安装高度
CHUNK_LENGTH = 1000.0                  # 按轨迹分块的距离 (m)

# 3. 细提取参数（形态学与聚类过滤）
EPS = 1.0                              # DBSCAN 聚类的邻域半径 (m)，欧式聚类的距离阈值 (m)，RANSAC 的最大半径 (m)
CLUSTERING_METHOD = 'EUCLIDEAN'        # 'RANSAC', 'DBSCAN' 或 'EUCLIDEAN'
MAX_AREA_CELLS = 8                     # 形态学过滤：最大允许的连续网格数
MAX_RADIUS = 0.8                       # 聚类过滤：路标的最大半径阈值 (m)（暂时没用到）
MAX_SPAN = 1.0                         # 聚类过滤：路标在XY平面的最大长宽跨度阈值 (m)
MIN_TOTAL_POINTS = 50                  # 聚类过滤：单根支柱最少雷达点数
MAX_TOTAL_POINTS = 5000                # 聚类过滤：单根支柱最多雷达点数

# 4. GPS 逆投影输出参数
GEOSOT_LEVEL = 20                       # GeoSOT 网格层级 (根据精度需求调整，15级约几米级)
OUTPUT_GPS = True                       # 是否将提取出来的局部坐标路标点转换回地理经纬度
LAT0 = 32.1648016                       # 初始原点纬度 (Latitude) 
LON0 = 112.14373859999999               # 初始原点经度 (Longitude) 
ALT0 = 64.15                            # 初始原点海拔 (Altitude) 

from scipy.spatial import cKDTree
from geosot import GeoSOT  # 引入我们刚才写的库

def read_pcd_with_intensity(filepath):
    """
    由于 Open3D 传统 API 不支持原生的 intensity 字段，
    这里如果直接用 o3d.io.read_point_cloud 会丢失强度。
    由于原始的局部地图可能是利用 merge_keyframes.py 生成的 (将强度伪装成了颜色 R 通道)，
    如果颜色存在，则从颜色还原；如果文件是直接带 intensity 字段的标准二进制/ASCII PCD，
    则只能使用 open3d tensor API 或者人工读取。这里统一采用 tensor API 以防万一。
    """
    t_pcd = o3d.t.io.read_point_cloud(filepath)
    
    # 提取坐标
    points = t_pcd.point.positions.numpy()
    
    # 构建传统的 o3d pcd 用于现有算法管线
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    # 提取强度 (寻找 intensity, 或者是 R 通道伪装)
    if 'intensity' in t_pcd.point:
        intensities = t_pcd.point.intensity.numpy().flatten()
        # 伪装回颜色的 R 通道（和 merge_keyframes 逻辑一致，保证能被各种处理和下采样支持）
        colors = np.zeros((len(points), 3), dtype=np.float32)
        colors[:, 0] = intensities
        pcd.colors = o3d.utility.Vector3dVector(colors)
    elif len(pcd.colors) == 0:
        # 如果既没有 intensity 也没有 colors，则创建一个全 1 的颜色
        colors = np.ones((len(points), 3), dtype=np.float32)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        
    # 如果原本通过 t_pcd 读取时就已经自带 colors 字段（你之前的伪装逻辑），
    # 那么 tensor API 也会读取进 colors 中，转过去会自动保留。
    elif 'colors' in t_pcd.point:
        pcd.colors = o3d.utility.Vector3dVector(t_pcd.point.colors.numpy() / 255.0 if t_pcd.point.colors.numpy().dtype == np.uint8 else t_pcd.point.colors.numpy())

    return pcd

def save_pcd_with_intensity(filename, pcd):
    """
    将 Open3D 中的伪装强度 (R通道) 还原并保存为标准的 PCD v0.7 文件
    与 merge_keyframes.py 的保存逻辑完全一致
    """
    points = np.asarray(pcd.points)
    if len(pcd.colors) > 0:
        intensities = np.asarray(pcd.colors)[:, 0]  # First column stores the intensity
    else:
        intensities = np.ones(len(points), dtype=np.float32)
        
    num_points = points.shape[0]
    points = points.astype(np.float32)
    intensities = intensities.astype(np.float32)
    
    with open(filename, 'wb') as f:
        header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH {num_points}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {num_points}
DATA binary
"""
        f.write(header.encode('ascii'))
        
        # Interleave X, Y, Z, Intensity into an N x 4 array
        cloud_data = np.zeros((num_points, 4), dtype=np.float32)
        cloud_data[:, :3] = points
        cloud_data[:, 3] = intensities.flatten()
        f.write(cloud_data.tobytes())

def filter_pcd_by_roi(pcd, poses, y_min=-10.0, y_max=10.0, z_min=-5.0, z_max=15.0):
    """
    根据轨迹线横向距离(Y)过滤点云，构建轨道 ROI (带高度截断)
    pcd: open3d.geometry.PointCloud
    poses: Nx3 or Nx2 np.darray (tx, ty, ...)
    """
    points = np.asarray(pcd.points)
    if len(points) == 0 or len(poses) == 0:
        return pcd
    
    # 建立 KD-Tree (仅使用 XY 二维距离)检索最近轨迹点
    traj_tree = cKDTree(poses[:, :2])
    # 查询所有点到轨迹最近点的距离和最近位姿的索引
    dists, nearest_idx = traj_tree.query(points[:, :2])
    
    # 计算轨迹的前向切向量以区分左右 (横向 Y)
    tangents = np.zeros_like(poses[:, :2])
    n = len(poses)
    if n > 1:
        tangents[1:-1] = poses[2:, :2] - poses[:-2, :2]
        tangents[0] = poses[1, :2] - poses[0, :2]
        tangents[-1] = poses[-1, :2] - poses[-2, :2]
    else:
        tangents = np.array([[1.0, 0.0]])
    norms = np.linalg.norm(tangents, axis=1, keepdims=True)
    norms[norms == 0] = 1e-6
    tangents = tangents / norms
    
    # 法向量计算: 逆时针旋转 90 度，前进方向的左侧为正 Y (即 vector: [-dy, dx])
    left_vectors = np.empty_like(tangents)
    left_vectors[:, 0] = -tangents[:, 1]
    left_vectors[:, 1] = tangents[:, 0]
    
    # 投影计算相对横向距离 rel_y
    rel_vec_xy = points[:, :2] - poses[nearest_idx, :2]
    rel_y = np.sum(rel_vec_xy * left_vectors[nearest_idx], axis=1)
    
    # 在拥有 Z 坐标的情况下，计算相对高程
    if poses.shape[1] >= 3:
        nearest_z = poses[nearest_idx, 2]
        rel_z = points[:, 2] - nearest_z
        # 限制 Y 横向距离与 Z 轴相对高度
        mask = (rel_y >= y_min) & (rel_y <= y_max) & (rel_z >= z_min) & (rel_z <= z_max)
    else:
        mask = (rel_y >= y_min) & (rel_y <= y_max)
        
    return pcd.select_by_index(np.where(mask)[0])

def coarse_extract_poles(pcd_ng, grid_res=0.15, layer_res=0.2, N_thresh=6, dz_thresh=1.5, n_layers_thresh=5, max_area_cells=16, debug_prefix=""):
    """
    2D栅格法粗提取柱状物
    """
    points = np.asarray(pcd_ng.points)
    if len(points) == 0:
        return None, [] 

    # XY 投影到栅格
    x_min, y_min = np.min(points[:, :2], axis=0)
    x_max, y_max = np.max(points[:, :2], axis=0)
    
    # 计算栅格尺寸
    grid_w = int(np.ceil((x_max - x_min) / grid_res)) + 1
    grid_h = int(np.ceil((y_max - y_min) / grid_res)) + 1
    
    grid_coords = np.floor((points[:, :2] - [x_min, y_min]) / grid_res).astype(np.int32)
    
    # 高度分层
    z_min_global = np.min(points[:, 2])
    z_layers = np.floor((points[:, 2] - z_min_global) / layer_res).astype(np.int32)
    
    # 统计栅格信息
    grid_stats = defaultdict(lambda: {'N': 0, 'z_min': float('inf'), 'z_max': float('-inf'), 'layers': set()})
    
    for i, (gx, gy) in enumerate(grid_coords):
        z = points[i, 2]
        layer = z_layers[i]
        
        stat = grid_stats[(gx, gy)]
        stat['N'] += 1
        stat['z_min'] = min(stat['z_min'], z)
        stat['z_max'] = max(stat['z_max'], z)
        stat['layers'].add(layer)
        
    # 筛选候选格
    mask_2d = np.zeros((grid_w, grid_h), dtype=bool)
    
    for (gx, gy), stat in grid_stats.items():
        dz = stat['z_max'] - stat['z_min']
        
        # 寻找该网格内的所有曾存在点的层级
        sorted_layers = sorted(list(stat['layers']))
        max_continuous_layers = 0
        if sorted_layers:
            current_continuous = 1
            max_continuous_layers = 1
            for i in range(1, len(sorted_layers)):
                # 如果两层相邻
                if sorted_layers[i] == sorted_layers[i-1] + 1:
                    current_continuous += 1
                    if current_continuous > max_continuous_layers:
                        max_continuous_layers = current_continuous
                else:
                    current_continuous = 1
        
        if stat['N'] > N_THRESH and dz > DZ_THRESH and max_continuous_layers >= N_LAYERS_THRESH:
            mask_2d[gx, gy] = True
            
    # 【新增功能】输出 2D 拍扁的粗提取栅格布尔图像
    if debug_prefix:
        try:
            import cv2
            # 将布尔型 mask_2d (True=发亮的特征栅格, False=黑背景) 转为黑白图像 (255=白, 0=黑)
            img_2d = (mask_2d.T.astype(np.uint8)) * 255
            
            # 翻转一下 Y 轴，让它符合一般点云俯视图的直觉 (非强求)
            img_2d = np.flipud(img_2d)
            
            # 为了更清楚，将细小的点膨胀一点便于可视化观感
            from scipy.ndimage import binary_dilation
            img_2d_dilated = binary_dilation(img_2d > 0, iterations=1).astype(np.uint8) * 255

            cv2.imwrite(f"../poles/debug_mask_{debug_prefix}.png", img_2d_dilated)
        except Exception as e:
            print(f"Warning: Failed to save debug mask image. ({e})")

    # 2D 连通域聚类
    struct = generate_binary_structure(2, 2) # 8连通
    
    # ---------------- 关键修改在这里 ----------------
    # 为了解决一根柱子正好落在两个栅格中间，被判定为独立的两个聚类的情况，
    # 我们对 mask_2d 进行一次形态学膨胀(Dilation) + 腐蚀(Erosion) 即闭运算。
    # 这样距离很近(相邻)的两个栅格就会被桥接(融合)成同一个连通域，然后再找连通域。
    from scipy.ndimage import binary_closing
    mask_2d = binary_closing(mask_2d, structure=struct, iterations=1)
    
    labeled_array, num_features = label(mask_2d, structure=struct)
    
    if num_features == 0:
        return None, []
        
    # [新增防线1] 限制连通域最大面积，剔除连成一片的墙面/建筑外立面
    component_sizes = np.bincount(labeled_array.ravel())
    
    valid_cluster_ids = set()
    for i in range(1, num_features + 1):
        if component_sizes[i] <= max_area_cells:
            valid_cluster_ids.add(i)
            
    # 收集满足条件的 3D 点索引，使用 select_by_index 可保留 intensity 等附加属性
    valid_indices = []
    coarse_centers = []
    cluster_points_map = defaultdict(list)
    for i, (gx, gy) in enumerate(grid_coords):
        cluster_id = labeled_array[gx, gy]
        if cluster_id in valid_cluster_ids:  # 必须属于小面积(非墙面)的候选区域
            valid_indices.append(i)
            cluster_points_map[cluster_id].append(i)
            
    if not valid_indices:
        return None, []
        
    for cid, indices in cluster_points_map.items():
        pts = points[indices]
        cx, cy = np.mean(pts[:, :2], axis=0)
        cz = np.mean(pts[:, 2])
        coarse_centers.append([cx, cy, cz, len(indices)])
        
    extracted_pcd = pcd_ng.select_by_index(valid_indices)
        
    return extracted_pcd, coarse_centers

def extract_pole_centers(extracted_pcd, poses, eps=0.8, min_points=30, max_radius_thresh=0.8, max_span_thresh=1.5, min_total_points=50, max_total_points=5000):
    """
    对粗提取的点云进行 DBSCAN 或 RANSAC 聚类，为每根支柱计算稳定的 (X, Y, Z) 坐标，
    并返回提纯后绝对符合条件的路标点云簇。
    """
    if extracted_pcd is None or len(extracted_pcd.points) == 0:
        return [], None

    pole_centers = []
    refined_points_list = []
    refined_colors_list = []
    
    # 建立 KD-Tree 以便计算最近位姿 (用 XY)
    traj_tree = None
    if poses is not None and len(poses) > 0:
        traj_tree = cKDTree(poses[:, :2])
        
    points = np.asarray(extracted_pcd.points)
    colors = np.asarray(extracted_pcd.colors) if len(extracted_pcd.colors) > 0 else None
    
    remaining_points = points.copy()
    remaining_colors = colors.copy() if colors is not None else None
    
    # === 根据配置选择聚类算法 ===
    if CLUSTERING_METHOD == 'DBSCAN':
        # DBSCAN 聚类
        pcd_for_clustering = o3d.geometry.PointCloud()
        pcd_for_clustering.points = o3d.utility.Vector3dVector(remaining_points)
        labels = np.array(pcd_for_clustering.cluster_dbscan(eps=1.5, min_points=25, print_progress=False))
        
        max_label = labels.max()
        if max_label < 0:
             return [], None
             
        for cluster_idx in range(max_label + 1):
            cluster_indices = np.where(labels == cluster_idx)[0]
            cluster_points = remaining_points[cluster_indices]
            cluster_colors = remaining_colors[cluster_indices] if remaining_colors is not None else None
            
            # [防线2] 计算类簇在 XY 平面的跨度(长宽)，过滤掉厚墙
            xy_pts = cluster_points[:, :2]
            xy_span = np.max(xy_pts, axis=0) - np.min(xy_pts, axis=0)
            
            if xy_span[0] > max_span_thresh or xy_span[1] > max_span_thresh: 
                continue
                
            # 2. X, Y 采用类簇的中心
            center_x, center_y = np.mean(xy_pts, axis=0)
            
            # 半径检测
            distances = np.linalg.norm(xy_pts - [center_x, center_y], axis=1)
            cluster_radius = np.max(distances)
            
            center_z = np.mean(cluster_points[:, 2]) # 默认 Z
            
            # 3. Z 高程稳定性投影：绑定到最近轨道关键帧姿态的 Z
            if traj_tree is not None:
                _, nearest_idx = traj_tree.query([center_x, center_y])
                nearest_pose_z = poses[nearest_idx, 2]
                center_z = nearest_pose_z  # 强制将 Z 锚定为轨迹高程
                
            # 4. 统计额外特征与点数过滤
            num_points_in_cluster = len(cluster_points)
            if num_points_in_cluster < min_total_points or num_points_in_cluster > max_total_points:
                continue
                
            pole_centers.append([center_x, center_y, center_z, cluster_radius, xy_span[0], xy_span[1], num_points_in_cluster])
            refined_points_list.append(cluster_points)
            if cluster_colors is not None:
                refined_colors_list.append(cluster_colors)
    elif CLUSTERING_METHOD == 'EUCLIDEAN':
        # === 欧式聚类 (Euclidean Clustering) ===
        # 使用 Open3D 的 cluster_dbscan 进行快速的欧式距离聚类 (min_points=1 则等价于欧式聚类)
        pcd_for_clustering = o3d.geometry.PointCloud()
        pcd_for_clustering.points = o3d.utility.Vector3dVector(remaining_points)
        
        # 欧式聚类不需要核心点的概念，只要距离小于 eps 即视为连通
        labels = np.array(pcd_for_clustering.cluster_dbscan(eps=eps, min_points=1, print_progress=False))
        
        max_label = labels.max()
        if max_label >= 0:
            for cluster_idx in range(max_label + 1):
                cluster_indices = np.where(labels == cluster_idx)[0]
                cluster_points = remaining_points[cluster_indices]
                
                # 如果点数不符合，直接跳过 (DBSCAN min_points=1 会产生很多小簇)
                if len(cluster_indices) < min_points or len(cluster_indices) < min_total_points or len(cluster_indices) > max_total_points:
                    continue
                    
                cluster_colors = remaining_colors[cluster_indices] if remaining_colors is not None else None
                
                # [防线2] 计算类簇在 XY 平面的跨度(长宽)，过滤掉厚墙
                xy_pts = cluster_points[:, :2]
                xy_span = np.max(xy_pts, axis=0) - np.min(xy_pts, axis=0)
                
                if xy_span[0] > max_span_thresh or xy_span[1] > max_span_thresh: 
                    continue
                    
                # 2. X, Y 采用类簇的中心
                center_x, center_y = np.mean(xy_pts, axis=0)
                
                # 半径检测
                distances = np.linalg.norm(xy_pts - [center_x, center_y], axis=1)
                cluster_radius = np.max(distances)
                
                center_z = np.mean(cluster_points[:, 2]) # 默认 Z
                
                # 3. Z 高程稳定性投影：绑定到最近轨道关键帧姿态的 Z
                if traj_tree is not None:
                    _, nearest_idx = traj_tree.query([center_x, center_y])
                    nearest_pose_z = poses[nearest_idx, 2]
                    center_z = nearest_pose_z - 5.0 # 强制将 Z 锚定为轨迹高程
                    
                num_points_in_cluster = len(cluster_points)
                pole_centers.append([center_x, center_y, center_z, cluster_radius, xy_span[0], xy_span[1], num_points_in_cluster])
                refined_points_list.append(cluster_points)
                if cluster_colors is not None:
                    refined_colors_list.append(cluster_colors)
    else:
        # === 用 Multi-RANSAC 拟合垂直柱体替代 DBSCAN ===
        max_ransac_iters = 50 # RANSAC 迭代次数，速度极快
        
        while len(remaining_points) >= min_points:
            best_inliers = []
            best_cx, best_cy = 0.0, 0.0
            
            # RANSAC: 随机选点作为中心找柱子
            for _ in range(max_ransac_iters):
                idx = np.random.randint(len(remaining_points))
                cx, cy = remaining_points[idx, :2]
                
                # 计算剩余点到假设柱中心的 2D 距离
                dists = np.linalg.norm(remaining_points[:, :2] - [cx, cy], axis=1)
                # 距离在最大半径内即算作该圆柱内点
                inliers = np.where(dists <= max_radius_thresh)[0]
                
                if len(inliers) > len(best_inliers):
                    best_inliers = inliers
                    best_cx, best_cy = cx, cy
                    
            # 如果找不出点数足够的路标，说明剩下的都是噪点，结束聚类
            if len(best_inliers) < min_points:
                break
                
            cluster_points = remaining_points[best_inliers]
            cluster_colors = remaining_colors[best_inliers] if remaining_colors is not None else None
                
            # [防线2] 计算类簇在 XY 平面的跨度(长宽)，过滤掉厚墙
            xy_pts = cluster_points[:, :2]
            xy_span = np.max(xy_pts, axis=0) - np.min(xy_pts, axis=0)
            
            if xy_span[0] > max_span_thresh or xy_span[1] > max_span_thresh: 
                # 即使是墙，也要把它从待处理池里删掉，免得死循环
                remaining_points = np.delete(remaining_points, best_inliers, axis=0)
                if remaining_colors is not None: remaining_colors = np.delete(remaining_colors, best_inliers, axis=0)
                continue
                
            # 2. X, Y 采用类簇的中心
            center_x, center_y = np.mean(xy_pts, axis=0)
            
            # 半径检测，计算点到中心的距离作为半径
            distances = np.linalg.norm(xy_pts - [center_x, center_y], axis=1)
            cluster_radius = np.max(distances)

            center_z = np.mean(cluster_points[:, 2]) # 默认 Z
            
            # 3. Z 高程稳定性投影：绑定到最近轨道关键帧姿态的 Z
            if traj_tree is not None:
                _, nearest_idx = traj_tree.query([center_x, center_y])
                nearest_pose_z = poses[nearest_idx, 2]
                center_z = nearest_pose_z  # 强制将 Z 锚定为轨迹高程
                
            # 4. 统计额外特征与点数过滤
            num_points_in_cluster = len(cluster_points)
            if num_points_in_cluster < min_total_points or num_points_in_cluster > max_total_points:
                remaining_points = np.delete(remaining_points, best_inliers, axis=0)
                if remaining_colors is not None: remaining_colors = np.delete(remaining_colors, best_inliers, axis=0)
                continue
                
            pole_centers.append([center_x, center_y, center_z, cluster_radius, xy_span[0], xy_span[1], num_points_in_cluster])
            refined_points_list.append(cluster_points)
            if cluster_colors is not None:
                refined_colors_list.append(cluster_colors)
            
            # ★ 剥离提取出的这个柱子，准备找下一根
            remaining_points = np.delete(remaining_points, best_inliers, axis=0)
            if remaining_colors is not None:
                remaining_colors = np.delete(remaining_colors, best_inliers, axis=0)
        
    refined_pcd = None
    if len(refined_points_list) > 0:
        refined_pcd = o3d.geometry.PointCloud()
        refined_pcd.points = o3d.utility.Vector3dVector(np.vstack(refined_points_list))
        if len(refined_colors_list) > 0:
            refined_pcd.colors = o3d.utility.Vector3dVector(np.vstack(refined_colors_list))

    return pole_centers, refined_pcd

def local_to_gps(local_points, lat0, lon0, alt0):
    """
    使用 pyproj 将局部 ENU 坐标系 (X, Y, Z) 转化回 (纬度, 经度, 海拔)
    local_points: Nx3 array (X：东, Y：北, Z：天)
    """
    # 构建一个以 (lat0, lon0, alt0) 为原点的局部东北天 (ENU) 投影系
    proj_string = f"+proj=tmerc +lat_0={lat0} +lon_0={lon0} +k=1 +x_0=0 +y_0=0 +ellps=WGS84 +units=m +vunits=m +no_defs"
    tmerc = pyproj.Proj(proj_string)
        
    gps_points = []
    for pt in local_points:
        # ENU 中 X是东(相当于投影里的x), Y是北(相当于投影里的y)
        # tmerc(x, y, inverse=True) 会把局部平面坐标转回经纬度
        lon, lat = tmerc(pt[0], pt[1], inverse=True)
        # 高度直接加上参考海拔
        h = alt0 + pt[2]
        gps_points.append([lat, lon, h])
    return np.array(gps_points)

def process_directory(input_dir, output_dir=None, start_idx=START_MAP_IDX, end_idx=END_MAP_IDX):
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
    print(f"Starting extraction from {FILE_PREFIX}{start_idx}.pcd to {FILE_PREFIX}{end_idx}.pcd in {input_dir}")
    
    combined_pcd = o3d.geometry.PointCloud()
    refined_combined_pcd = o3d.geometry.PointCloud()
    all_land_marks = [] # 用于保存所有局部地图提取的路标点
    all_coarse_landmarks = [] # 用于保存粗提取的连通域地标点
    
    for i in range(start_idx, end_idx + 1):
        map_start_time = time.time()
        file_name = f"{FILE_PREFIX}{i}.pcd"
        pcd_file = os.path.join(input_dir, file_name)
        
        if not os.path.exists(pcd_file):
            print(f"File not found, skipping: {file_name}")
            continue
            
        print(f"Processing {file_name}...")
        t0 = time.time()
        pcd = read_pcd_with_intensity(pcd_file)
        t_io = time.time() - t0
        print(f"  [Time] Read PCD: {t_io:.3f}s")
        
        # 0. ROI 过滤
        current_poses = None
        t0 = time.time()
        if USE_ROI:
            # 找到对应局部地图的位姿文件
            poses_file = os.path.join(input_dir, f"{FILE_PREFIX}{i}_poses.txt")
            if os.path.exists(poses_file):
                print(f"  -> Found poses, applying ROI filter (Y: {ROI_Y_MIN} to {ROI_Y_MAX}, Z: {ROI_Z_MIN} to {ROI_Z_MAX})...")
                # 读取位姿
                data = np.loadtxt(poses_file)
                if len(data) > 0 and data.shape[1] >= 4:
                    poses = data[:, 1:4]  # tx, ty, tz (TUM 格式)
                    current_poses = poses
                    pcd = filter_pcd_by_roi(pcd, poses, y_min=ROI_Y_MIN, y_max=ROI_Y_MAX, z_min=ROI_Z_MIN, z_max=ROI_Z_MAX)
                    print(f"  -> Points after ROI filtering: {len(pcd.points)}")
                    
                    # 输出 ROI 裁剪后的全景 PCD (方便可视化检查裁剪是否正确)
                    if output_dir and len(pcd.points) > 0:
                        roi_out_path = os.path.join(output_dir, f"roi_{file_name}")
                        save_pcd_with_intensity(roi_out_path, pcd)
                        print(f"  -> Saved ROI restricted cloud to {roi_out_path}")
            else:
                print(f"  -> Ported poses file not found: {poses_file}. Skipping ROI.")
        t_roi = time.time() - t0
        print(f"  [Time] ROI Filtering: {t_roi:.3f}s")
        
        # 0.5 根据轨迹按距离分块处理防止 OOM
        t0 = time.time()
        if current_poses is not None and len(current_poses) > 1:
            print(f"  -> Splitting point cloud along trajectory by {CHUNK_LENGTH}m chunks...")
            # 计算轨迹的累计距离
            diffs = np.linalg.norm(np.diff(current_poses[:, :3], axis=0), axis=1)
            cum_dist = np.insert(np.cumsum(diffs), 0, 0.0)
            
            pcd_points = np.asarray(pcd.points)
            pcd_tree = cKDTree(pcd_points[:, :2]) if len(pcd_points) > 0 else None
            
            total_dist = cum_dist[-1]
            num_chunks = int(np.ceil(total_dist / CHUNK_LENGTH))
            
            extracted_sub_pcds = []
            
            for chunk_idx in range(num_chunks):
                start_dist = chunk_idx * CHUNK_LENGTH
                end_dist = (chunk_idx + 1) * CHUNK_LENGTH
                
                # 找到属于当前距离段的位姿
                pose_mask = (cum_dist >= start_dist) & (cum_dist < end_dist)
                chunk_poses = current_poses[pose_mask]
                
                if len(chunk_poses) == 0:
                    continue
                    
                # 查找在该段轨迹附近的点云 (带一点膨胀防止漏掉边缘)
                chunk_tree = cKDTree(chunk_poses[:, :2])
                max_lat_dist = max(abs(ROI_Y_MIN), abs(ROI_Y_MAX))
                dists, _ = chunk_tree.query(pcd_points[:, :2], distance_upper_bound=max_lat_dist + 2.0)
                valid_mask = dists <= (max_lat_dist + 2.0)
                valid_indices = np.where(valid_mask)[0]
                
                if len(valid_indices) == 0:
                    continue
                    
                chunk_pcd = pcd.select_by_index(valid_indices)
                print(f"    - Chunk {chunk_idx + 1}/{num_chunks} ({start_dist:.0f}m-{end_dist:.0f}m): {len(chunk_pcd.points)} points")
                
                # 对这个小块进行特征提取
                sub_extracted, sub_centers = coarse_extract_poles(
                    chunk_pcd, 
                    grid_res=GRID_RES, layer_res=LAYER_RES, N_thresh=N_THRESH, 
                    dz_thresh=DZ_THRESH, n_layers_thresh=N_LAYERS_THRESH,
                    max_area_cells=MAX_AREA_CELLS,
                    debug_prefix=f"{file_name}_chunk{chunk_idx}"
                )
                
                if sub_extracted is not None and len(sub_extracted.points) > 0:
                    extracted_sub_pcds.append(sub_extracted)
                    all_coarse_landmarks.extend(sub_centers)
            
            # 合并当前整张地图所有的候选提取结果
            if extracted_sub_pcds:
                extracted_pcd = extracted_sub_pcds[0]
                for sp in extracted_sub_pcds[1:]:
                    extracted_pcd += sp
            else:
                extracted_pcd = None

        else:
            # 如果没有轨迹文件，只能 fallback 到全图提取 (极大概率OOM)
            extracted_pcd, coarse_centers = coarse_extract_poles(
                pcd, 
                grid_res=GRID_RES, 
                layer_res=LAYER_RES, 
                N_thresh=N_THRESH, 
                dz_thresh=DZ_THRESH, 
                n_layers_thresh=N_LAYERS_THRESH,
                max_area_cells=MAX_AREA_CELLS,
                debug_prefix=f"{file_name}_full"
            )
            if coarse_centers:
                all_coarse_landmarks.extend(coarse_centers)
        
        t_coarse = time.time() - t0
        print(f"  [Time] Coarse extraction: {t_coarse:.3f}s")
        
        # 2. 保存
        if output_dir and extracted_pcd is not None:
            print(f"  -> Extracted poles containing {len(extracted_pcd.points)} points.")
            out_path = os.path.join(output_dir, file_name)
            
            # 使用包含 intensity 的方法保存
            save_pcd_with_intensity(out_path, extracted_pcd)
            
            # 累加到全局点云
            combined_pcd += extracted_pcd
            
            # --- 2.5 提取稳定单点坐标 (X, Y, Z) 用于 SLAM 地标匹配 ---
            # 设置 RANSAC 聚类提取，添加总点数过滤
            # z高程锚定在 current_poses
            t0 = time.time()
            pole_centers, refined_pcd = extract_pole_centers(
                extracted_pcd, poses=current_poses, eps=EPS, min_points=30,
                max_radius_thresh=MAX_RADIUS, max_span_thresh=MAX_SPAN,
                min_total_points=MIN_TOTAL_POINTS, max_total_points=MAX_TOTAL_POINTS
            )
            t_cluster = time.time() - t0
            print(f"  [Time] Clustering: {t_cluster:.3f}s")
            
            if pole_centers:
                print(f"  -> Clustered {len(pole_centers)} stable pole landmarks.")
                all_land_marks.extend(pole_centers)
                if refined_pcd is not None:
                    refined_combined_pcd += refined_pcd
                
        else:
            print(f"  -> Extracted 0 points.")
            
        t_map = time.time() - map_start_time
        print(f"  [Time] Total time for map {i}: {t_map:.3f}s\n")
            
    # 保存所有局部地图组合结果
    if output_dir and len(combined_pcd.points) > 0:
        combined_out_path = os.path.join(output_dir, "all_poles_combined.pcd")
        print(f"\nSaved combined poles point cloud: {combined_out_path} ({len(combined_pcd.points)} points)")
        save_pcd_with_intensity(combined_out_path, combined_pcd)
        
        # 保存提纯后与 TXT 完全一一对应的点云
        if len(refined_combined_pcd.points) > 0:
            refined_out_path = os.path.join(output_dir, "all_poles_refined.pcd")
            print(f"Saved REFINED strict poles point cloud: {refined_out_path} ({len(refined_combined_pcd.points)} points)")
            save_pcd_with_intensity(refined_out_path, refined_combined_pcd)
            
    # 保存所有粗提取的地标单点坐标到 TXT
    if output_dir and len(all_coarse_landmarks) > 0:
        coarse_array = np.array(all_coarse_landmarks)
        coarse_out = os.path.join(output_dir, "all_coarse_landmarks.txt")
        np.savetxt(coarse_out, coarse_array, fmt="%.6f %.6f %.6f %d", 
                   header="X Y Z TotalPoints", comments="")
        print(f"Saved {len(all_coarse_landmarks)} COARSE landmarks to {coarse_out}")
        
    # 保存所有地标单点坐标到 TXT
    if output_dir and len(all_land_marks) > 0:
        landmarks_array = np.array(all_land_marks)
        
        # 1. 统一计算距离并对局部坐标系进行从小到大排序
        dists = np.linalg.norm(landmarks_array[:, :2], axis=1)
        sorted_indices = np.argsort(dists)
        sorted_landmarks_array = landmarks_array[sorted_indices]
        
        # 2. 保存排序后的局部坐标文件
        landmarks_out = os.path.join(output_dir, "all_landmarks.txt")
        np.savetxt(landmarks_out, sorted_landmarks_array[:, :3], fmt="%.6f %.6f %.6f", 
                   header="X Y Z", comments="")
        print(f"Saved {len(all_land_marks)} stable landmarks (sorted by dist) to {landmarks_out}")

        # 3. 如果开启了 GPS 逆投影输出，则直接对排好序的局部坐标逐行转换
        if OUTPUT_GPS:
            gps_landmarks = local_to_gps(sorted_landmarks_array[:, :3], LAT0, LON0, ALT0)
            if gps_landmarks is not None:
                # 并排存放： 经度, 纬度, 海拔
                gps_expanded = np.zeros((len(gps_landmarks), 3))
                gps_expanded[:, 0] = gps_landmarks[:, 1]  # Longitude
                gps_expanded[:, 1] = gps_landmarks[:, 0]  # Latitude
                gps_expanded[:, 2] = gps_landmarks[:, 2]  # Altitude
                
                gps_out = os.path.join(output_dir, "all_landmarks_gps.txt")
                np.savetxt(gps_out, gps_expanded, fmt="%.8f %.8f %.3f", 
                           header="Longitude Latitude Altitude", comments="")
                print(f"Saved {len(gps_landmarks)} GPS landmarks (sorted by dist) to {gps_out}")
                
                # 保存包含 GeoSOT 23级编码的文件
                geosot_out = os.path.join(output_dir, "all_landmarks_geosot.txt")
                with open(geosot_out, 'w') as f:
                    f.write("GeoSOT_Code Longitude Latitude Altitude\n")
                    for i in range(len(gps_landmarks)):
                        lon = gps_expanded[i, 0]
                        lat = gps_expanded[i, 1]
                        alt = gps_expanded[i, 2]
                        # 对于 GeoSOT, level 为 23
                        code = GeoSOT.encode(lat, lon, GEOSOT_LEVEL)
                        f.write(f"{code} {lon:.8f} {lat:.8f} {alt:.3f}\n")
                print(f"Saved {len(gps_landmarks)} GeoSOT landmarks to {geosot_out}")

if __name__ == "__main__":
    # 测试目录是否存在
    if not os.path.exists(INPUT_DIR):
        print(f"Please specify a valid input directory. (Current: {INPUT_DIR})")
    else:
        total_start = time.time()
        process_directory(INPUT_DIR, OUTPUT_DIR, START_MAP_IDX, END_MAP_IDX)
        total_time = time.time() - total_start
        print(f"\n[Time] Processed all maps in: {total_time:.3f}s")
