import os
import open3d as o3d
import numpy as np   
# file_path = "/home/autel/lidar_data_processed/processed_data_08_01/pcd/1.pcd"
def save_pcd(file_path, selected_point_cloud):
    original_file_name = file_path.split('/')[-1]
    current_floder_path = os.path.dirname(os.path.abspath(__file__))
    save_path_fisheye = os.path.join(current_floder_path, '/data/pcd_fisheye', original_file_name)
    o3d.io.write_point_cloud(save_path_fisheye, selected_point_cloud)
    save_path_pinhole = os.path.join(current_floder_path, '/data/pcd_pinhole', original_file_name)
    o3d.io.write_point_cloud(save_path_pinhole, selected_point_cloud)
    print('saved')

def process_point_cloud(file_path, eps_1, min_points_1, k_l_1, eps_2, min_points_2, k_l_2, times):
    #print("Current file:", file_path)
    point_cloud = o3d.io.read_point_cloud(file_path)
    o3d.visualization.draw_geometries([point_cloud])
    centroid = np.mean(np.asarray(point_cloud.points), axis=0)
    distances = np.linalg.norm(np.asarray(point_cloud.points) - centroid, axis=1)
    filtered_point_cloud_by_centroid = point_cloud.select_by_index(np.where(distances <= 2)[0])
    # 每次裁切会导致索引变更，在裁切之后第一时间保留裁切后的点云在原点云中的索引
    cut_1_index = np.where(distances <= 2)[0]
    #o3d.visualization.draw_geometries([filtered_point_cloud_by_centroid])
    labels_1 = np.array(filtered_point_cloud_by_centroid.cluster_dbscan(eps=eps_1, min_points=min_points_1))
    keep_label_1 = k_l_1 
    # selected_indices_1 = np.where(labels_1 == keep_label_1)[0]
    # 获取标签为keep_label的点的索引
    keep_indices = np.where(labels_1 == keep_label_1)[0] ##########如果使用manual_1选出来的参数 这里要写成 == 如果是manual_2选出来的 应该是!=
    # 保留除了keep_label以外的点云
    filtered_point_cloud_by_DBSCAN = filtered_point_cloud_by_centroid.select_by_index(keep_indices)
    # 索引保留
    cut_2_index = keep_indices
    # o3d.visualization.draw_geometries([filtered_point_cloud_by_DBSCAN])
    # 判断需要几次裁切
    if(times == 1):
        # 如果只需要一次裁切 保存后进行可视化 随后退出
        save_pcd(file_path, filtered_point_cloud_by_DBSCAN)
        all_indices = np.arange(len(point_cloud.points))
        cut_indices = all_indices[cut_1_index[cut_2_index]]
        selected_point_cloud = point_cloud.select_by_index(cut_indices)
        colors = np.zeros((len(point_cloud.points), 3))
        colors[cut_indices] = [1, 0, 0]  # 红色
        colors[np.logical_not(np.isin(np.arange(len(point_cloud.points)), cut_indices))] = [0, 0, 1]  # 蓝色
        point_cloud.colors = o3d.utility.Vector3dVector(colors)
        o3d.visualization.draw_geometries([point_cloud])
        return
    # o3d.visualization.draw_geometries([filtered_point_cloud_by_DBSCAN])
    labels_2 = np.array(filtered_point_cloud_by_DBSCAN.cluster_dbscan(eps=eps_2, min_points=min_points_2))
    keep_label_2 = k_l_2 
    # selected_indices_1 = np.where(labels_1 == keep_label_1)[0]
    # 获取标签为keep_label的点的索引
    keep_indices_2 = np.where(labels_2 != keep_label_2)[0]
    # 保留除了keep_label以外的点云
    filtered_point_cloud_by_DBSCAN_2 = filtered_point_cloud_by_DBSCAN.select_by_index(keep_indices_2)
    # 索引保留
    cut_3_index = keep_indices_2
    # o3d.visualization.draw_geometries([filtered_point_cloud_by_DBSCAN_2])

    # 可视化过程 在原图中显示裁切点云
    all_indices = np.arange(len(point_cloud.points))
    cut_indices = all_indices[cut_1_index[cut_2_index[cut_3_index]]]
    selected_point_cloud = point_cloud.select_by_index(cut_indices)
    colors = np.zeros((len(point_cloud.points), 3))
    colors[cut_indices] = [1, 0, 0]  # 红色
    colors[np.logical_not(np.isin(np.arange(len(point_cloud.points)), cut_indices))] = [0, 0, 1]  # 蓝色
    point_cloud.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([point_cloud])
    save_pcd(file_path, selected_point_cloud)


def process_file(file_path):
    # 点云聚类参数及次数设置
    original_file_name = file_path.split('/')[-1]
    if(original_file_name == '1.pcd'):
        eps_1 = 0.4
        min_points_1 = 700
        k_l_1 = 0
        eps_2 = 0.8
        min_points_2 = 1000
        k_l_2 = 1
        times = 1
    elif(original_file_name == '4.pcd' ):
        eps_1 = 0.4
        min_points_1 = 800
        k_l_1 = 0
        eps_2 = 0.8
        min_points_2 = 500
        k_l_2 = 0
        times = 1

    elif(original_file_name == '5.pcd'):
        eps_1 = 0.6
        min_points_1 = 800
        k_l_1 = 1
        eps_2 = 0.8
        min_points_2 = 800
        k_l_2 = 0
        times = 1

    elif(original_file_name == '2.pcd'):
        eps_1 = 0.6
        min_points_1 = 800
        k_l_1 = 1
        eps_2 = 0.8
        min_points_2 = 1000
        k_l_2 = 1
        times = 1

    elif(original_file_name == '6.pcd'):
        eps_1 = 0.4
        min_points_1 = 700
        k_l_1 = 1
        eps_2 = 0.8
        min_points_2 = 1000
        k_l_2 = 1
        times = 1

    elif(original_file_name == '3.pcd'):
        eps_1 = 0.4
        min_points_1 = 700
        k_l_1 = 0
        eps_2 = 0.8
        min_points_2 = 1000
        k_l_2 = 0
        times = 1

    process_point_cloud(file_path, eps_1, min_points_1, k_l_1, eps_2, min_points_2, k_l_2, times)

def traverse_folder(folder_path):
    for folder_name, _, filenames in os.walk(folder_path):
        for filename in filenames:
            file_path = os.path.join(folder_name, filename)
            print(file_path)
            process_file(file_path)


if __name__ == "__main__":
    folder_path = "processed_data_08_03/pcd"  
    traverse_folder(folder_path)