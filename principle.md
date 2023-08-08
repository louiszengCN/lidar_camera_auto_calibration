### pcd select principle
#### 1.计算点云质心坐标
对于一个三维点云，假设有 N 个点，每个点的坐标为 (xᵢ, yᵢ, zᵢ)，其中 i 取值范围从 1 到 N，则点云的质心坐标 (X, Y, Z) 计算如下：
X = (x₁ + x₂ + ... + xₙ) / N
Y = (y₁ + y₂ + ... + yₙ) / N
Z = (z₁ + z₂ + ... + zₙ) / N
其中，X、Y、Z 分别是质心在 X、Y、Z 轴上的坐标。这个计算公式表示了将所有点的坐标分量分别相加，然后除以点的总数 N，从而得到质心坐标。
```python
point_cloud = o3d.io.read_point_cloud(file_path)
centroid = np.mean(np.asarray(point_cloud.points), axis=0)
```
#### 2.通过限制点云到质心的距离，筛选出质心附近点云

计算点云中每个点到质心的距离，可以使用欧氏距离（Euclidean Distance）公式。假设点云中有一个点 P(x, y, z)，质心为 C(X, Y, Z)，则点 P 到质心 C 的欧氏距离计算如下：
Distance = √((x - X)² + (y - Y)² + (z - Z)²)
这是三维空间中两点之间的欧氏距离公式，其中 (x, y, z) 是点 P 的坐标，(X, Y, Z) 是质心 C 的坐标。
```PYTHON
distances = np.linalg.norm(np.asarray(point_cloud.points) - centroid, axis=1)
filtered_point_cloud_by_centroid = point_cloud.select_by_index(np.where(distances <= 2)[0])
```
#### 3.对质心附近点云采用DBSCAN聚类
>*DBSCAN（Density-Based Spatial Clustering of Applications with Noise）是一种基于密度的空间聚类算法，用于将数据点分成不同的簇，并能够识别噪声点。与传统的聚类算法（如 K-Means）相比，DBSCAN 不需要预先指定簇的数量，能够发现任意形状的簇，并且对异常值（噪声）具有鲁棒性。*
>
>DBSCAN 的基本原理如下：
>
> - 核心对象（Core Points）：一个数据点被称为核心对象，如果在它的邻域内（由一个指定的半径ε内）至少有一定数量（由一个指定的最小点数MinPts）的数据点。这意味着核心对象是比较密集的区域内的点。
> - 直接密度可达（Directly Density-Reachable）：一个数据点A被称为从核心对象B直接密度可达，如果A在以B为中心，半径ε的邻域内，并且B是核心对象。
> - 密度可达（Density-Reachable）：一个数据点A被称为从核心对象B密度可达，如果存在一串核心对象 C1, C2, ..., Cn，其中 C1 = B，Cn = A，且 Ci+1 从 Ci 直接密度可达。
> - 密度相连（Density-Connected）：两个数据点A和B被称为密度相连，如果存在一个核心对象C，使得A和B都从C密度可达。
>
>基于上述定义，DBSCAN 将数据点分为以下三类：
>
> - 核心对象：满足密度条件的点，即在其ε邻域内至少有MinPts个点。
> - 边界点：不满足密度条件，但至少在某个核心对象的ε邻域内。
> - 噪声点：既不是核心对象，也不是边界点。
>
>DBSCAN 的工作流程如下：
>1. 随机选择一个未访问的数据点。
>2. 如果该点是核心对象，则以它为种子展开一个新的簇，即找出密度可达的所有点。
>3. 如果该点不是核心对象，标记为噪声点。
>4. 对未访问的点重复上述过程，直到所有点都被访问。
>5. DBSCAN 算法的输出是一组簇，每个簇包含一些核心对象和边界点，以及一些可能的噪声点。

通过质心筛选后，留下的点云主要为地面、天花板、标定板，地面与天花板的点云数量较多，间距比较宽，可以通过设定DBSCAN中半径 ε 和最小点数 MinPts 这两个参数来把标定板点云筛选出来。
```python
#eps为半径，min_points为做小点数
labels_1 = np.array(filtered_point_cloud_by_centroid.cluster_dbscan(eps=eps_1, min_points=min_points_1))
keep_label_1 = k_l_1 
# 获取标签为keep_label的点的索引
keep_indices = np.where(labels_1 == keep_label_1)[0] 
# 保留除了keep_label以外的点云
filtered_point_cloud_by_DBSCAN = filtered_point_cloud_by_centroid.select_by_index(keep_indices)
```

