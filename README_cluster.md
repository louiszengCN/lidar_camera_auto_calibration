# lidar_camera_auto_calibration
A method to automatically calibrate lidar and camera

## principle
#### 1.Calculate the centroid of point clouds
For a three-dimensional point cloud with N points, where each point has coordinates (xᵢ, yᵢ, zᵢ), where i ranges from 1 to N, the centroid coordinates (X, Y, Z) of the point cloud are computed as follows:
>X = (x₁ + x₂ + ... + xₙ) / N
Y = (y₁ + y₂ + ... + yₙ) / N
Z = (z₁ + z₂ + ... + zₙ) / N

Where X, Y, and Z are the coordinates of the centroid along the X, Y, and Z axes, respectively. This calculation formula entails summing up the coordinate components of all points individually and then dividing by the total number of points, N, to obtain the centroid coordinates.
```python
point_cloud = o3d.io.read_point_cloud(file_path)
centroid = np.mean(np.asarray(point_cloud.points), axis=0)
```
#### 2.Filtering Point Cloud Near the Centroid by Limiting Distance

To compute the distance from each point in a point cloud to the centroid, you can use the Euclidean Distance formula. Let's consider a point P(x, y, z) in the point cloud and a centroid C(X, Y, Z). The Euclidean distance from point P to centroid C is calculated as follows:
>Distance = √((x - X)² + (y - Y)² + (z - Z)²)

This is the Euclidean distance formula between two points in three-dimensional space, where (x, y, z) represents the coordinates of point P, and (X, Y, Z) represents the coordinates of centroid C.
```PYTHON
distances = np.linalg.norm(np.asarray(point_cloud.points) - centroid, axis=1)
filtered_point_cloud_by_centroid = point_cloud.select_by_index(np.where(distances <= 2)[0])
```
#### 3.Applying DBSCAN Clustering to Points Near the Centroid
>*DBSCAN (Density-Based Spatial Clustering of Applications with Noise) is a density-based spatial clustering algorithm used for partitioning data points into different clusters and identifying noise points. Compared to traditional clustering algorithms like K-Means, DBSCAN does not require the pre-specification of the number of clusters, can discover clusters of arbitrary shapes, and is robust to outliers (noise).*
>
>##### The fundamental principles of DBSCAN are as follows:
>
> - **Core Points:** A data point is considered a core point if within its neighborhood (defined by a specified radius ε), there are at least a certain number (specified as MinPts) of data points. This implies that core points exist in denser regions.
> - **Directly Density-Reachable:** A data point A is directly density-reachable from a core point B if A is within the ε radius neighborhood of B, and B itself is a core point.
> - **Density-Reachable:** A data point A is density-reachable from a core point B if there exists a sequence of core points C1, C2, ..., Cn, where C1 = B and Cn = A, and each Ci+1 is directly density-reachable from Ci.
> - **Density-Connected:** Two data points A and B are density-connected if there exists a core point C such that both A and B are density-reachable from C.
>
>##### Based on the above definitions, DBSCAN categorizes data points into three types:
>
> - **Core Points:** Points that satisfy the density criteria, meaning they have at least MinPts points within their ε-neighborhood.
> - **Border Points:** Points that do not satisfy the density criteria themselves but are within the ε-neighborhood of a core point.
> - **Noise Points:** Neither core points nor border points.
>
>##### The workflow of the DBSCAN algorithm is as follows:
>1. Randomly select an unvisited data point.
>2. If the point is a core point, expand a new cluster from it by finding all density-reachable points.
>3. If the point is not a core point, mark it as a noise point.
>4. Repeat the above steps for unvisited points until all points are visited.

After filtering through centroid selection, the remaining point cloud mainly consists of ground, ceiling, and calibration board points. The ground and ceiling points dominate the point cloud with a relatively larger spacing. To extract the calibration board points, one can adjust the parameters of the DBSCAN algorithm, specifically the radius ε and the minimum point count MinPts.
```python
#eps is radius，min_points is the minimum point count
labels_1 = np.array(filtered_point_cloud_by_centroid.cluster_dbscan(eps=eps_1, min_points=min_points_1))
keep_label_1 = k_l_1 
# get the index of label "keep_label"
keep_indices = np.where(labels_1 == keep_label_1)[0] 
# Retain the point cloud except for the points with the label "keep_label".
filtered_point_cloud_by_DBSCAN = filtered_point_cloud_by_centroid.select_by_index(keep_indices)
```

