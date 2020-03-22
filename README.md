

# Lidar Obstacle Detection Project

### Sensor Fusion Nanodegree 



### 1. Goal

The objective of this project is to detect obstacle on the road with PCD file.

Plus there is to make understand how to use PCL and how RANSAC and Euclidean Clustering are implemented.



### 2. Pipeline

1. Load PCD file which is from Lidar
2. Filtering PCD file with the these techniques below
   - DownSampling by using 'pcl::VoxelGrid'
   - Defined region which will be filtered actually by using 'pcl::CropBox'

3. Do Segmentation(separating road and obstacle) by implementing 'RANSAC algorithm'
4. Do Clustering obstacles by implementing 'k-d Tree' and  'Euclidean Clustering algorithm'
5. Make Bounding Box for each clusters.
6. Rendering all objects which was filtered.



### 3. Result

- #### **Before**

  ![](https://github.com/Din-2785/SFND_Lidar_Obstacle_Detection/blob/master/media/jhbak_before.gif?raw=true)

- #### **After**

  ![](https://github.com/Din-2785/SFND_Lidar_Obstacle_Detection/blob/master/media/jhbak_result.gif?raw=true)



### 4. Build

```
$> sudo apt install libpcl-dev
$> cd ~
$> git clone https://github.com/udacity/SFND_Lidar_Obstacle_Detection.git
$> cd SFND_Lidar_Obstacle_Detection
$> mkdir build && cd build
$> cmake ..
$> make
$> ./environment
```

