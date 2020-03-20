

# Lidar Obstacle Detection Project

### Sensor Fusion Nanodegree 



The objective of this project is to detect obstacle from PCD file which is from Lidar and to know how filtering, segmentation and Clustering are implemented.

This project is showing techniques below.

1. **Filtering** : 'Down Sampling' and 'Region user will handle with filter' is implemented with PCL

2. **Segmentation :** Road and Obstacle (car, poll, anything on road) are separated by implementing RANSAC_3d

3. **Clustering** : Through implementing 'K-d tree' and 'Euclidean Clustering' , cluster is defined



### Build

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

if on VisualStudio,

[PCL]: https://github.com/PointCloudLibrary/pcl/releases

 version is necessary on PC.

