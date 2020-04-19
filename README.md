# Self-Driving Car Lidar Object detection

<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />


## Repo for the Lidar object detection project from the Sensor Fusion course for self-driving cars.

### Instructions for running

The main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./environment


### Results

#### Clustering and bounding box prediction on streamed point cloud data
<img src="media/lidar.gif" width="700" height="400" />

#### Lidar data
<img src="media/lidar-data.png" width="700" height="400" />

#### RANSAC Ground plane segmentation
<img src="media/ground-plane-segmentation.png" width="700" height="400" />

#### RANSAC Ground plane segmentation and clustering
<img src="media/ground-plane-segmentation-clustering.png" width="700" height="400" />

#### Voxel grid filtering
<img src="media/Voxel-grid.png" width="700" height="400" />

#### Euclidian Clustering and bounding box prediction
<img src="media/bb-prediction.png" width="700" height="400" />

