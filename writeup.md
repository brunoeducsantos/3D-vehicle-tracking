# Results report 

## Match 3D Objects

The algorithm performing match of bounding boxes is in $matchBoundingBoxes$ ($camera_fusion.cpp$).
The implementation is divided into three seteps: 
1. Store in a pair of values box ids which are in previous and current frame bounding boxes
2. Evaluate the number of pair points per bounding box match between current and previous frame
3. Find the highest number of points per bounding box in prev and current frame above a certain threshold, choosing only the max counting bounding box per object detected 

![match]()

## Compute Lidar-based TTC



## Associate Keypoint Correspondences with Bounding Boxes



## Compute Camera-based TTC


## Performance Evaluation 1: Lidar TTC evaluation  



## Performance Evaluation 2: Lidar and Camera TTC difference evaluation 


