# Depth to Points

Here are some basic instruction for the tools reconstructing scene points using depth image, rgb image, intrinsics matrix and extrinsics matrix. 

## Pipeline for reconstructing single frame pointcloud
Read "examples/test_one_frame.cpp" for more details.

1. Read intrinsics parameters, path to depth npy file and corresponding rgb image (for dyeing the pointcloud).

    N.B. please make sure the datatype of npy file is "float"; 
    
    please check the shape of rgb image and depth file are the same.
2. Construct an object of PointCloudSaver.
3. Call the function 
   ``` c++
   PointCloudSaver::saveOnePointCloud(path_to_depth, path_to_image, path_to_pcd); 
   ```

## Pipeline for concatenating multi frame pointcloud
Read "examples/test_multi_frame.cpp" for more details.

1. Read intrinsics parameters, paths to all NavState exported csv files, path to the list of images, path to the list of depths.
2. Load and extract NavState gps raw data into a gps buffer.
3. Set the first gps data as initial pose.
4. Load image and depth and find their nearest pose in the pool of gps buffer.
5. Main loop
    * extract pointcloud in world coordinates for every frame (depth+image);
    * call the function below to concatenate current frame pointcloud with the scene pointcloud.

    ``` c++
    PointCloudSaver::addPointCloud(cur_path_to_depth, cur_path_to_image, cur_pos, cur_q);
    ```
6. Save the scene pointcloud.

## Environment package
* C++14
* Eigen 3.4.0
* PCL 1.10
* OpenCV 4.5.5

## Test data
* For single frame reconstruction, please check "examples/assets/*"
* For multi frame reconstruction, the following files are required:
    * NavState csv files (or any data source towards gps position and orientation)
    * list of depth files path
    * list of rgb images files 