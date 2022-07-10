# Reconstrucion scene from images
This is a documentation for illustrating one pipeline of reconstruction scene from sequence images using monocular depth estimation techniques. This doc also propose some potential issues about this pipeline and prospect on this topic.

A naive definition of **Monocular depth estimation** is using model infer depth of each pixels in a image.

Having depth image, pixels on the image plane can be reprojected to the camera coordinates using camera intrinsic matrix. That's one frame pointcloud. We could also align pointclouds to get the scene pointcloud using camera extrinsic matrix.

## Capture Data
* Some monocular depth estimation [work](https://arxiv.org/abs/1904.04998) can obatin depth images given only a video sequence (estimate depth, camera pose, camera K at the same time). But these works only predict relative depth. In order to reduce the gap between a novel technique and a practical techique, this pipeline utilize camera intrinsic matrix and poses from other navigation system.
* The capture frequency of dataset train and test in this project is 10 Hz. You can also refer to popular dataset like KITTI or CItyScape.

## Monocular Depth Estimation Model
### Select Model
At first, we were amazed by a demo video from [MonoRec](https://github.com/Brummi/MonoRec) but found their inputs data are complex. So we decided to focus on another work published in 2021, [Manydepth](https://github.com/nianticlabs/manydepth). Then, we noticed this work was based on the previous popular work [Monodepth2](https://github.com/nianticlabs/monodepth2), which was also from this group. And after testing on our dataset, the performance on Manydepth were not better than Monodepth2 like the paper said. So, in the end, we chose Monodepth2 (model structure is lighter) as our monocular depth estimation model.

Besides Monodepth2, [PackNet](https://github.com/TRI-ML/packnet-sfm) from TRI performs good as well.

### Preprocess Data
* Undistort and crop captured images. Undistortion and cropping probably change the value of camera intrinsic matrix.
* Cropping sky has tiny influence on depth estimation, but reduce the usage of GPU memory while training.
* One assumption of monocular depth estimation is moving camera. People usually use splits excluded static camera samples (e.g. KITTI Eigen Zhou splits). For non-public dataset, using poses from navigation system or optical flow bwtween adjecent frames can get rid of the static frame.
* Another assumption is static scene. An intuitive way is using moving mask. In pratice, we use Mask-RCNN to instance segment images and filter out small instance. Then compute the optical flow value of each elected big instance and delete the instance with small optical flow value.
* Monodepth2 performs worse at the edge of images with distortion. 

### Train model
* As a self-supervised and monocular model, Monodepth2 will lose scale and only obtain relative depth. In this project, refer to velocity loss in PackNet, we find translation loss (use translation between two frames from navigation as a weak supervision) could solve scale issue without considering coordinates transformation.
* Data augmentation is important. But we can't use crop, resize, transform images, such methods will change the value of camera intrinsic matrix.
* Replacing PoseNet with poses from other navigation systems doesn't have significant influence on depth estimation.
* Dynamic objects on images not only will produce fault empty hole in depth images, but also violate consistency of depth estimation on ground floor. For example, when being overtaken by other car, depth estimation of lower area of depth images will be influenced by the abrupt object.

## Recover Pointcloud
### Prepare Data
* Monocular depth estimation includes a PoseNet predicting the pose between adjecent frames. You can use this pose if pose from other source is not provided.
* No matter using poses from PoseNet or other systems, poses while the car turing are usually not accurate enough to align the pointclouds To relieve this issue, consider lower the images capture frequency when the car is moving straight.

### Recover Single Frame Pointcloud
* The depth estimation model performs bad on image edges and far points. So these points should be removed.
* Now this pipeline only recover the rough pointcloud structure and miss most of detailed structure. So it is better to treat these detailed as outlier and filter them out.

### Align Multi Frame Pointcloud
* Before alignment, check the scale of single frame pointcloud.
* Use voxelFilter to remove the overlapping pointcloud.
* I have tried to use pose from navigation system as initial value and used ICP or NDT method to optimize the relative poses between single pointclouds. But the results are not satisfying.

### Evaluation metrics
It is common to use depth images from Lidar as ground truth to evaluate the predicted depth images. But in our dataset, we don't have this information. So basically we are just stuck at here.

However, we tried to reconstruct pointclouds from the depth images generated by some model perform pretty good in normal depth evaluation. And the results are not satisfying as well. 

Therefore, I think this current evaluation metrics for depth estimation is too hard and the meaning is not obvious. A more proper way is to evaluate the depth images by using them in a downstream task. And we evaluate this downstream task to judge the depth images. More discussion see next chapter.

## Problems and Prospect
This pipeline of reconstructing scene from images using monocular depth estimation (based on SfMLearner, like Monodepth2, MonoRec, PackNet, [Google's work](https://arxiv.org/abs/2010.16404)) can't get good enough depth images directly used in downstream tasks (such as reprojected to pointclouds) in limited time. Other depth estimation methods like [depth completion](https://arxiv.org/abs/2103.16690), can't obtain accurate enough depth images too.

Besided reconstructing scene pointcloud, depth images can be used in 3D detection, 3D segmentation et al. If these downstream tasks trust the depth images provided by monocular depth estimation module, large error would be introduced to the system.

The keypoint here is that, for downstream tasks, only accurate depth images are useful. And such depth images are usually obtained from 3D model using rendering techniques.

So, instead of using a intermediate product like depth image, why not modify this module to a multi-task module, like [Insta-DM](https://github.com/SeokjuLee/Insta-DM)? We can also treat monocular depth estimation as a sub-module and insert it into multi-modal multi-task model, like [BEVFusion](https://bevfusion.mit.edu/) merge Lidar information and image information to do a BEV segmentation task.
