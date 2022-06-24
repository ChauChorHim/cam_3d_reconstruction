cd ../build
make -j4

cd ..

./bin/postprocess_data/multi_frame \
	310.31 314.98 359.60 53.79 \
	/home/Autowise/data/autowise/lingang_map5/depth/depth_03/pcd_short/ \
	/home/Autowise/data/autowise/lingang_map5/depth/depth_03/scene/lingang_map5_0-199.pcd \
	/home/Autowise/data/autowise/lingang_map5/poses/opt/poses_cam_icp_3.txt \
	0 200 \

./bin/postprocess_data/multi_frame \
	310.31 314.98 359.60 53.79 \
	/home/Autowise/data/autowise/lingang_map5/depth/depth_03/pcd_short/ \
	/home/Autowise/data/autowise/lingang_map5/depth/depth_03/scene/lingang_map5_200-399.pcd \
	/home/Autowise/data/autowise/lingang_map5/poses/opt/poses_cam_icp_3.txt \
	200 200 \
