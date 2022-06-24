cd ../build
make -j4

cd ..

./bin/depth_to_points/one_frame \
	310.31 314.98 359.60 53.79 \
	/home/Autowise/data/autowise/lingang_map5/depth/depth_03/npy/ \
	/home/Autowise/data/autowise/lingang_map5/depth/depth_03/jpg/ \
	/home/Autowise/data/autowise/lingang_map5/depth/depth_03/delete/ \
