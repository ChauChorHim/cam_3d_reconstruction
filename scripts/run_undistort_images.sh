cd ../../build
make

cd ../examples/bin
./undistort_images \
	/home/Autowise/data/autowise/lingang_map5/raw/cam00.yaml \
	/home/Autowise/data/autowise/lingang_map5/camera_images/cam00/raw \
	/home/Autowise/data/autowise/lingang_map5/camera_images/cam00/undistorted \

