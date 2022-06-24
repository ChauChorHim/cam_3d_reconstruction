cd ../build
make -j4

cd ..
./bin/prepare_dataset/extract_relative_poses \
	/home/Autowise/data/autowise/lingang_map5/raw/lingang_map5.csv \
	/home/Autowise/data/autowise/lingang_map5/splits/image_files.txt \
	/home/Autowise/data/autowise/lingang_map5/poses/gps/poses_cam.txt \