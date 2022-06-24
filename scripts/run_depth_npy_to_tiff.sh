cd ../build
make -j4

cd ..
./bin/postprocess_data/depth_npy_to_tiff \
    /home/Autowise/data/autowise/lingang_map5/mask/cam00/npy/ \
	/home/Autowise/data/autowise/lingang_map5/mask/cam00/tiff/
