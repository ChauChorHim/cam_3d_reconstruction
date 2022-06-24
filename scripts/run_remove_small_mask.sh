cd ../build
make -j4

cd ..
./bin/prepare_dataset/remove_small_mask \
    /home/Autowise/data/autowise/lingang_map5/mask/cam00/tiff/ \
    /home/Autowise/data/autowise/lingang_map5/mask/cam00/mask_for_training/ 
