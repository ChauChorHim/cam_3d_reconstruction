cd ..

CUDA_VISIBLE_DEVICES="0" python -m inference_depth \
  --output_directory ~/data/autowise/lingang_map5/depth/depth_16/ \
	--data_path ~/data/autowise/lingang_map5/camera_images//cam00/cropped/ \
	--split lingang_map5 \
	--batch_size 4 \
	--model_path ~/log/16/models/weights_11
