import os.path

from .mono_dataset import MonoDataset

import PIL.Image
import numpy as np
import random

from torchvision import transforms
import torch

import cv2


def mask_loader(path):
    # open path as file to avoid ResourceWarning
    # (https://github.com/python-pillow/Pillow/issues/835)
    with open(path, 'rb') as f:
        with PIL.Image.open(f) as mask:
            return mask


class AutowiseDataset(MonoDataset):

    def __init__(self, *args, **kwargs):
        super(AutowiseDataset, self).__init__(*args, **kwargs)

        self.K = np.array([[0.440789, 0, 0.510801, 0],
                           [0, 1.64056, 0.280167, 0],
                           [0, 0, 1.0, 0],
                           [0, 0, 0, 1]], dtype=np.float32)
        # self.translation_dir = "None"
        self.translation_dir = "/home/zhouchuqian/data/autowise/lingang_map5/poses/gps/translation_gps.txt"

        # self.mask_dir = "None"
        self.mask_dir = "/home/zhouchuqian/data/autowise/lingang_map5/mask/cam00/mask_for_training/"

        self.filenames_dict = {}
        self.mask_dict = {}
        for line in self.filenames:
            line = line.split()
            index = int(line[0])
            filename = line[1]
            self.filenames_dict[index] = filename

            if self.mask_dir != "None":
                filename = os.path.splitext(filename)[0] + ".tiff"
                self.mask_dict[index] = filename

        if self.translation_dir != "None":
            with open(self.translation_dir, 'r') as f:
                translation_filenames = f.read().splitlines()
            self.translation_dict = {}
            index = 0
            for line in translation_filenames:
                line = line.split()
                translation = float(line[1])
                self.translation_dict[index] = translation
                index += 1

    def __getitem__(self, index):
        inputs = {}

        do_color_aug = self.is_train and random.random() > 0.5
        do_flip = self.is_train and random.random() > 0.5

        line = self.filenames[index].split()
        frame_index = int(line[0])

        for i in self.frame_idxs:
            inputs[("color", i, -1)] = self.get_color(self.data_path, frame_index + i, None, do_flip)

        # adjusting intrinsics to match each scale in the pyramid
        for scale in range(self.num_scales):
            K = self.K.copy()

            K[0, :] *= self.width // (2 ** scale)
            K[1, :] *= self.height // (2 ** scale)

            inv_K = np.linalg.pinv(K)

            inputs[("K", scale)] = torch.from_numpy(K)
            inputs[("inv_K", scale)] = torch.from_numpy(inv_K)

        if do_color_aug:
            color_aug = transforms.ColorJitter.get_params(
                self.brightness, self.contrast, self.saturation, self.hue)
        else:
            color_aug = (lambda x: x)

        self.preprocess(inputs, color_aug)

        for i in self.frame_idxs:
            del inputs[("color", i, -1)]
            del inputs[("color_aug", i, -1)]

        if self.translation_dir != "None":
            translation = self.translation_dict[frame_index]
            inputs["translation_norm"] = torch.tensor(translation)

        return inputs

    def get_color(self, folder, frame_index, side, do_flip):
        if frame_index not in self.filenames_dict:
            if frame_index + 1 in self.filenames_dict:
                frame_index = frame_index + 1
            elif frame_index - 1 in self.filenames_dict:
                frame_index = frame_index - 1

        color = self.loader(folder + self.filenames_dict[frame_index])

        if self.mask_dict != "None":
            mask = cv2.imread(self.mask_dir + self.mask_dict[frame_index], cv2.IMREAD_GRAYSCALE)
            mask = mask == 0
            mask = np.array([mask, mask, mask])
            mask = np.transpose(mask, (1, 2, 0))

            color = color * mask
            color = PIL.Image.fromarray(color)

        if do_flip:
            color = color.transpose(PIL.Image.FLIP_LEFT_RIGHT)
        return color

    def check_depth(self):
        pass

    def get_depth(self, folder, frame_index, side, do_flip):
        pass
