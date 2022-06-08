import argparse
import os

os.environ["MKL_NUM_THREADS"] = "1"  # noqa F402
os.environ["NUMEXPR_NUM_THREADS"] = "1"  # noqa F402
os.environ["OMP_NUM_THREADS"] = "1"  # noqa F402

import numpy as np

import torch
import torch.nn.functional as F
from torch.utils.data import DataLoader

from utils import readlines
from layers import transformation_from_parameters, disp_to_depth

import datasets
import networks

import matplotlib as mpl
import matplotlib.cm as cm

import PIL.Image as pil

import time


class Predictor:
    def __init__(self, options):
        self.opt = options

        self.models = {}
        self.device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")

        self.frames_to_load = [0, -1]

        # MODEL SETUP
        # Loading pretrained model
        print("   Loading pretrained encoder")
        encoder_dict = torch.load(os.path.join(self.opt.model_path, "encoder.pth"), map_location=self.device)

        self.width = encoder_dict['width']
        self.height = encoder_dict['height']

        self.models["encoder"] = networks.ResnetEncoder(18, "pretrained")

        filtered_dict_enc = {k: v for k, v in encoder_dict.items() if k in self.models["encoder"].state_dict()}
        self.models["encoder"].load_state_dict(filtered_dict_enc)

        print("   Loading pretrained decoder")
        depth_dict = torch.load(os.path.join(self.opt.model_path, "depth.pth"), map_location=self.device)

        self.models["depth_decoder"] = networks.DepthDecoder(num_ch_enc=self.models["encoder"].num_ch_enc,
                                                             scales=range(4))

        self.models["depth_decoder"].load_state_dict(depth_dict)

        print("   Loading pose network")
        pose_enc_dict = torch.load(os.path.join(self.opt.model_path, "pose_encoder.pth"), map_location=self.device)
        pose_dec_dict = torch.load(os.path.join(self.opt.model_path, "pose.pth"), map_location=self.device)

        self.models["pose_enc"] = networks.ResnetEncoder(18, False, num_input_images=2)
        self.models["pose_dec"] = networks.PoseDecoder(self.models["pose_enc"].num_ch_enc, num_input_features=1,
                                                       num_frames_to_predict_for=2)

        self.models["pose_enc"].load_state_dict(pose_enc_dict, strict=True)
        self.models["pose_dec"].load_state_dict(pose_dec_dict, strict=True)

        # Setting states of networks
        self.models["encoder"].eval()
        self.models["depth_decoder"].eval()
        self.models["pose_enc"].eval()
        self.models["pose_dec"].eval()
        if torch.cuda.is_available():
            self.models["encoder"].cuda()
            self.models["depth_decoder"].cuda()
            self.models["pose_enc"].cuda()
            self.models["pose_dec"].cuda()

        fpath = os.path.join("splits", self.opt.split, "{}_files.txt")
        self.test_filenames = readlines(fpath.format("image"))

        self.dataset = datasets.AutowiseDataset(self.opt.data_path, self.test_filenames,
                                                self.height, self.width,
                                                self.frames_to_load, 4, is_train=False, img_ext=".jpg")

        self.test_loader = DataLoader(
            self.dataset, self.opt.batch_size, False,
            num_workers=self.opt.batch_size, pin_memory=True, drop_last=True)

        self.test_iter = iter(self.test_loader)

        self.pose = []

        print("There are {:d} inference items\n".format(len(self.dataset)))

    def set_eval(self):
        """Convert all models to testing/evaluation mode
        """
        for m in self.models.values():
            m.eval()

    def predict(self):
        print(" --- Inference --- ")
        self.set_eval()
        for batch_idx, inputs in enumerate(self.test_loader):
            keyframe = inputs[("color", 0, 0)].detach()

            outputs = self.process_batch(inputs)
            translation = outputs['translation'].detach().cpu().numpy()
            axisangle = outputs['axisangle'].detach().cpu().numpy()
            q = get_quaternion_from_euler(axisangle[0], axisangle[1], axisangle[2])
            self.pose.append(str(batch_idx)
                             + " " + str(translation[0]) + " " + str(translation[1]) + " " + str(translation[2])
                             + " " + str(q[0]) + " " + str(q[1]) + " " + str(q[2]) + " " + str(q[3]))

            sigmoid_output = outputs[("disp", 0)]
            sigmoid_output_resized = torch.nn.functional.interpolate(
                sigmoid_output, (self.height, self.width), mode="bilinear", align_corners=False)
            sigmoid_output_resized = sigmoid_output_resized.detach().cpu().numpy()

            _, depth = disp_to_depth(sigmoid_output_resized, 0.1, 100)

            for i in range(self.opt.batch_size):
                index = batch_idx * self.opt.batch_size + i
                if index % 100 == 0:
                    print(index, "/", len(self.dataset))
                save_dir = self.opt.output_directory + format(index, '06d')
                toplot = np.squeeze(sigmoid_output_resized[i, :])
                normalizer = mpl.colors.Normalize(vmin=toplot.min(), vmax=np.percentile(toplot, 95))
                mapper = cm.ScalarMappable(norm=normalizer, cmap='magma')
                colormapped_im = (mapper.to_rgba(toplot)[:, :, :3] * 255).astype(np.uint8)
                im = pil.fromarray(colormapped_im)
                im.save(save_dir + ".png")

                one_depth = np.squeeze(depth[i, :])
                np.save(save_dir + ".npy", one_depth)

                cur_keyframe = keyframe[i, :].cpu().numpy() * 255
                cur_keyframe = np.rollaxis(cur_keyframe, 0, 3)
                input_im = pil.fromarray(np.uint8(cur_keyframe))
                input_im.save(save_dir + ".jpg")

        with open("./poses_pred.txt", "w") as pose_file:
            for i in range(len(self.pose)):
                print("{}".format(self.pose[i]), file=pose_file)

        print(len(self.dataset), "/", len(self.dataset))
        print(" --- Inference complete --- ")

    def process_batch(self, inputs):
        """Pass a minibatch through the network and generate images and losses
        """
        for key, ipt in inputs.items():
            inputs[key] = ipt.to(self.device)

        with torch.no_grad():
            features = self.models["encoder"](inputs["color_aug", 0, 0])
            outputs = self.models["depth_decoder"](features)
            outputs.update(self.predict_poses(inputs))

        return outputs

    def predict_poses(self, inputs, features=None):
        """Predict poses between input frames for monocular sequences.
        """
        outputs = {}

        # now we need poses for matching - compute without gradients
        pose_feats = {-1: inputs["color_aug", -1, 0],
                      0: inputs["color_aug", 0, 0]}
        with torch.no_grad():
            pose_inputs = [pose_feats[-1], pose_feats[0]]
            pose_inputs = [self.models["pose_enc"](torch.cat(pose_inputs, 1))]
            axisangle, translation = self.models["pose_dec"](pose_inputs)
            pose = transformation_from_parameters(
                axisangle[:, 0], translation[:, 0], invert=True)

            # set missing images to 0 pose
            for batch_idx, feat in enumerate(pose_feats[-1]):
                if feat.sum() == 0:
                    pose[batch_idx] *= 0

            # inputs[('relative_pose', -1)] = pose
            outputs[('relative_pose', -1)] = pose
            outputs['translation'] = torch.squeeze(translation[:, 0])
            outputs['axisangle'] = torch.squeeze(axisangle[:, 0])

        return outputs

def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.

    Input
      :param roll: The roll (rotation around x-axis) angle in radians.
      :param pitch: The pitch (rotation around y-axis) angle in radians.
      :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
      :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)

    return np.array([qx, qy, qz, qw])

def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument('--output_directory',
                        type=str,
                        help='path to output depth image',
                        required=True)

    parser.add_argument('--data_path',
                        type=str,
                        help='path to input rgb image',
                        required=True)

    parser.add_argument('--model_path',
                        type=str,
                        help='path to a folder of weighted to load',
                        required=True)

    parser.add_argument('--split',
                        type=str,
                        help='filenames list',
                        required=True)

    parser.add_argument('--batch_size',
                        type=int,
                        help='batch size',
                        default=16)

    return parser.parse_args()


if __name__ == '__main__':
    options = parse_args()
    predictor = Predictor(options)

    before_op_time = time.time()
    predictor.predict()
    duration = time.time() - before_op_time

    print("Takes time: ", duration / 60.0, " min")
