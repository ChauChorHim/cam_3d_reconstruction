# Copyright Niantic 2019. Patent Pending. All rights reserved.
#
# This software is licensed under the terms of the Monodepth2 licence
# which allows for non-commercial use only, the full terms of which are made
# available in the LICENSE file.

from __future__ import absolute_import, division, print_function

from trainer import Trainer
from options import MonodepthOptions

import os
import glob

options = MonodepthOptions()
opts = options.parse()


if __name__ == "__main__":
    # Backup code
    save_folder = os.path.join(opts.log_dir, opts.model_name, "code_backup")
    if not os.path.exists(save_folder):
        os.makedirs(save_folder)
    files = glob.iglob("./*")
    exclude_file = [opts.log_dir]
    for file in files:
        if file not in exclude_file:
            os.system('cp -r ' + str(file) + " " + str(save_folder))

    trainer = Trainer(opts)
    trainer.train()
