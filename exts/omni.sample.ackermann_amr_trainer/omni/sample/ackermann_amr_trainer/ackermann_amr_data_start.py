# Copyright (c) 2020-2024, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import numpy as np
import torch
import torchvision
from torch.utils.data import Dataset
import os
from pathlib import Path

class ackermann_amr_data(Dataset):
    # Initializing the dataset
    def __init__(self, root_paths, device, thumbnail_width, thumbnail_height):
        super().__init__()

        # Initialize variables
        self.device = device
        self._THUMB_ROWS = thumbnail_height
        self._THUMB_COLS = thumbnail_width
        self._raw_click = []
        self._raw_ratio = []
        self._raw_coords = []
        self._raw_images = []
        self._root_paths = root_paths

        # Iterate through all pre-existing images
        for i, path in enumerate(root_paths):

            files = os.listdir(path)
            for fname in files:
                # Only process .png files
                if fname.endswith(('.png')):

                    # Split out the x coord, y coord and UID of the image
                    stokens = fname.split("_")

                    # Only process images that have an x, y, and UID
                    if len(stokens) == 3:
                        # Read image from file
                        image = torchvision.io.read_image(
                            os.path.join(path, fname))
                        # Store raw image data for future use
                        self._raw_images.append(
                            (image[:-1]/255).float().unsqueeze(0))

                        # Get x coord
                        col = int(stokens[0])
                        # Get y coord
                        row = int(stokens[1])
                        # Convert from "click" coords to image coords
                        x, y = self.rowcol2xy(row,
                                              col,
                                              self._THUMB_ROWS,
                                              self._THUMB_COLS)

                        # Determine Aspect Ratio
                        col_ratio = float(col/self._THUMB_COLS)
                        row_ratio = float(row/self._THUMB_ROWS)

                        # Save Raw Annotation for later use
                        self._raw_click.append([row, col])
                        self._raw_ratio.append([row_ratio, col_ratio])
                        self._raw_coords.append([x, y])
                    else:
                        # Delete any images that are not properly annotated
                        fpath = os.path.join(path, fname)
                        if os.path.isfile(fpath):
                            os.remove(fpath)

        # Convert raw data to pytorch-compatible data
        self._click_tensor = torch.from_numpy(
            np.array(self._raw_click)).float().to(device)
        self._ratio_tensor = torch.from_numpy(
            np.array(self._raw_ratio)).float().to(device)
        self._coords_tensor = torch.from_numpy(
            np.array(self._raw_coords)).float().to(device)

        # Compute dataset length
        if len(self._raw_images) > 0:
            self._image_tensor = torch.cat(self._raw_images, dim=0).to(device)
            self._total_length = self._image_tensor.shape[0]
        else:
            self._total_length = 0

    # Adding an image to the dataset
    def Add_Item(self, row, col, width, height, file_path):
        # Read the image from file
        image = torchvision.io.read_image(file_path)

        # Save the image raw data
        self._raw_images.append((image[:-1]/255).float().unsqueeze(0))

        # Convert from `click` coordinates to image coordinates
        x, y = self.rowcol2xy(row, col, height, width)

        # Process and save click annotation
        col_ratio = float(col/width)
        row_ratio = float(row/height)
        self._raw_click.append([row, col])
        self._raw_ratio.append([row_ratio, col_ratio])
        self._raw_coords.append([x, y])

        # Convert data to pytorch-compatible values
        self._click_tensor = torch.from_numpy(
            np.array(self._raw_click)).float().to(self.device)
        self._ratio_tensor = torch.from_numpy(
            np.array(self._raw_ratio)).float().to(self.device)
        self._coords_tensor = torch.from_numpy(
            np.array(self._raw_coords)).float().to(self.device)
        self._image_tensor = torch.cat(self._raw_images, dim=0).to(self.device)
        self._total_length = self._image_tensor.shape[0]

        # 2.2.4 Prepend file name with x and y coordinates for later loading

    # Get a single image from the dataset
    def __getitem__(self, index):
        img = self._image_tensor[index]
        coords = self._coords_tensor[index]
        return img, coords

    # Find the number of images in the dataset
    def __len__(self):
        return self._total_length

    # Convert from image to annotation coordinates
    def rowcol2xy(self, row, col, num_rows=224, num_cols=224):
        x = 2*col/num_cols - 1
        y = 1 - 2*row/num_rows
        return float(x), float(y)

    # Convert from annotation to image coordinates
    def xy2rowcol(self, x, y, num_rows=224, num_cols=224):
        col = 0.5*num_cols*(1 + x)
        row = 0.5*num_rows*(1 - y)
        return int(row), int(col)
