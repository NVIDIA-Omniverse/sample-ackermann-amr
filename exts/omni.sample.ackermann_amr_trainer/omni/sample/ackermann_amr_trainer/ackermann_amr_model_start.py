# Copyright (c) 2020-2024, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from .ackermann_amr_policy import ackermann_amr_policy
from .ackermann_amr_data import ackermann_amr_data

import time
import carb
import numpy as np
import torch
import PIL.Image
import ctypes
import os

from torch import optim
from torch.utils.data import DataLoader
import torchvision.transforms as transforms


class ackermann_amr_model():

    def __init__(self, rows, columns, image_directory):
        self.device = torch.device('cuda')
        self.model = ackermann_amr_policy()
        self.model = self.model.to(self.device)
        self.save_directory = image_directory
        self.rows = rows
        self.columns = columns

        # Initialize Dataset
        self.dataset = ackermann_amr_data([image_directory],
                                          self.device,
                                          self.columns,
                                          self.rows)

    # Add Item to Dataset
    def add_item(self, click_y, click_x, file_path):
        return self.dataset.Add_Item(click_y,
                                     click_x,
                                     self.columns,
                                     self.rows,
                                     file_path)

    # Train Model
    def train(self):

        time.sleep(0.1)

        mse = torch.nn.MSELoss(reduction='mean')
        optimizer = optim.Adam(self.model.parameters(),
                               lr=1E-4,
                               weight_decay=1E-5)

        # 2.2.6 Load Training Data

        self.model.train()
        temp_losses = []
        for images, coords in loader:
            prediction = self.model(images)
            optimizer.zero_grad()
            loss = mse(prediction, coords)
            loss.backward()
            optimizer.step()
            temp_losses.append(loss.detach().item())
        print(np.mean(temp_losses))

    # Evaluate Model
    def Evaluate(self,
                 buffer,
                 buffer_size,
                 source_width,
                 source_height,
                 dest_width,
                 dest_height):
        try:
            ctypes.pythonapi.PyCapsule_GetPointer.restype = ctypes.POINTER(
                ctypes.c_byte * buffer_size)
            ctypes.pythonapi.PyCapsule_GetPointer.argtypes = [
                                                        ctypes.py_object,
                                                        ctypes.c_char_p]
            content = ctypes.pythonapi.PyCapsule_GetPointer(buffer, None)
        except Exception as e:
            carb.log_error(f"Failed to capture viewport: {e}")
            return

        # Evaluate Model
        self.model.eval()

        # 2.2.7 Load the viewport capture from buffer and add it to the GPU
        img = PIL.Image.frombuffer('RGBA',
                                   (source_width, source_height),
                                   content.contents)
        tensor_image = transforms.functional.to_tensor(img)
        image = tensor_image[:3, ...].to(self.device)

        mean = torch.Tensor([0.485, 0.456, 0.406]).cuda()
        std = torch.Tensor([0.229, 0.224, 0.225]).cuda()
        image.sub_(mean[:, None, None]).div_(std[:, None, None])
        img = image[None, ...]

        prediction = self.model(img)
        rows = dest_height
        cols = dest_width
        self.prediction_y, self.prediction_x = self.dataset.xy2rowcol(
            *prediction.squeeze().cpu(),
            num_rows=rows, num_cols=cols)

        carb.log_info(
            "prediction made: %.1f %.1f" % (
                self.prediction_x, self.prediction_y))

        return [self.prediction_y, self.prediction_x]

    # Load Model
    def load(self, model_name):
        model_path = os.path.join(self.save_directory,
                                  model_name)
        self.model.load_state_dict(torch.load(model_path))

    # Save Model
    def save(self, model_name):
        model_path = os.path.join(self.save_directory,
                                  model_name)
        torch.save(self.model.state_dict(), model_path)
