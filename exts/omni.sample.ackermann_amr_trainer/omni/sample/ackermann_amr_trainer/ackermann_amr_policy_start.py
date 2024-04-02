# Copyright (c) 2020-2024, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from torch import nn

from torchvision import transforms
from torchvision.models import resnet18, ResNet18_Weights


class ackermann_amr_policy(nn.Module):
    def __init__(self):
        super().__init__()
        # Initialize Base Model
        full_model = resnet18(weights=ResNet18_Weights.DEFAULT)

        # Configure Layer Connections
        self.model = nn.Sequential(*list(full_model.children())[:-1])
        self.fc = nn.Linear(512, 2)

        # Node Activation Function
        self.sigma = nn.Tanh()

        # 2.3.2 Image Processing Transform

    # Forward Propogation
    def forward(self, x):
        x = self.transform(x)
        x = self.model(x)
        x = x.view(x.size(0),-1)
        x = self.sigma(self.fc(x))
        return x
