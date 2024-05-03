#!/usr/bin/env python3

# Copyright (c) 2020-2024, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Image
import numpy as np

from ackermann_amr_policy import ackermann_amr_policy

import PIL.Image

import torch
import torchvision
import torchvision.transforms as transforms
import os
import math


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        self.subscription = self.create_subscription(
            Image,
            'rgb',
            self.image_callback,
            1)
        self.subscription  # prevent unused variable warning

        self.publisher = self.create_publisher(AckermannDriveStamped, 'ackermann_cmd', 10)

        # Initialize AI Model
        self.device = torch.device('cuda')
        self.model = ackermann_amr_policy()
        self.model = self.model.to(self.device)

        self.save_dir = os.path.dirname(os.path.realpath(__file__))
        self.model_file = "road_following_model.pth"
        model_path = os.path.join(self.save_dir, self.model_file)
        self.model.load_state_dict(torch.load(model_path))

    def image_callback(self, msg):

        width = msg.width
        height = msg.height

        # Evaluate Model
        self.model.eval()
        img = PIL.Image.frombuffer('RGB', (width, height), msg.data.tobytes())
        tensor_image = transforms.functional.to_tensor(img)
        image = tensor_image[:3, ...].to(self.device)

        mean = torch.Tensor([0.485, 0.456, 0.406]).cuda()
        std = torch.Tensor([0.229, 0.224, 0.225]).cuda()
        image.sub_(mean[:, None, None]).div_(std[:, None, None])
        img = image[None, ...]

        prediction = self.model(img)

        prediction_x, prediction_y = self.xy2rowcol(*prediction.squeeze().cpu())
        
        # 3.2. Compute Steering Angle
        forward = np.array([0.0, -1.0])

        offset = np.array([0.0, 1.0])

        traj = np.array([prediction_x.item(), prediction_y.item()]) - offset

        unit_traj = traj / np.linalg.norm(traj)
        unit_forward = forward / np.linalg.norm(forward)
        steering_angle = np.arccos(np.dot(unit_traj, unit_forward)) / 2.0 - np.deg2rad(15.0)

        # 3.3. Create and Send Message
        msg = AckermannDriveStamped()

        msg.header.frame_id = "f1_tenth"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle =  steering_angle

        # Challenge: Increase the drive speed as much as you can!
        msg.drive.speed = 2.0

        self.publisher.publish(msg)

    def xy2rowcol(self, x, y):
        return x, y


def main(args=None):
    rclpy.init(args=args)

    evaluate_node = MinimalSubscriber()

    try:

        rclpy.spin(evaluate_node)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
    except KeyboardInterrupt:
        evaluate_node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
