
# Copyright (c) 2020-2024, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from .ackermann_amr_model import ackermann_amr_model
import omni.ext
import omni.ui as ui
import carb.events
import os
import tempfile
import time
import omni.usd
from omni.kit.viewport.utility import get_active_viewport
from omni.kit.viewport.utility import capture_viewport_to_file
from omni.kit.viewport.utility import capture_viewport_to_buffer
import omni.kit.app
import uuid
from omni.ui import color as cl
import asyncio

class ackermann_trainer_window(ui.Window):
    def __init__(self, *args, **kwargs):
        super(ackermann_trainer_window, self).__init__(*args, **kwargs)
        self.file_path = ""
        self.click_x = 0
        self.click_y = 0
        self.prediction_x = 0
        self.prediction_y = 0
        self.thumbnail_height = 300
        self.thumbnail_width = 600

        # Configure Directory Where Data Will Be Saved
        self.save_dir = tempfile.gettempdir()
        self.save_dir = os.path.join(self.save_dir, 'road_following')
        if not os.path.exists(self.save_dir):
            os.mkdir(self.save_dir)

        self.save_dir = os.path.join(self.save_dir, 'apex')
        if not os.path.exists(self.save_dir):
            os.mkdir(self.save_dir)

        # Initialize AI Model
        self.model = ackermann_amr_model(self.thumbnail_height,
                                         self.thumbnail_width,
                                         self.save_dir)

        self.update_stream = omni.kit.app.get_app().get_update_event_stream()
        self.frame_update_count = 0

        self.ov_update = None

        self.build_ui()

    # Capture Image
    def onCapture(self):
        # Get Filename
        filename = '%s.png' % (str(uuid.uuid1()))
        self.file_path = os.path.join(self.save_dir, filename)

        # Request Image Capture Asynchronously
        asyncio.ensure_future(self.capture_image(self.replace_image,
                                                 self.file_path))

    # Capture the Viewport to File
    async def capture_image(self, on_complete_fn: callable, file_path) -> str:
        # 2.1.2 Capture the Viewport to File
        viewport = get_active_viewport()
        capture_viewport_to_file(viewport, file_path=file_path)
        carb.log_warn("Image Captured: {file_path}")

        # Wait for the Capture to Complete
        if on_complete_fn:
            await omni.kit.app.get_app().next_update_async()
            await omni.kit.app.get_app().next_update_async()
            # Update the User Interface
            on_complete_fn()

    # Thumbnail Clicked Callback
    def onMouseReleased(self, x, y, button, modifier, canvas):

        # 2.1.4 Capture Click Position
        self.click_x, self.click_y = canvas.screen_to_canvas(x, y)

        # 2.1.4 Add Image to Dataset
        self.file_path = self.model.add_item(self.click_y,
                                             self.click_x,
                                             self.file_path)

        carb.log_warn("Image Annotated: {self.file_path}")

        # Update Image Annotation
        self.replace_image()

        # Update Count
        self.count_model.set_value(str(len(self.model.dataset)))

    # Replace the Thumbnail with the latest image
    def replace_image(self):

        counter = 0
        while not os.path.exists(self.file_path) and counter < 10:
            time.sleep(0.1)
            counter += 1

        with self.ClickableCanvas:
            with ui.ZStack():
                # 2.1.3 Update the image from the capture
                ui.Image(self.file_path)
                with ui.VStack():
                    # 2.1.5 Set Annotation Y-Position
                    ui.Spacer(height=self.click_y)
                    with ui.HStack():
                        # 2.1.5 Set Annotation Z-Position
                        ui.Spacer(width=self.click_x)
                        style = {"Circle": {"background_color": cl("#cc0000"),
                                            "border_color": cl("#cc0000"),
                                            "border_width": 2}}
                        ui.Circle(width=10,
                                  height=10,
                                  alignment=ui.
                                  Alignment.LEFT_TOP,
                                  style=style)
                # Add Prediction Dot
                with ui.VStack():
                    ui.Spacer(height=self.prediction_y)
                    with ui.HStack():
                        ui.Spacer(width=self.prediction_x)
                        style = {"Circle": {"background_color": cl("#00cc00"),
                                            "border_color": cl("#00cc00"),
                                            "border_width": 2}}
                        ui.Circle(width=10,
                                  height=10,
                                  alignment=ui.Alignment.LEFT_TOP,
                                  style=style)

    # Train the AI Model
    def train(self):

        self.train_button.enabled = False

        # 2.1.6 Train AI Model
        self.model.train()

        self.train_button.enabled = True
        carb.log_info("Training Complete")

    # Turn Evaluation On and Off
    def toggle_eval(self, model):
        # Toggle Evaluation On and Off
        if self.eval_model.get_value_as_bool():
            self.ov_update = self.update_stream.create_subscription_to_pop(
                self.on_update,
                name="Eval_Subscription")
        else:
            self.ov_update.unsubscribe()

    # Omniverse Update Callback
    def on_update(self, e: carb.events.IEvent):
        # Capture the Viewport Every 30 Frames
        self.frame_update_count += 1

        if self.frame_update_count % 30 == 0:
            self.frame_update_count = 1
            # 2.1.7 Capture the Viewport to Buffer
            viewport_api = get_active_viewport()
            capture_viewport_to_buffer(viewport_api, self.on_viewport_captured)

    # Evaluate the Viewport with the AI Model
    def on_viewport_captured(self, buffer, buffer_size, width, height, format):

        # 2.1.7 Evaluate Viewport Image
        self.prediction_y, self.prediction_x = self.model.Evaluate(buffer,
                                                                   buffer_size,
                                                                   width,
                                                                   height,
                                                                   self.thumbnail_width,
                                                                   self.thumbnail_height)

        self.replace_image()

    # 2.1.8 Load Model
    def onLoadModel(self):
        self.model.load(self.model_path_model.as_string)

    # 2.1.8 Save Model
    def onSaveModel(self):
        self.model.save(self.model_path_model.as_string)

    # Build the UI
    def build_ui(self):
        # Build UI
        with self.frame:
            with ui.ScrollingFrame():
                with ui.VStack():
                    ui.Spacer(height=40)
                    # Capture Image
                    with ui.HStack(height=self.thumbnail_height):
                        with ui.HStack():
                            ui.Spacer()
                            self.ClickableCanvas = ui.CanvasFrame(
                                draggable=False,
                                width=self.thumbnail_width,
                                height=self.thumbnail_height)
                            self.ClickableCanvas.set_mouse_released_fn(
                                lambda x, y, b, m, c=self.ClickableCanvas:
                                self.onMouseReleased(x, y, b, m, c))
                            ui.Spacer()

                    # Capture Button
                    with ui.HStack(height=40):
                        ui.Spacer()
                        ui.Button(
                            "Capture",
                            clicked_fn=lambda: self.onCapture(), 
                            style={"margin": 5}, 
                            height=30,
                            width=self.thumbnail_width)
                        ui.Spacer()

                    # Count Widget
                    with ui.HStack(height=40):
                        ui.Spacer()
                        ui.Label(
                            'Count: ',
                            style={"margin": 5},
                            width=self.thumbnail_width/2,
                            alignment=ui.Alignment.RIGHT_CENTER)
                        self.count_model = ui.SimpleStringModel(
                            str(len(self.model.dataset)))
                        ui.StringField(
                            self.count_model,
                            style={"margin": 5},
                            height=30,
                            width=self.thumbnail_width/2)
                        ui.Spacer()

                    # Train and Eval Buttons
                    with ui.HStack(height=40):                       
                        ui.Spacer()
                        self.train_button = ui.Button(
                            "Train",
                            clicked_fn=lambda: self.train(),
                            style={"margin": 5},
                            height=30,
                            width=self.thumbnail_width/2)
                        self.eval_button = ui.ToolButton(
                            text="Evaluate",
                            style={"margin": 5},
                            height=30,
                            width=self.thumbnail_width/2)
                        self.eval_model = self.eval_button.model
                        self.eval_model.add_value_changed_fn(
                            lambda m: self.toggle_eval(m))
                        ui.Spacer()

                    # Model File Path Widget
                    with ui.HStack(height=40):
                        ui.Spacer()
                        ui.Label(
                            'model path: ',
                            style={"margin": 5},
                            height=30, width=self.thumbnail_width/2,
                            alignment=ui.Alignment.RIGHT_CENTER)
                        self.model_path_model = ui.SimpleStringModel(
                            'road_following_model.pth')
                        ui.StringField(
                            self.model_path_model,
                            style={"margin": 5},
                            height=30,
                            width=self.thumbnail_width/2)
                        ui.Spacer()

                    # Model Load and Save Buttons
                    with ui.HStack(height=40):
                        ui.Spacer()
                        ui.Button(
                            "load model",
                            clicked_fn=lambda: self.onLoadModel(),
                            style={"margin": 5},
                            height=30,
                            width=self.thumbnail_width/2)
                        ui.Button(
                            "save model",
                            clicked_fn=lambda: self.onSaveModel(),
                            style={"margin": 5},
                            height=30,
                            width=self.thumbnail_width/2)
                        ui.Spacer()

                    ui.Spacer()

    def destroy(self):
        super().destroy()
        if self.ov_update:
            self.ackerman_trainer_window.ov_update.unsubscribe()
            self.ov_update = None
            