# Copyright (c) 2020-2024, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from .ackermann_trainer_window import ackermann_trainer_window
import omni.ext

# omni.kit.pipapi extension is required
import omni.kit.pipapi

omni.kit.pipapi.install(
    package="torch"
)

omni.kit.pipapi.install(
    package='torchvision'
)


class AckermanAMRTrainerExtension(omni.ext.IExt):

    def on_startup(self, ext_id):
        print("Ackerman AMR Trainer Startup")

        # Build UI
        self._window = ackermann_trainer_window("Ackerman AMR Trainer",
                                                width=600,
                                                height=500)

    def on_shutdown(self):
        print("Ackerman AMR Trainer Shutdown")
        
        if self._window:
            self._window.destroy()
            self._window = None
