#!/usr/bin/env python3

# Copyright 2022 Trossen Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import sys

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np


"""
This script makes the end-effector go to a specific pose by defining the pose components

To get started, open a terminal and type:

    ros2 launch interbotix_xsarm_control xsarm_control.launch robot_model:=wx250

Then change to this directory and type:

    python3 ee_pose_components.py
"""


def main():
    bot = InterbotixManipulatorXS(
        robot_model='px150',
        group_name='arm',
        gripper_name='gripper'
    )

    bot.gripper.left_finger_lower_limit=0.01


    bot.arm.go_to_home_pose()
    bot.gripper.release()
    bot.arm.set_ee_pose_components(x=0.2, z=0,pitch=1.5, moving_time=4)
    bot.gripper.grasp(3)
    bot.arm.set_ee_pose_components(x=-0.25,z=0.15,moving_time=4)
    bot.gripper.release()
    bot.gripper.grasp()
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()

    bot.shutdown()

     

if __name__ == '__main__':
    main()
    
















