#!/bin/bash

rosservice call Arm1/controller_manager/switch_controller "{start_controllers:['cartesian_velocity_control'], stop_controllers:[], strictness: 2}"

#rosservice call controller_manager/switch_controller "{start_controllers:['joint_array_control'], stop_controllers:[], strictness: 2}"

