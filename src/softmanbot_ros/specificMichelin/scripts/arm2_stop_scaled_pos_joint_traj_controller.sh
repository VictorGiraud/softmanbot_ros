#!/bin/bash

rosservice call Arm2/controller_manager/switch_controller "{start_controllers:[], stop_controllers:['scaled_pos_joint_traj_controller'], strictness: 2}"


