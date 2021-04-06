#!/bin/bash

rosservice call Arm2/controller_manager/switch_controller "{start_controllers:['cartesian_velocity_control'], stop_controllers:[], strictness: 2}"


