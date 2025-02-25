#!/bin/bash

ros2 run serial_link_action_client trajectory_tracking_client \
  --ros-args \
  --params-file config/kuka_iiwa_14/iiwa_endpoint_poses.yaml \
  --params-file config/kuka_iiwa_14/iiwa_joint_configurations.yaml \
  --params-file config/tracking_tolerances.yaml
