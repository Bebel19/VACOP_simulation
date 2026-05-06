#!/bin/bash
set -e

# 1. Sourcer l'environnement ROS 2 global
source /opt/ros/humble/setup.bash

# 2. Sourcer le workspace du projet s'il est compilé
if [ -f /root/ros2_ws/install/setup.bash ]; then
  source /root/ros2_ws/install/setup.bash
fi

# 3. Exécuter la commande demandée par Docker
exec "$@"