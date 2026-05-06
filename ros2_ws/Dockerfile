# Image officielle ROS 2 Humble Desktop
FROM osrf/ros:humble-desktop

# Éviter les interactions lors de l'installation apt
ENV DEBIAN_FRONTEND=noninteractive

# 1. Installation des outils de base et dépendances système
RUN apt-get update && apt-get install -y \
    wget \
    python3-pip \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# 2. Installation manuelle de Webots R2025a 
WORKDIR /tmp
RUN wget https://github.com/cyberbotics/webots/releases/download/R2025a/webots_2025a_amd64.deb \
    && apt-get update \
    && apt-get install -y ./webots_2025a_amd64.deb \
    && rm webots_2025a_amd64.deb \
    && rm -rf /var/lib/apt/lists/*

# Définir la variable d'environnement pour que ROS 2 trouve Webots
ENV WEBOTS_HOME /usr/local/webots

# 3. Installation des paquets ROS 2 spécifiés dans votre projet
RUN apt-get update && apt-get install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-webots-ros2-control \
    ros-humble-webots-ros2-driver \
    ros-humble-cv-bridge \
    ros-humble-rqt-image-view \
    && rm -rf /var/lib/apt/lists/*

# 4. Installation des dépendances Python (requirements.txt)
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# 5. Configuration de l'environnement de travail
WORKDIR /root/ros2_ws

# 6. Copie du code source
COPY . /root/ros2_ws/

# 7. Compilation du projet
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# 8. Configuration du script de démarrage
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]