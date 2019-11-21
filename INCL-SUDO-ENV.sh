#!/bin/bash

################################################################################

# Pass 'sudo' privileges if previously granted in parent scripts.
if [ ! -z "$SUDO_USER" ]; then
  export USER=$SUDO_USER
fi

################################################################################

# Install Docker
curl https://get.docker.com | sh \
  && sudo systemctl --now enable docker

# Test the Docker installation after making sure that the service is running.
service docker stop
service docker start
while ! pgrep dockerd > /dev/null; do
  sleep 1
done
docker version
docker run --rm hello-world

################################################################################

# Add the current user to the 'docker' group to run Docker without 'sudo'.
# Logging out and back in is required for the group change to take effect.
usermod -a -G docker ${USER}
echo "Added the current user '${USER}' to the 'docker' group."

# Configure the host system so that 'adduser' command adds future new users to the 'docker' group automatically.
# This enables new users to set up their environment without 'sudo' by only executing 'INCL-USER-ENV.sh'.
ADDUSER_CONFIG=/etc/adduser.conf
if [ ! -f ${ADDUSER_CONFIG} ]; then
  echo "Failed to add future new users to the 'docker' group because the system configuration file '${ADDUSER_CONFIG}' was not found."
else
  if ! grep -q "#EXTRA_GROUPS=\"dialout cdrom floppy audio video plugdev users\"" ${ADDUSER_CONFIG}; then
    echo "Failed to add future new users to the 'docker' group because 'EXTRA_GROUPS' in '${ADDUSER_CONFIG}' has already been customized."
  else
    sed -i 's/#EXTRA_GROUPS="dialout cdrom floppy audio video plugdev users"/EXTRA_GROUPS="dialout cdrom floppy audio video plugdev users docker"/' ${ADDUSER_CONFIG}
    sed -i 's/#ADD_EXTRA_GROUPS=1/ADD_EXTRA_GROUPS=1/' ${ADDUSER_CONFIG}
    echo "Modified '${ADDUSER_CONFIG}' to add all future new users to the 'docker' group upon creation."
  fi
fi

################################################################################

# Install Docker Compose.
# https://docs.docker.com/compose/install/#install-compose
# https://github.com/docker/compose/releases

# Install Docker Compose version 'DOCKER_COMPOSE_VERSION'.
apt-get update && apt-get install -y nvidia-docker2

################################################################################

# Install Nvidia Docker 2.
# https://github.com/NVIDIA/nvidia-docker
# https://github.com/NVIDIA/nvidia-docker/wiki/Usage
# https://github.com/nvidia/nvidia-container-runtime#environment-variables-oci-spec

# Add the Nvidia Docker package repositories.
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
   && curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add - \
   && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

# Install 'nvidia-docker2' version 'NVIDIA_DOCKER_VERSION' and reload the Docker daemon configuration.
apt-get update && apt-get install -y nvidia-docker2

# Test the Nvidia Docker installation after making sure that the service is running and that Nvidia drivers are found.
service docker stop
service docker start
while ! pgrep dockerd > /dev/null; do
  sleep 1
done
if [ -e /proc/driver/nvidia/version ]; then
  docker run --runtime=nvidia --rm nvidia/cuda nvidia-smi
fi

################################################################################

# Install Terminator terminal.
# https://gnometerminator.blogspot.com/

# Install the latest version of Terminator from the Ubuntu repositories.
apt-get update && apt-get install -y \
  terminator

# Prevent the Terminator installation to replace the default Ubuntu terminal.
update-alternatives --set x-terminal-emulator /usr/bin/gnome-terminal.wrapper
