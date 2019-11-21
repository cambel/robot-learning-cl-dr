# Set default Docker runtime to use in './docker/docker-compose.yml'.
if ! grep -q "export DOCKER_RUNTIME" ~/.bashrc; then
  if [ -e /proc/driver/nvidia/version ]; then
    DOCKER_RUNTIME=nvidia
  else
    DOCKER_RUNTIME=runc
  fi
  cat <<EOF >> ~/.bashrc

# Set default Docker runtime to use in '~/o2ac-ur/docker/docker-compose.yml':
# 'runc' (Docker default) or 'nvidia' (Nvidia Docker 2).
export DOCKER_RUNTIME=${DOCKER_RUNTIME}
EOF
fi
