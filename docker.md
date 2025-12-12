DO NOT USE DOCKER DESKTOP

sudo pacman -Syu docker docker-compose
sudo systemctl start docker
sudo systemctl enable docker
sudo usermod -aG docker $USER
newgrp docker
docker context use default
docker compose build
xhost +local:docker
docker compose up -d
docker exec -it ros2_dev_env bash
