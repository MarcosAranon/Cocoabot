#!/bin/bash

cd docker

echo "Você acessou a pasta da Aula: $PWD"
    
echo "Construindo o Imagem ROS"
    
docker build \
    --network=host \
    -f ROS_humble.dockerfile \
    -t ros2_ws:humble \
    --rm \
    .
    
exit 0



