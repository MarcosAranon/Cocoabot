#!/bin/bash

echo "Você acessou a pasta da Aula: $PWD"
    
echo "Criando o Container"

# Allow local connections to X server
xhost +local:docker

# Run the Docker container with GPU support if available

# name ros_humble        -> Nomeia o contêiner como "ros_humble"
# user $(id -u):$(id -g) -> Define o usuário e grupo para o contêiner como o mesmo do host
# -e DISPLAY=$DISPLAY    -> Passa a variável DISPLAY para permitir que o contêiner use a interface gráfica do host
# -e QT_X11_NO_MITSHM=1  -> Corrige possíveis problemas de compartilhamento de memória com o Qt e o servidor X
# -e XDG_RUNTIME_DIR=/tmp/runtime-ros2_ws -> Define o diretório de runtime temporário para o ROS
# --gpus all             -> Habilita o uso de todas as GPUs, se disponível
# --network=host         -> Configura o contêiner para usar a mesma rede do host
# --ipc=host             -> Compartilha o IPC (inter-process communication) com o host
# --privileged           -> Permite ao contêiner acessar dispositivos e recursos com privilégios elevados
# --device /dev/video0   -> Concede acesso ao dispositivo de vídeo (câmera) no contêiner
# -v /dev/video0:/dev/video0 -> Mapeia o dispositivo de vídeo do host para o contêiner
# -v /tmp/.X11-unix:/tmp/.X11-unix:rw -> Mapeia o socket do servidor X para permitir que o contêiner use a interface gráfica
# -v "$PWD/ros2_ws:/home/ros2_ws:rw" -> Mapeia o diretório "ros2_ws" do host para o contêiner com permissão de leitura e escrita
# --workdir /home        -> Define o diretório de trabalho do contêiner como /home
# ros2_ws:humble         -> Especifica a imagem do contêiner a ser usada, que é "ros2_ws:humble"

docker run -it --rm \
    $USE_GPUS \
    --name ros_humble \
    -e HOME=/home/ros2_ws \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XDG_RUNTIME_DIR=/tmp/runtime-ros2_ws \
    --network=host \
    --ipc=host \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$PWD/ros2_ws:/home/ros2_ws:rw" \
    --workdir /home/ros2_ws \
    ros2_ws:humble
