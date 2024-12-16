import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Função principal que gera a descrição de lançamento da simulação
def generate_launch_description():
    
    # Obtém o diretório compartilhado do pacote `cocoabot_description` usando a função do ament_index, que localiza pacotes instalados no ambiente do ROS.
    puma560_description_dir = get_package_share_directory("cocoabot_description")

    # Declara um argumento de lançamento para o modelo do robô.
    # O argumento é o caminho absoluto para o arquivo URDF gerado pelo xacro, que contém a descrição do robô.
    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(puma560_description_dir,"urdf","cocoabot_description.urdf.xacro"),
        description="Caminho absoluto para o arquivo URDF do robô"
    )

    # Define a variável de ambiente `GZ_SIM_RESOURCE_PATH`, que indica ao Gazebo onde encontrar os recursos (arquivos de simulação) do robô.
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(puma560_description_dir).parent.resolve())]
    )

    # Verifica qual a distribuição do ROS sendo usada no ambiente.
    # Se for `humble`, usa o motor de física padrão do Gazebo, caso contrário, usa o plugin Bullet/Featherstone.
    ros_distro = os.environ["ROS_DISTRO"]
    physics_engine = "" if ros_distro == "humble" else "--physics-engine gz-physics-bullet-featherstone-plugin"

    # Define o parâmetro `robot_description` para o nó `robot_state_publisher`.
    # Ele usa o comando `xacro` para processar o arquivo xacro URDF e converter para o formato URDF final.
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]), 
        value_type=str
    )

    # Cria o nó `robot_state_publisher`, que publica o estado do robô (suas juntas e links) em tempo real, baseando-se na descrição URDF.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}]
    )

    # Inclui o arquivo de lançamento `gz_sim.launch.py`, que inicializa o Gazebo com os argumentos fornecidos.
    # O argumento `-v 4` define o nível de verbosidade do Gazebo, e o arquivo SDF (`empty.sdf`) define o mundo vazio.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")]
        ),
        launch_arguments=[
            ("gz_args", ["-v 4 -r empty.sdf ", physics_engine])
        ]
    )

    # Define o nó `create` do pacote `ros_gz_sim`, que cria a entidade do robô na simulação usando as informações publicadas no tópico `robot_description`.
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "cocoabot"]
    )

    # Configura o nó `parameter_bridge`, que faz a ponte entre os tópicos do Gazebo e do ROS, permitindo o compartilhamento de informações, como o tempo simulado.
    gz_ros2_bridge = Node(
        package="ros_gz_bridge", 
        executable="parameter_bridge", 
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msg.Clock]"]
    )

    # Retorna a LaunchDescription contendo todos os nós e variáveis configurados acima, prontos para serem lançados juntos.
    return LaunchDescription([
        model_arg,
        gazebo_resource_path, 
        robot_state_publisher_node,
        gazebo, 
        gz_spawn_entity,
        gz_ros2_bridge
    ])
