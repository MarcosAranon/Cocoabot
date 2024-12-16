# cocoabot
O Projeto Cocoabot é um manipulador robótico de 3 graus de liberdade desenvolvido para auxiliar na colheita de plantações de cacau. Ele busca automatizar o processo de colheita, reduzindo o esforço manual e aumentando a precisão e eficiência das operações no campo.

O projeto é dividido em dois pilares principais: Eletromecânica, que cuida da estrutura física, motores e componentes do robô, e Programação em ambiente ROS 2 (parte abordada neste repositório). Aqui estão incluídos os modelos do robô em URDF/XACRO, arquivos de lançamento para simulação e controle, além das configurações para ferramentas como Gazebo e RViz, permitindo a visualização, simulação e integração do manipulador.

**Pastas**
```
  COCOABOT/
  ├── docker/                          
  │   ├── config/                      
  │   └── scripts/                     
  │       ├── build.sh                                                 # Script para compilar o workspace utilizando colcon
  │       └── run.sh                                                   # Script para iniciar o container Docker configurado
  │
  ├── ros2_ws/                                                         # Workspace ROS2 principal
  │   ├── build/                                                       # Arquivos gerados pelo colcon build (ignorar)
  │   ├── install/                                                     # Pacotes instalados após o build (ignorar)
  │   ├── log/                                                         # Logs de execução e build
  │   └── src/                                                         # Código-fonte dos pacotes
  │       └── cocoabot_description/                                    # Pacote com a descrição e configuração do COCOABOT
  │           ├── launch/                                              # Arquivos de lançamento (Launch Files)
  │           │   ├── display.launch.py                                # Launch file para visualização do modelo no RViz
  │           │   │                                                      - Inicia o RViz2 com o modelo descrito em Xacro
  │           │   │                                                      - Inclui o `robot_state_publisher` para publicar o estado do robô
  │           │   │                                                      - Pode iniciar o `joint_state_publisher` para interação com as juntas
  │           │   └── gazebo.launch.py                                 # Launch file para simulação do robô no Gazebo
  │           │                                                          - Carrega o modelo no simulador Gazebo
  │           │                                                          - Configura os plugins necessários para interatividade física
  │
  │           ├── meshes/                                              # Modelos 3D do robô no formato STL
  │           │   ├── base_link.STL                                    # Modelo 3D da base do robô
  │           │   ├── lamina_link.STL                                       # Modelo 3D do link da lamina
  │           │   ├── ...                                              # Outros links e partes do robô
  │
  │           ├── urdf/                                                # Arquivos de descrição do robô em URDF e Xacro
  │           │   ├── cocoabot_description.urdf.xacro                  # Arquivo principal com a descrição do robô
  │           │   │                                                      - Define links, juntas e materiais usando Xacro
  │           │   │                                                      - Permite parametrização e reutilização do código
  │           │   ├── description.txt                                  # Arquivo auxiliar com propriedades e limites das juntas
  │           │   │                                                      - Contém dimensões de links e restrições das juntas
  │           │   └── material.xacro                                   # Definição dos materiais do robô (cores e aparência)
  │
  │           ├── rviz2/                                               # Configurações de visualização para o RViz2
  │           │   └── puma560_config.rviz                              # Arquivo de configuração do RViz
  │           │                                                          - Define câmeras, display e estilos de visualização
  │
  │           ├── CMakeLists.txt                                       # Configuração para compilar o pacote com colcon
  │           └── package.xml                                          # Metadados do pacote ROS2 (nome, versão, dependências)
```

# **Descrição Técnica**
# **URDF e Xacro**


O URDF (Unified Robot Description Format) é um formato baseado em XML usado para descrever a estrutura e dinâmica de um robô em ROS2. No COCOABOT, utilizamos Xacro (XML Macros) para facilitar a manutenção e reutilização dos arquivos.


  **Arquivos Xacro:**

  
    - cocoabot_description.urdf.xacro: Principal arquivo que descreve as juntas (joints), links, massas e inércias do COCOABOT.
    - material.xacro: Define cores e materiais utilizados para os links do robô.
    - description.txt: Define os parâmetros das juntas (limites de esforço, velocidade, posição) e dimensões dos links.
    

 **Explicação Geral:**

 
    - Cada link representa uma parte do robô (base, braços, etc).
    - Cada joint conecta dois links e pode ser do tipo revoluto (rotação) ou prismático (movimento linear).
    

**Lançamento de Dados e Interação dos Arquivos**


O cocoabot_description utiliza arquivos launch para facilitar a visualização e simulação do modelo do robô, além de gerenciar a comunicação entre os nós do ROS2. Abaixo, seguem os principais aspectos de como os dados são lançados e como os arquivos interagem:


  **Arquivos de Lançamento (launch)**
  

   display.launch.py:
   Utilizado para visualizar o modelo do robô no RViz.

   
        Nós principais:
          robot_state_publisher: Publica os estados e as transformações (frames) dos links e juntas do robô.
          joint_state_publisher_gui: Permite controlar manualmente os ângulos das juntas, fornecendo interatividade no RViz.
          rviz2: Abre o RViz com uma configuração específica para o COCOABOT.

  gazebo.launch.py:
  Utilizado para simular o modelo do robô no Gazebo.

  
        Nós principais:
          gazebo_ros: Inicia o ambiente de simulação do Gazebo.
          robot_state_publisher: Publica os frames do modelo URDF/XACRO no ambiente de simulação.
          Parâmetros adicionais:
            odem incluir variáveis como gravidade, física e configurações específicas do ambiente de simulação.

  Arquivos URDF e XACRO

  
    O XACRO é uma extensão do URDF que permite parametrizar o modelo do robô. Em vez de escrever tudo manualmente no URDF, o XACRO usa macros e propriedades, facilitando a modificação do modelo.
    O cocoabot_description.urdf.xacro inclui todas as descrições dos links, juntas e materiais do COCOABOT.
    Durante o lançamento:
        O robot_state_publisher lê o arquivo XACRO, o converte em URDF, e publica o parâmetro robot_description para os demais nós.
        O RViz e o Gazebo utilizam este parâmetro compartilhado para renderizar e simular o robô.

  Dados Compartilhados

      O arquivo description.txt contém propriedades importantes do robô, como limites das juntas, dimensões e parâmetros físicos, que são referenciados no XACRO.
      O RViz utiliza arquivos de configuração localizados na pasta config/ para personalizar a visualização do robô, como cores, frames e interatividade.
      A comunicação entre os nós (por exemplo, entre o joint_state_publisher_gui e o robot_state_publisher) é feita através de tópicos ROS2.

**Utilização**

  Primieramente os scripts de construção de imagem e para rodar o container de imagem devem ser usados. Assim considerando que o terminal foi aberto dentro da pasta Cocoabot:

    docker/scripts/build.sh

    docker/scripts/run.sh

  Após isso o usuário deve se localizar dentro do container. Assim basta executar os seguintes comandos inicialmente e toda vez que algum arquivo for atualizado ou modificado:

    colcon build

    source install/setup.bash

  Então para obter o atuador no rviz bta executar o seguinte comando:

    ros2 launch cocoabot_description display.launch.py

  E para executar a simulação no gazebo basta executar o seguinte comando:

    ros2 launch cocoabot_description gazebo.launch.py

    
