# Robot Controller e KillSwitch

## Demonstração em Vídeo

[Vídeo de Demonstração](https://drive.google.com/file/d/17lp4SG_v2WHbRr3-P5KEkQRdHMMQdkfZ/view?usp=sharing)

## Descrição

Este projeto implementa um sistema de controle de robô utilizando ROS 2. Ele inclui um cliente para controlar um robô (teleoperação) e um serviço para lidar com a parada de emergência.

## Estrutura do Projeto

O projeto está organizado em dois pacotes principais:

- `new_bringup_ws`: Contém o serviço para lidar com o bring up e o killswitch.
- `my_robot_controller`: Contém o cliente para teleoperação do robô.

## Requisitos

- ROS 2 (Humble)
- Python 3.10
- `typer`, `rclpy`, `geometry_msgs`, `std_srvs`, `inquirer`

### Configurar o Workspace

#### No Robô

1. Clone o Repo:

   ```sh
   git clone https://github.com/LuanRM1/Pond_bot1/tree/main
   ```

2. Mova o pacote `new_bringup_ws` para fora do repo:

   ```sh
   cd Pond_bot1/src
   mv -r new_bringup_ws ~/home/
   cd seu/diretorio
   ```

#### No Computador

1. Clone o Repo:

   ```sh
   git clone https://github.com/LuanRM1/Pond_bot1/tree/main
   cd Pond_bot1
   ```

2. Apague o `new_bringup_ws`:

   ```sh
   cd Pond_bot1/src
   rm -rf new_bringup_ws
   ```

### Instalar Dependências

Instale as dependências necessárias em ambos, robô e computador:

```sh
pip install typer inquirer
```

### Construir o Workspace

Construa o workspace ROS 2 em ambos, robô e computador:

```sh
cd seu/diretorio
colcon build
```

### Source o Ambiente

Source o ambiente ROS 2 em ambos, robô e computador:

```sh
source install/setup.bash
```

## Execução

### Executar o Serviço no Robô

No robô, execute o serviço que lida com a parada de emergência:

```sh
ros2 run new_bringup bringup_manager
```

### Executar o Cliente no Computador

No seu computador, que deve estar na mesma rede do robô, execute o cliente para controlar o robô:

```sh
ros2 run my_robot_controller robot_controller
```
