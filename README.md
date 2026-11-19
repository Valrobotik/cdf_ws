# cdf_ws

Espace de travail ROS 2 (fork de `glcs_ws`).

## Compilation

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Raspberry Pi: interface CAN

```bash
# dtparam=spi=on
# dtoverlay=mcp251xfd,spi0-0,interrupt=7,oscillator=40000000

# Outils CAN
sudo apt install can-utils

# Vérifier que le contrôleur est détecté
dmesg | grep -i mcp251xfd
ip link show can0

# Démarrer can0 (500 kbit/s, FD sans BRS)
sudo ip link set can0 up type can bitrate 500000 dbitrate 500000 fd on

# Vérifier / écouter le bus
ip -details link show can0
candump can0

# Arrêter
sudo ip link set can0 down
```

## Lancer les nodes

```bash
# Simulation Gazebo (ros2_control via gz_ros2_control)
ros2 launch cdf_bringup bringup.launch.py sim:=true

# Matériel réel (stepper_foc_1_hw sur can0)
ros2 launch cdf_bringup bringup.launch.py sim:=false

# Matériel ros2_control (stepper FOC)
ros2 launch stepper_foc_1_hw stepper_foc_test.launch.py

# Navigation
ros2 launch bnav nav.launch.py

# Détection de marqueurs ArUco
ros2 launch aruco_pose_estimation aruco_pose_estimation.launch.py
```

## Ajouter des boîtes ArUco dans Gazebo

```bash
# Modèle Box_Aruco
ros2 run ros_gz_sim create \
  -file "$(ros2 pkg prefix cdf_bringup)/share/cdf_bringup/model/Pierre/model.sdf" \
  -name box1 -x 0.5 -y 0.5 -z 0.055 -R 1.5708

# Autre boîte
ros2 run ros_gz_sim create \
  -file "$(ros2 pkg prefix cdf_bringup)/share/cdf_bringup/model/Pierre/model.sdf" \
  -name box2 -x -0.5 -y 0.5 -z 0.055 -R 1.5708 -Y 1.57
```

## Téléopération (clavier twist, `stamped`)

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

## Tester le matériel ros2_control

```bash
ros2 control list_hardware_components
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 topic echo /joint_states

# Bouger les deux articulations des steppers (contrôleur de vitesse)
ros2 topic pub --once /velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.0, 1.0]}"
```

## Commandes utiles

```bash
ros2 node list
ros2 topic list
ros2 topic echo /cmd_vel
ros2 topic echo /odom
ros2 topic hz /joint_states
```
