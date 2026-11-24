# cdf_ws

Espace de travail ROS 2 (fork de `glcs_ws`).

## Compilation

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Caméra USB

### Dev container

```json
"runArgs": [
    "--device=/dev/video0",
    "--group-add=keep-groups",
    // [...]
]
```

```bash
# Lister les périphériques
v4l2-ctl --list-devices

# Lister les contrôles disponibles
v4l2-ctl -d /dev/video0 --list-ctrls

# Rotation
v4l2-ctl -d /dev/video0 --set-ctrl=rotate=180

# Sinon, retournement horizontal + vertical
v4l2-ctl -d /dev/video0 --set-ctrl=horizontal_flip=1
v4l2-ctl -d /dev/video0 --set-ctrl=vertical_flip=1
```

```bash
ros2 param set /mcc/camera/v4l2_camera rotate 180
```

Pour rendre le réglage permanent, ajouter `"rotate": 180` aux paramètres du
nœud dans `camera.launch.py`. Corriger l'orientation avant de lancer la
calibration.

### Lancer la caméra ZDC

```bash
ros2 launch cdf_bringup camera.launch.py

# Vérifier le flux lu par le nœud ArUco
ros2 topic hz /mcc/camera/image_raw
```

La caméra publie dans le namespace `/mcc/camera` avec le frame
`camera_link_optical`, ce qui correspond aux paramètres par défaut du nœud
ArUco.

### Calibrer la caméra

```bash
sudo apt install ros-lyrical-camera-calibration

ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 \
  image:=/mcc/camera/image_raw camera:=/mcc/camera
```

```bash
mkdir -p /home/ws/calibration /tmp/calib
tar -xzf /tmp/calibrationdata.tar.gz -C /tmp/calib
cp /tmp/calib/ost.yaml /home/ws/calibration/camera.yaml
```

```bash
ros2 launch cdf_bringup camera.launch.py \
  camera_info_url:=file:///home/ws/calibration/camera.yaml
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

# Navigation (simulation)
ros2 launch bnav nav.launch.py sim:=true

# Navigation (matériel réel)
ros2 launch bnav nav.launch.py sim:=false

# Détection de marqueurs ArUco (simulation)
ros2 launch aruco_pose_estimation aruco_pose_estimation.launch.py sim:=true

# Détection de marqueurs ArUco (matériel réel)
ros2 launch aruco_pose_estimation aruco_pose_estimation.launch.py sim:=false
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
