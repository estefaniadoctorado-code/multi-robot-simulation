# multi-robot-simulation

ROS1 multi robot simulation

1. Clonar en la carpeta src de `catkin_ws`
```bash
cd ~/catkin_ws/src
git clone <este repo>
```
2. Compilar los paquetes
Compilar el paquete Mundos
```bash
catkin build planning_worlds
```
Compilar el paquete Robot description
```bash
catkin build untamo_description
```
Compilar el Paquete para enviar las coordenadas
```bash
catkin build multiple_robot_navigation
```

3. Correr las simulaciones.

Se necesitan varias ventanas de terminal:
```bash
roslaunch multiple_robot_navigation experiment1.launch
roslaunch multiple_robot_navigation experiment1_rviz_nav.launch
```

Ejecutar las planificaciones (cada launchfile ejecuta una planificación diferente):
```bash
roslaunch multiple_robot_navigation experiment1_path_publisher_astar.launch
roslaunch multiple_robot_navigation experiment1_path_publisher_global.launch
roslaunch multiple_robot_navigation experiment1_path_publisher_MPP.launch
roslaunch multiple_robot_navigation experiment1_path_publisher_partial.launch
```

## Caminos
Dentro de cada archivo `experiment1_path_publisher_MPP.launch` está el script que ejecuta la planificación por el camino que se le pasa.
```xml
  <group ns="robot0">
    <param name="filename" value="$(find planning_worlds)/worlds/maps/experiment1/R1_MPP.txt"/>
    <param name="robotid" value="0"/>
    <param name="metricpath" value="True"/>
    <node name="robot0_path_viewer" pkg="multiple_robot_navigation" type="path_publisher.py">
    </node>
    <node name="robot0_goals_publisher" pkg="multiple_robot_navigation" type="goal_publisher.py">
    </node>
  </group>
```

El archivo `R1_MPP.txt` es donde está el camino que ejecuta el robot.

## Posición inicial de los robots
En el archivo `launch/experiment1/experiment1_nav.launch` está la cantidad de robots y su posición inicial en el mapa:
```xml
<!-- BEGIN ROBOT 0 -->
  <group ns="robot0">
    <param name="tf_prefix" value="robot0_tf" />
    <include file="$(find multiple_robot_navigation)/launch/experiment1/one_robot0_nav.launch" >
    <arg name="init_pose" value="-x -7.05736195038 -y 15.7290799262 -z 0.0 -Y $(arg yaw)" />
      <arg name="namespace"  value="robot0" />
    </include>
  </group>
```

## Mundo para la simulación
El mundo se puede configurar desde el archivo `experiment1.launch`:
```xml
<param name="/use_sim_time" value="true" />
<arg name="world" default="boxes3" />
<arg name="type" default="nav" />
<arg name="yaw" default="0.0" />
```
Los mundos están en el paquete `planning_worlds` en la carpeta `worlds`



