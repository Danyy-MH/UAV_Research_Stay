# Research_stay_UAV
Repositorio para la programación del UAV de la Estancia de Investigación

simulation_4dof_tray1.launch lanza una simulación en Gazebo, es necesario mandar a través de rostopic pub /ref el mensaje tipo Twist para cambiar la referencia y que se veas reflejado en el simulador
 
## Simulación en PX4 SITL con custom model

Agregar 3012_hexaflat textualmente en el archivo CMakeLists.txt en la dirección:
PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt 
en la función px4_add_romfs_files(
    ...
    3012_hexaflat

Agregar el archivo 3012_hexaflat en la siguiente dirección:
PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes

Crear una carpeta que se llame específicamente "hexaflat" en la siguiente dirección:
PX4-Autopilot/Tools/simulation/gazebo/sitl_gazebo/models

Agregar hexaflat.sdf, model.config y la carpeta de meshes en la siguiente dirección (carpeta hexaflat previamente creada):
PX4-Autopilot/Tools/simulation/gazebo/sitl_gazebo/models/hexaflat/model.config

Agregar hexaflat.world en la siguiente dirección:
PX4-Autopilot/Tools/simulation/gazebo/sitl_gazebo/worlds

La simulación se corre con:
make px4_sitl gazebo_hexaflat
