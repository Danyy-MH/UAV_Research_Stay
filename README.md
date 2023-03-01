# Research_stay_UAV
Repositorio para la programación del UAV de la Estancia de Investigación
 
## Simulación en PX4 SITL con custom model

### Clonar PX4-Autopilot 

https://github.com/PX4/PX4-Autopilot.git

Agregar 3012_hexaflat textualmente en el archivo CMakeLists.txt en la dirección:
PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt 
en la función
```
px4_add_romfs_files(
    ...
    3012_hexaflat
```

Agregar el archivo 3012_hexaflat en la siguiente dirección:
PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes

Crear una carpeta que se llame específicamente "hexaflat" en la siguiente dirección:
PX4-Autopilot/Tools/simulation/gazebo/sitl_gazebo/models

Agregar hexaflat.sdf, model.config y la carpeta de meshes en la siguiente dirección (carpeta hexaflat previamente creada):
PX4-Autopilot/Tools/simulation/gazebo/sitl_gazebo/models/hexaflat/model.config

Agregar hexaflat.world en la siguiente dirección:
PX4-Autopilot/Tools/simulation/gazebo/sitl_gazebo/worlds

Agregar la carpeta carga_util en el siguieten path:
PX4-Autopilot/Tools/simulation/gazebo/sitl_gazebo/models

Agregar el comando hexaflat en el archivo de sitl_targets_gazebo.cmake en la dirección src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake

```cmake

set(models
	none
...
	uuv_hippocampus
	hexaflat
)

```


La simulación se corre con:
make px4_sitl gazebo_hexaflat

Simulacion con roslaunch: 
Dentro del directorio de PX4-Autopilot correr:

```

source Tools/simulation/gazebo/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo/sitl_gazebo
roslaunch px4 mavros_posix_sitl.launch

```
