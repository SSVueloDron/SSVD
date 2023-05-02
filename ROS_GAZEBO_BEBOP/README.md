# Instalación de ROS
## ¿Qué es ROS? 
Robot Operating System es un framework open-source que permite a investigadores y desarrolladores a crear aplicaciones de robots. ROS provee abstracción de hardware, controladores de dispositivos, librerías, herramientas de visualización, comunicación por mensajes, administración de paquetes y más.
## Instalando ROS.
Se seguirán los pasos mostrados en  [Ubuntu install of ROS Noetic](http://wiki.ros.org/Installation/Ubuntu "Ubuntu install of ROS Noetic")
- Agregar la fuente a sources.list
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
- Agregar la llave
```
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
- Instalar la versión Desktop-Full
```
sudo apt install ros-noetic-desktop-full
```
- Modificar el archivo .bashrc para dar el path a nuestro script
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc 
source ~/.bashrc
```
- Instalar la dependencia rosinstall para descargar paquetes ROS y otras como rosdep, que se necesita para utilizar componentes clave de ROS
```
`sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential`
```
- Ahora podemos iniciar rosdep y actualizarlo
```
sudo rosdep init
rosdep update
```
- Ahora ROS está instalado, podemos verificar la instalación con 
```
roscore
```
![](https://github.com/SSVueloDron/SSVD/blob/main/img/roscore.png)

- Ahora que ROS está instalado, necesitamos configurar un workspace catkin con los siguientes comandos
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws
catkin_make
```
-  Luego añadimos el path al .bashrc
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
- Actualizamos las dependencias de ros con 
```
rosdep update
rosdep check –from-paths . –ignore-src –rosdistro noetic
```
-Si tenemos alguna dependencia faltante, entonces la instalamos con
```
rosdep instal –from-paths . –ignore-src -rosdistro noetic -y
```
# Instalación de Bebop driver
Para poder utilizar el dron Bebop necesitamos el driver que nos provee [Installation — bebop_autonomy indigo-devel documentation](https://bebop-autonomy.readthedocs.io/en/latest/installation.html "Installation — bebop_autonomy indigo-devel documentation")
```
sudo apt-get install python3-catkin-pkg python3-osrf-pycommon build-essential python3-rosdep python3-catkin-tools
```
- Si tenemos la versión ROS Noetic, las instrucciones de la página de bebop_autonomy no nos serán útiles, ya que recibiremos el error 
`bebop_driver: Cannot locate rosdep definition for [parrot_arsdk].`

Gracias al proyecto https://github.com/antonellabarisic/parrot_arsdk/tree/noetic_dev, podemos instalar el driver en la versión actual. Se siguen los pasos que se encuentran en el READ.MD del repositorio.

- Clonamos el proyecto en nuestro catkin workspace.
```
cd ~/catkin_ws/src
git clone https://github.com/antonellabarisic/parrot_arsdk.git
cd parrot_arsdk
git checkout noetic_dev
sudo apt-get install libavahi-client-dev
sudo ln -s /usr/bin/python3 /usr/bin/python
cd ~/catkin_ws
catkin_make
```
- Luego clonamos el repositorio donde se encuentra el driver para el bebop
```
cd ~/catkin_ws/src
git clone https://github.com/AutonomyLab/bebop_autonomy.git
```
- Una vez terminados estos pasos, modificamos el documento bebop_video_decoder.cpp
```
vim ~/catkin_ws/src/bebop_autonomy/bebop_driver/src/bebop_video_decoder.cpp
```
- y cambiamos tres líneas del documento a las que les agregaremos el prefijo AV_
 - En la línea 93:
 `CODEC_AP_TRUNCATED → AV_CODEC_CAP_TRUNCATED`
 - En la línea 95:
 `CODEC_FLAG_TRUNCATED →  AV_CODEC_FLAG_TRUNCATED`
 - En la línea 97:
 `CODEC_FLAG2_CHUNKS →  AV_CODEC_FLAG2_CHUNKS`
 
- Quedando de la siguiente manera:

![](https://github.com/SSVueloDron/SSVD/blob/main/img/bebopvideo.png)

- Luego añadimos la siguiente línea al .bashrc
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/catkin_ws/devel/lib/parrot_arsdk
```
![](https://github.com/SSVueloDron/SSVD/blob/main/img/bashedit.png)

- Instalamos los siguientes paquetes
```
sudo apt install ros-noetic-joy ros-noetic-joy-teleop ros-noetic-teleop-twist-joy
```
- Checamos si hace falta una dependencia
```
rosdep update
rosdep install --from-paths src -i
```
- Y construimos
```
cd ~/catkin_ws/
catkin_make
```
- La instalación ha finalizado.


# YOLO y ROS

YOLO es un sistema de detección de objetos en tiempo real. El sitio web del proyecto es [YOLO: Real-Time Object Detection.](https://pjreddie.com/darknet/yolo/ "YOLO: Real-Time Object Detection.")

Existe un paquete desarrollado para unir ROS y YOLO, el sitio es [GitHub - leggedrobotics/darknet_ros: YOLO ROS: Real-Time Object Detection for ROS.](https://github.com/leggedrobotics/darknet_ros "GitHub - leggedrobotics/darknet_ros: YOLO ROS: Real-Time Object Detection for ROS.")

Para instalar el paquete debemos seguir los siguientes pasos:
- Nos movemos a nuestro catkin workspace 
```
cd ~/catkin_ws/src
```
- Clonamos usando SSH, por lo que debemos configurarlo antes.
```
git clone --recursive 
git@github.com:leggedrobotics/darknet_ros.git
```
- Modificamos el archivo Makefile en `~/catkin_ws/src/darknet_ros/darknet/Makefile` y borramos `-gencode arch=compute_30,code=sm_30`  de la línea 7. 
- Ahora quedará de la siguiente manera:

![](https://github.com/SSVueloDron/SSVD/blob/main/img/makefile1.png)

- También borraremos la misma línea del documento `~/catkin_ws/src/darknet_ros/darknet_ros/CMakeLists.txt` que se encuentra en la linea 26. 
- Quedará de la siguiente manera:

![](https://github.com/SSVueloDron/SSVD/blob/main/img/makefile2.png)

- Volvemos al workspace y hacemos el build en Release mode.
```
cd ../
catkin_make -DCMAKE_BUILD_TYPE=Release
```

- Listo, ya se tiene YOLO ROS instalado.

## Instalación de CUDA
Queremos que YOLO detecte imágenes en tiempo real, y con la instalación de arriba estamos corriendo YOLO con el procesador, que tarda de 10 a 20 segundos en un procesador reciente.  (Aproximadamente 15 segundos en un Ryzen 5 2600X). Por lo tanto, necesitamos correr YOLO en la GPU, así que necesitamos instalar CUDA.
- Seguimos las instrucciones de [CUDA Toolkit 11.6 Update 1 Downloads | NVIDIA Developer.](https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=20.04&target_type=deb_network "CUDA Toolkit 11.6 Update 1 Downloads | NVIDIA Developer.")

```
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/7fa2af80.pub
sudo add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/ /"
sudo apt-get update
sudo apt-get -y install cuda


```
- Ahora añadimos las siguientes líneas a nuestro .bashrc
```
export PATH=/usr/local/cuda-11.6/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda-11.6/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
```

## Activando CUDA y OPENCV en yolo_ros
- Cambiamos la primera línea del Makefile en `~/catkin_ws/src/darknet_ros/darknet/Makefile` para que diga GPU=1 y OPENCV=1 en la línea 3.
- También se cambiará la versión de opencv a opencv4 de la siguiente manera:

![](https://github.com/SSVueloDron/SSVD/blob/main/img/actCuda.png)

```
cd ~/catkin_ws/src/darknet_ros/darknet
make
```
