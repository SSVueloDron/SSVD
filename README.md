# Vuelo de Dron Parrot Bebop 1 con control de teclado.

El presente documento detalla el proceso que se llevó a cabo para poner en vuelo el Dron Parrot Bebop 1 manejando desde el teclado, haciendo uso del software ROS, y Python.

A finales de 2014 Parrot presentó el Bebop 1, un dron con un procesador interno de doble núcleo ARM Cortex A9 que trabaja sobre Linux. Más adelante publicó un SDK (Software Development Kit), que posibilitó la creación del driver llamado bebop_autonomy el cual nos permite interactuar con el dron a través de Linux, utilizando ROS. El cual hemos instalado con anterioridad (ROS Installation - Documentos de Google)

A finales de 2015 Parrot lanzó al mercado el Bebop 2, un modelo mejorado y con más autonomía de vuelo que, sin embargo, se rige por el mismo sistema operativo haciendo también posible su control a través del driver bebop_autonomy. 

El Bebop 2 presenta una autonomía de hasta 25 minutos y permite la realización de vídeos en 1080p x 1920p y fotografías a 14 megapíxeles. 

Su velocidad máxima es de 60km/h y se estabiliza gracias a la acción de un acelerómetro, un giróscopo y un magnetómetro.

Gracias a sus dos antenas internas el Bebop 2 emite Wifi en 2,4Ghz y en 5Ghz, y cuenta con una baliza de luz de color rojo en la parte trasera para localizarlo en la distancia

Antes que nada debemos ver el manual de Uso del Parrot Bebop, el cual lo podemos encontrar en la siguiente dirección: [PARROT BEBOT](https://asset.conrad.com/media10/add/160267/c1/-/es/001300885ML03/manual-1300885-parrot-bebop-drone-blau-quadcopter-rtf-camera-drone-first-person-view-gps-function.pdf "PARROT BEBOT")

El cual nos presenta todas las funciones del dron, las cuales podemos hacer uso con el software ROS, esto con la finalidad de saber características específicas, como por ejemplo la orientación que el dron utiliza, etc.

Una vez conocidas las funciones del dron y sabiendo la funcionalidad de ROS, comenzamos con la comunicación con nuestro dron, para esto en terminal ejecutamos el siguiente comando:
`$ roslaunch bebop_driver bebop_node.launch`

Lo que hace este comando es ejecutar un nodo de Ros llamado bebop donde se asignan parámetros para poder identificar las características o funcionalidades al dron, podemos ver que obtenemos el ip del dron pues al conectarse al wifi del drop es la que este posee, bebop_node.launch contiene el código (Puede observarse en la siguiente imagen)  al que desde otra terminal se le pueden mandar diferentes comandos por ejemplo:

`$rosrun pub -- once /bebop/Takeoff std_msgs/Empty `

[![](https://github.com/SSVueloDron/SSVD/blob/main/img/bebop_nodelauch.png)](https://github.com/SSVueloDron/SSVD/blob/main/img/bebop_nodelauch.png)

El cual es un comando para hacer que el dron empiece a volar, es decir se mantiene en un solo lugar.

Lo siguiente ahora es hacer un mapeo de las teclas,en python, que se usaran para manejar el dron, se utilizaron las teclas 

-  W - Mover hacia delante
-  S -  Mover hacia atrás
-  A - Mover a la izquierda
-  D - Mover a la derecha
-  Q - Rotar el dron a la izquierda
-  E - Rotar el dron a la derecha
- T - Despegar
- Space - Aterrizar
- Flecha hacia Arriba - Incrementar altura del dron
- Flecha hacia Abajo - Decrecer altura del dron
- H - Hovering
- R - Release

## Código
- El código utilizado lo podemos ver enseguida: llamado `keyboard.py`

- Lo que necesitábamos ahora es un intérprete de estas teclas, para poder conectarse al dron y que este pudiera ejecutar lo deseado, esto se realizó con el siguiente código en Python llamado `interpreter.py :`

- Una vez que terminamos de codificar podemos ejecutarlos para poder manejar el dron, primero debemos ejecutar el intérprete para poder conectarse al dron, esto se hizo con el siguiente comando:

`$ python3 interpreter.py`

[![](https://github.com/SSVueloDron/SSVD/blob/main/img/interpreter.jpg)](https://github.com/SSVueloDron/SSVD/blob/main/img/interpreter.jpg)

- Ahora ejecutamos el comando para utilizar las teclas para maniobrar el dron:

`$ python3 keyboard.py`
- En seguida se nos mostrará lo siguiente:

[![](https://github.com/SSVueloDron/SSVD/blob/main/img/batery96.jpg)](https://github.com/SSVueloDron/SSVD/blob/main/img/batery96.jpg)

- Ahora podemos maniobrar el dron Parrot Bebop con el teclado. 
- Para poder Acceder a la cámara del Dron Bebop Utilizamos el comando:
`$ rqt_image_view `
- Y podemos ver lo siguiente el en pantalla:

[![](https://github.com/SSVueloDron/SSVD/blob/main/img/pantalla.png)](https://github.com/SSVueloDron/SSVD/blob/main/img/pantalla.png)


