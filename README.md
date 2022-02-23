# Webots
Entregable para la asignatura de Simuladores de Robots del Máster de Robótica y Automatización (UC3M)

- Se ha implementado un algoritmo para que el robot sea capaz de moverse de una posición inicial hasta la meta.
- Las tareas realizadas se encuentran implementadas en el siguiente fichero [here](controllers/my_controller_cpp//my_controller_cpp.cpp)

# Extras
- Extra 1: El código se ha subido a un repositorio de [GitHub](https://github.com/lucas-rib-oli/P1_Webots)    
- Extra 2: Se ha añadido el uso de argumentos dentro de "controllerArgs" para escoger los diferentes métodos desarrollados para resolver el problema.
- Extra 3: Se ha utilizado el [script](webots-tools-master/greedy_path_planning.py) desarrollado en Planificación de Robots para implementar el algoritmo A<sup> * </sup>.
- Extra 4: Se ha utilizado el [script](webots-tools-master/greedy_path_planning.py) desarrollado en Planificación de Robots para implementar el algoritmo Breadth-first Search.
- Extra 5: Se ha utilizado el [script](webots-tools-master/greedy_path_planning.py) desarrollado en Planificación de Robots para implementar el algoritmo Depth-first search.
- Extra 6: Se ha utilizado el [script](webots-tools-master/greedy_path_planning.py) desarrollado en Planificación de Robots para implementar el algoritmo Best-first search.
- Extra 7: Para la comunicación entre el script de Python y el código del controlador en c++ se han guardado los puntos del path en ficheros para cada uno de los algoritmos de planificación. Los ficheros se encuentran en [here](webots-tools-master/)
- Extra 8 Se ha implementado recursividad de carpetas para evitar problemas con las rutas hardcodeadas.
- Extra 9: Se han implementado tres formas de resolver el mapa:
- Extra 9.1: Dando el punto de meta y con el uso de sensores se esquivan los obstáculos (method="sensors")
- Extra 9.2: Siguiendo los puntos descritos por los distintos algoritmos de planificación con el uso del GPS.
- Extra 9.3: Siguiendo los puntos por defecto con el GPS que evitan obstáculos. 
- Extra 10: Se visualiza la cámara del robot para ver el recorrido que hace. 
- Extra 11: Al llegar al punto objetivo se activan los LEDs que dispone el robot.
- Extra 12: Se ha sacado por imagen la trayectoria descrita por los algoritmos de planificación para llegar al punto de meta. Las imágenes se encuentran [here](webots-tools-master/images/).
- Extra 13: Cuando el problema se resuelva haciendo el uso del método de los sensores (method="sensors"), los valores de destino se introducen a través de argumentos del programa ("controllerArgs"), para la coordenada X argumento (2) y para la coordenada Y argumento (3).
- Extra 14: Se han tenido en cuenta las dimensiones del robot para calcular la velocidad tangencial y angular del robot con la velocidad de giro de las ruedas.
- Extra 15: Link a vídeo: https://youtu.be/1uvd1lUIXHE


Para ejecutar el código de planificación de trayectorias:
```
$ python main_greedy.py --root ROOT/GreedyPathPlanning/ --map 1 --start_x 2 --start_y 2 --end_x 5--end_y 5 --algorithm a_star --neighbourhood 8 --cost euclidean --viz 1 --video 0
```

