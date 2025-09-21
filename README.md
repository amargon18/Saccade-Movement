## SACCADE MOVEMENT
El proyecto SACCADE implementa un sistema de control de movimientos rápidos de cabeza y ojos (sacadas) en el iCub Simulator, utilizando la infraestructura de comunicación YARP.

El programa conecta el simulador con un controlador PD que recibe como entrada la posición de un objetivo visual (una esfera dentro del mundo simulado) y genera comandos de movimiento para la cabeza y los ojos del robot. Además, registra datos de posiciones y errores en ficheros CSV para su posterior análisis y ofrece un script en Python para graficar los resultados.

En conjunto, el proyecto permite simular, controlar y evaluar el seguimiento visual del robot iCub en un entorno virtual.

El proyecto incluye un ejecutable en C++ que:
- Se conecta al controlboard remoto del simulador (/icubSim/head) para leer encoders y comandar cabeza y ojos.
- Ejecuta un control PD (proporcional-derivativo) para seguir un objetivo visual.
- Mueve un objetivo esférico en el mundo simulado a través de un puerto RPC, generando trayectorias que el controlador debe seguir.
- Registra datos de posición de cabeza y ojos, así como errores, en ficheros CSV bajo data/.
- Incluye un script Python (graphs.py) para graficar la evolución temporal.
  
**Funcionalidades**
- Conexión YARP a remote_controlboard del simulador.
- Seguimiento de objetivo con controlador PD.
- Generación/movimiento de objetivo vía RPC (esfera en el mundo simulado).

**Tecnologías utilizadas**
- C++
- YARP
- iCub Simulator 
- CMake

  




