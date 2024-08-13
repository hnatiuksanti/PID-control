# PID-control
En este repositorio se muestran los códigos para la calibración de los parámetros PID, a lazo abierto (pid.py) y cerrado con botón de stop (MedicionContinua_conStop.py) y calibración por método de Ziegler-Nichols, para el control de la velocidad de una rueda. Para el control del motor se utilizó una placa NUCLEO F334R8.

# ¿Qué es un control PID? 

Un capitán de barco, para corregir la dirección, debe tener en cuenta el error en la dirección, su velocidad, y sus correcciones anteriores. De igual forma, para controlar alguna variable en un proceso contínuo (temperatura, velocidad, dirección, presión) se fija un valor deseado (Setpoint) y se mide su error con respecto a este, la velocidad de cambio de la variable, y sus cambios anteriores. A estos tres parámetros se los conoce como Proporcional, Derivativo e Integral, respectivamente. De ahí el nombre 'Control PID'. Utilizando estos tres parámetros se logra controlar de manera automática la/las variables deseadas de un dado proceso.
![Esquema de control PID](https://github.com/hnatiuksanti/PID-control/blob/main/Diagramas/PID_diagrama%20de%20bloques.jpg)



## Montaje experimental y la conexiones de la placa. 
![Esquema experimental PID](https://github.com/hnatiuksanti/PID-control/blob/main/Diagramas/montaje_exp.png)
![Conexiones placa NUCLEO F334R8](https://github.com/hnatiuksanti/PID-control/blob/main/Diagramas/nucleo.png)

# Performance del controlador PID. 
### Resultados luego de una calibración manual de los parámetros P,I,D. Las unidades de tiempo se muestran en milisengundo, la velocidad es caulitativa. 
![Vel vs Tiempo PID](https://github.com/hnatiuksanti/PID-control/blob/main/Diagramas/Vel_Tiempo_PID.png)


### Resultados luego de una calibración por método de Ziegler-Nichols de los parámetros P,I,D. La velocidad es caulitativa. 
![Vel vs Tiempo ZN-PID](https://github.com/hnatiuksanti/PID-control/blob/main/Diagramas/Vel_Tiempo_PID_ZN.png)


### Control PID frente a un freno externo. 
![Fuerza PID](https://github.com/hnatiuksanti/PID-control/blob/main/Diagramas/Vel_Tiempo_Confuerza_externa.png)

