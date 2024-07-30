# PID-control
Repositorio con los código para el control PID de un motor de continua. Se incluye breve esquema experimental. 

¿Qué es un control PID? 

Un capitán de barco, para corregir la dirección, debe tener en cuenta el error en la dirección, su velocidad, y sus correcciones anteriores. De igual forma, para controlar alguna variable en un proceso contínuo (temperatura, velocidad, dirección, presión) se fija un valor deseado (Setpoint) y se mide su error con respecto a este, la velocidad de cambio de la variable, y sus cambios anteriores. A estos tres parámetros se los conoce como Proporcional, Derivativo e Integral, respectivamente. De ahí el nombre 'Control PID'. Utilizando estos tres parámetros se logra controlar de manera automática la/las variables deseadas de un dado proceso.
![Esquema de control PID](https://github.com/hnatiuksanti/PID-control/blob/main/Diagramas/PID_diagrama%20de%20bloques.jpg)

En este repositorio se muestran los códigos para la calibración de los parámetros PID, a lazo abierto (pid.py) y cerrado con botón de stop (MedicionContinua_conStop.py), para el control de la velocidad de una rueda. Para el control del motor se utilizó una placa NUCLEO F334R8. A continuación se muestra el montaje experimental y la conexiones de la placa. 
![Esquema experimental PID](https://github.com/hnatiuksanti/PID-control/blob/main/Diagramas/montaje_exp.png)
![Conexiones placa NUCLEO F334R8](https://github.com/hnatiuksanti/PID-control/blob/main/Diagramas/nucleo.png)
