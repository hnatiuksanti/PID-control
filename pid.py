# -*- coding: utf-8 -*-
"""
Created on Wed Aug 31 07:57:10 2022

@author: Publico
"""

import serial
import time
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

#%% Iniciar controlador serie
def start():
  ser = serial.Serial(port='COM4', baudrate=115200, bytesize=8, parity='N', stopbits=1, timeout=0.005, xonxoff=0, rtscts=0)
  ser.close() 
  ser.open()
  return ser

#reset controlador
def reset(ser):
  ser.write(bytes('X','utf-8')) 
  time.sleep(0.01)
  ser.flushInput()

#escribo voltaje, pregunto posicion y velocidad
def escribir_preguntar():
  str = 'V0\n\r'
  ser.write(bytes(str,'utf-8'))
  time.sleep(0.002)
  s = ser.readline(25)
  print(s)
ser=start()
reset(ser)

#%% Defino funciones de comunicacion con el controlador

def setVoltageGetData(puerto,voltaje):
    puerto.flushInput()
    str = 'V%f\n\r' % (voltaje)
    puerto.write(bytes(str,'utf-8'))
    time.sleep(0.002)
    s = puerto.readline(25)
    pos = float(s[0:9])
    vel = float(s[10:23])  
    return pos,vel

def resetControlador(puerto):
    puerto.write(bytes('X','utf-8')) 
    time.sleep(0.01)
    puerto.flushInput()

def voltajeCorregido(voltaje):
    voltpos = 2
    voltneg = 2 
    maxvolt = 12
    if(voltaje > 0 ):        
        voltaje *= maxvolt/(maxvolt+voltpos)
        voltaje += voltpos
    else:        
        voltaje *= maxvolt/(maxvolt+voltneg)
        voltaje -= voltneg
    return voltaje

#%% Respuesta a un pulso de voltaje
def respuestaalpulso(volt):    
  resetControlador(ser) #reseteo el controlador
  time.sleep(0.2)

  #inicializo variables
  voltajes = np.concatenate((0*np.ones(10) ,volt*np.ones(140), 0*np.ones(60)))
  N = len(voltajes)
  posiciones = np.zeros(N)
  posiciones[:] = np.nan
  velocidades=np.zeros(N)
  velocidades[:] = np.nan
  tiempos=np.zeros(N)
  tiempos[:] =np.nan

  #loop poniendo voltaje y midiendo posicion y velocidad
  toc = time.time()
  for i in range(N):
    time.sleep(.04)
    pos,vel = setVoltageGetData(ser,voltajes[i]) 
    time.sleep(.04)
    posiciones[i] = pos
    velocidades[i] = vel
    tiempos[i] = time.time()-toc
  return[tiempos,voltajes,posiciones,velocidades]

Variables=respuestaalpulso(10)

#plot de la salida
plt.close('all')
fig, axs = plt.subplots(3, sharex=True)
axs[0].plot(Variables[0], Variables[1],'.-')
axs[0].set(ylabel = 'Voltaje')
axs[1].plot(Variables[0], Variables[2],'.-')
axs[1].set(ylabel = 'Posición')
v2 = np.diff(Variables[2]) / np.diff(Variables[0]) /4000
axs[2].plot(Variables[0][:-1], v2,'.-')
axs[2].plot(Variables[0], Variables[3],'.-')
axs[2].set(ylabel = 'Velocidad')
plt.legend(('Medida por la PC','Pedida por la placa'))
plt.xlabel('Tiempo [s]')

time.sleep(1)
setVoltageGetData(ser,0)
# Calcula el promedio de la volcidad para cada pulso de voltaje distinto. 
def muchos_pulsos(voltajes):
    vmedios=[]
    
    for voltaje in voltajes: 
        resultados = respuestaalpulso(voltaje)
        vel = resultados[3][30:80]
        prom = sum(vel)/len(vel)
        vmedios.append(prom)
    
    return vmedios

voltajes = np.linspace(-20,20,num=40,endpoint=False,retstep=False, dtype=None )
vmedios = muchos_pulsos(voltajes)
plt.close("all")
plt.plot(voltajes, vmedios,'.-')
plt.show()

medicion= [voltajes,np.array(vmedios)]
medicion=np.vstack((voltajes,np.array(vmedios))).T
np.savetxt('medi_vol_vmid.txt',medicion)
#%%
from scipy.optimize import curve_fit
"""------------------------ Caracterización del motor ------------------------"""
#Quiero calcular la pendiente entre la velocidad y el voltaje. 
"VAMOS HACER UN AJUSTE LINEAL!!!!"

def fit_func(x, m, b):
    return m*x + b

#Seleccionamos un conjunto de puntos donde el gráfico se ve lineal. 
v = voltajes[:]
ve = vmedios[:]

params = curve_fit(fit_func, v, ve)
[m,b] = params[0]

print(m,b)
f=[]
for i in v:
    f.append(m*i + b)
plt.close("all")
plt.plot(v,ve,".-")
plt.plot(v,f,".-")

plt.show()

#%%
"""     Cálculo de la señal de control ( u(t) ) y  de los términos PID
        Proporcional = kp*e
        
        Integral = ki*integral(e)
        
        derivativo = kd*de/dt     """
        
def error(setpoint,variable):
    return (setpoint - variable)
        
def P(kp,error):
    return (kp*error)

def I(ki,error):
    return ki*error

def D(kd,error,error_prev,dt):
    return kd*(error - error_prev)/dt

def señal_control(P,I,D):
    return P+I+D
#%%

def Control_P(kp,setpoint):
  posicion = [] ; velocidad =[] ; tiempo = np.linspace(0,300,num=300)
  i=0 
  pos, vel = setVoltageGetData(ser, 0) #Le mando 0 volts al motor. Entonces pos,vel=0
  posicion.append(pos)
  velocidad.append(vel)
      
  e_0 = error(setpoint,vel) #Error Inicial.
  u = señal_control(P(kp,e_0), 0, 0)
          
  for t in tiempo:
      pos, vel = setVoltageGetData(ser, u)
      posicion.append(pos)
      velocidad.append(vel)
          
      e = error(setpoint, vel)
      u = señal_control(P(kp,e), 0, 0)
      
  plt.plot(tiempo, velocidad[0:300],'--', label = f"Kp={kp}")
  plt.ylabel('Vel', size=15)
  plt.xlabel('Tiempo', size=15)
  plt.grid()
  plt.axhline(y=setpoint, linestyle=':', c='k', label='Setpoint')
  plt.legend(loc=1)
  plt.show()


def control_PID(setpoint,L,R):
  posicion = [] ; velocidad =[] ; tiempo = np.linspace(0,50,num=50)
  kp=1.2/(L*R)
  ki=1.2/(2*R*L**2)
  kd=1.2/(2*R)
  dt =0.1

  #Primer paso
  pos, vel = setVoltageGetData(ser, 0) #Le mando 0 volts al motor. Entonces pos,vel=0
  posicion.append(pos)
  velocidad.append(vel)

  # Calculo error inicial. En este caso el error es +10.
  e_0 = error(setpoint,pos) 

  #Errores para en el termino integral.
  e_int_0 = 0 
  e_int = e_int_0 + e_0

  #Errores para el termino derivativo.
  e_prev_0 = 0 
  e_prev = e_prev_0 + e_0  

  u = señal_control(P(kp,e_0), I(ki,e_int) ,0) #Saque el termino derivativo que estaba puesto, en el instante 0 no se puede derivar.


  for t in tiempo:
      pos, vel = setVoltageGetData(ser, u)
      posicion.append(pos)
      velocidad.append(vel)
      
      e = error(setpoint, pos)
      e_prev = e_prev + e
      e_int = e_int + e
      u = señal_control(P(kp,e),I(ki,e_int) , D(kd,e,e_prev,dt))

  plt.plot(tiempo, posicion, '--', c='tab:red')
  plt.ylabel('Posición', size=15)
  plt.xlabel('Tiempo', size=15)
  plt.axhline(y=setpoint, linestyle=':', c='k', label='Setpoint')
  plt.legend()
  plt.show()
#%%
"""Posicion Vs Tiempo para distintos valores de Kp"""
reset(ser)
plt.figure()
#Control_P(1.25,10)
kp_lista=[0.5,0.8,1,1.5,2,2.2,2.5,3] ; tiempo = np.linspace(0,100,num=100)
setpoint = 10 
i=0 #Variable que voy a usar para crear gráficos.
for kp in kp_lista:
    posicion = [] ; velocidad =[]
    #Primer paso
    pos, vel = setVoltageGetData(ser, 0) #Le mando 0 volts al motor. Entonces pos,vel=0
    posicion.append(pos)
    velocidad.append(vel)
    
    e_0 = error(setpoint,vel) #Error Inicial.
    u = señal_control(P(kp,e_0), 0, 0)
        
    for t in tiempo:
        pos, vel = setVoltageGetData(ser, u)
        posicion.append(pos)
        velocidad.append(vel)
        
        e = error(setpoint, vel)
        u = señal_control(P(kp,e), 0, 0)
    
    plt.plot(tiempo, velocidad[0:100],'--', label = f"Kp={kp}")
    i=i+1
    medicion= [tiempo,np.array(velocidad[0:100])]
    medicion=np.vstack((tiempo,np.array(velocidad[0:100]))).T
    np.savetxt(f'Vel_Tiempo_{kp}.txt',medicion)

    
    reset(ser)
    time.sleep(1)

plt.ylabel('Posición', size=15)
plt.xlabel('Tiempo', size=15)
plt.grid()
plt.axhline(y=setpoint, linestyle=':', c='k', label='Setpoint')
plt.legend(loc=1)
plt.show()

#%%
# =============================================================================
#Hace lo mismo que control_P solo que le agrega el termino integral
"""Posicion Vs Tiempo para distintos valores de Ki"""
reset(ser)
ki_lista=[0.01,0.05,0.08,0.1,0.2,0.3]; tiempo = np.linspace(0,100,num=100)
plt.figure()
kp=1.25 ; setpoint = 10 ; Ti= 1
i=0
for ki in ki_lista:
    posicion = [] ; velocidad =[]
    #Primer paso
    pos, vel = setVoltageGetData(ser, 0) #Le mando 0 volts al motor. Entonces pos,vel=0
    posicion.append(pos)
    velocidad.append(vel)
    
    #Error inical. 
    e_0 = error(setpoint,vel) 
    
    #Errores para el termino integral.
    e_int_0 = 0 
    e_int = (e_int_0 + e_0)*Ti
    
    u = señal_control(P(kp,e_0), I(ki,e_int),0)
        
    for t in tiempo:
        pos, vel = setVoltageGetData(ser, u)
        posicion.append(pos)
        velocidad.append(vel)
        e = error(setpoint, vel)
        e_int = (e_int + e)*Ti
        u = señal_control(P(kp,e), I(ki,e_int), 0)

    plt.plot(tiempo, velocidad[0:100],'--', label = f"Ki={ki}")
    plt.legend(loc=1)
    i=i+1
#    medicion= [tiempo,np.array(velocidad[0:100])]
#    medicion=np.vstack((tiempo,np.array(velocidad[0:100]))).T
#    np.savetxt(f'Vel_Tiempo_kp125_{ki}.txt',medicion)
    reset(ser)
    time.sleep(1)

plt.ylabel('Posición', size=15)
plt.xlabel('Tiempo', size=15)
plt.axhline(y=setpoint, linestyle=':', c='k', label='Setpoint')
plt.legend(loc=1)
plt.show()
#%%
#Hace lo mismo que control_P solo que le agrega el termino derivativo

"""Posicion Vs Tiempo para distintos valores de Kd"""
reset(ser)
kd_lista=[0.1,0.3,0.5,0.8,1,3,10] ;tiempo = np.linspace(0,100,num=100)
plt.figure()
kp=1.25 ; ki = 0 ; setpoint = 10 ; dt =1

for kd in kd_lista:
    posicion = [] ; velocidad =[] 
    #Primer paso
    pos, vel = setVoltageGetData(ser, 0) #Le mando 0 volts al motor. Entonces pos,vel=0
    posicion.append(pos)
    velocidad.append(vel)
    
    #Error incial.
    e_0 = error(setpoint,vel) 
    
    #Errores para el termino derivativo.
    e_prev_0 = 0 
    e_prev = e_prev_0 + e_0  
    
    u = señal_control(P(kp,e_0), 0 ,D(kd,e,e_0,dt))
    
    
    for t in tiempo:
        pos, vel = setVoltageGetData(ser, u)
        posicion.append(pos)
        velocidad.append(vel)
        
        e = error(setpoint, vel)
        e_prev = e_prev - e
        u = señal_control(P(kp,e),0 , D(kd,e,e_prev,dt))
        
    plt.plot(tiempo, velocidad[0:100],'--', label = f"Kd={kd}")
    plt.legend()
#    medicion= [tiempo,np.array(velocidad[0:100])]
#    medicion=np.vstack((tiempo,np.array(velocidad[0:100]))).T
#    np.savetxt(f'Vel_Tiempo_kp125_kd_{kd}.txt',medicion)
    reset(ser)
    time.sleep(1)

#plt.plot(tiempo, velocidad[], '--', c='tab:red')
plt.ylabel('Vel', size=15)
plt.xlabel('Tiempo', size=15)
plt.axhline(y=setpoint, linestyle=':', c='k', label=setpoint)
plt.legend()
plt.show()
#%%
posicion = [] ; velocidad =[] ; tiempo = np.linspace(0,50,num=50)

kp=2 ; ki = 2 ; kd = 2 ; setpoint = 10 ; dt =0.1

#Primer paso
pos, vel = setVoltageGetData(ser, 0) #Le mando 0 volts al motor. Entonces pos,vel=0
posicion.append(pos)
velocidad.append(vel)

# Calculo error inicial. En este caso el error es +10.
e_0 = error(setpoint,pos) 

#Errores para en el termino integral.
e_int_0 = 0 
e_int = e_int_0 + e_0

#Errores para el termino derivativo.
e_prev_0 = 0 
e_prev = e_prev_0 + e_0  

u = señal_control(P(kp,e_0), 0 ,D(kd,e,e_0,dt))


for t in tiempo:
    pos, vel = setVoltageGetData(ser, m*u)
    posicion.append(pos)
    velocidad.append(vel)
    
    e = error(setpoint, pos)
    e_prev = e_prev + e
    e_int = e_int + e
    u = señal_control(P(kp,e),I(ki,e_int) , D(kd,e,e_prev,dt))

plt.plot(tiempo, posicion, '--', c='tab:red')
plt.ylabel('Posición', size=15)
plt.xlabel('Tiempo', size=15)
plt.axhline(y=setpoint, linestyle=':', c='k', label='Setpoint')
plt.legend()
plt.show()
#%%
"""   ------------------------    Método Ziegler Nichols      ------------------------""" 
v_ = [] ; t_= np.linspace(0,50,num =50)
for t in t_:
    pos, vel = setVoltageGetData(ser,6)
    v_.append(vel)


def y(x,m,b):
    return x*m +b

def sigmoid (x, A, h, slope):
    return A / (1 + np.exp ((h - x) / slope))

p, _ = curve_fit(sigmoid, t_, v_)

A = p[0] ; xm = p[1] ;  slope = p[2] 
   
lista=[] ; y_=[]

R=slope
ym=1/(2*A)
L=xm-ym/R-2.5
K_d=1.2/(2*R)



#Escribo Kp, Ki, Kd
K_p=1.2/(L*R)
K_i=1.2/(2*R*L**2)

for t in t_:
    y_.append(y(t,R,-R*L))
    lista.append(sigmoid(t,A,xm,R))

medicion_ZN= [t_,np.array(v_)]
medicion_ZN=np.vstack((t_,np.array(v_))).T

    
#    
#y_2 = lista[4:8]
#
#py, _ = curve_fit(y, t_[4:8], y_2)
# 
#py[0]

plt.close("all")

plt.plot(t_,v_,".-",label="v")
plt.plot(t_,lista,".-",label="sig")
plt.plot(t_, y_,".-",label=f"{L}")

plt.legend()
plt.show()
np.savetxt(f'Vel_Tiempo_ZN_{K_p}{K_i}{K_d}.txt',medicion_ZN)
reset(ser)


#%% """ BARRIDO DE PID"""

"""Posicion Vs Tiempo para distintos valores de Kd"""
reset(ser)
tiempo = np.linspace(0,100,num=100)
plt.figure()
#kp=0.5 ; ki = 0.11 ; kd = 0.1; setpoint = 10 ; dt =10

kp=K_p ; ki = K_i ; kd =K_d; setpoint = 10 ; dt =10
posicion = [] ; velocidad =[] 
#Primer paso
pos, vel = setVoltageGetData(ser, 0) #Le mando 0 volts al motor. Entonces pos,vel=0
posicion.append(pos)
velocidad.append(vel)
    
#Error incial.
e_0 = error(setpoint,vel) 
    
#Errores para el termino integral.
e_int_0 = 0 
e_int = (e_int_0 + e_0)*Ti
    
#Errores para el termino derivativo.
e_prev_0 = 0 
e_prev = e_prev_0 - e_0  
    
u = señal_control(P(kp,e_0), I(ki,e_int) ,D(kd,e,e_0,dt))

for t in tiempo:
    pos, vel = setVoltageGetData(ser, u)
    posicion.append(pos)
    velocidad.append(vel)
        
    e = error(setpoint, vel)
    e_int = e_int + e
    e_prev = e_prev - e
    u = señal_control(P(kp,e), I(ki,e_int),D(kd,e,e_0,dt))
reset(ser)     
   
medicion_pid= [tiempo,np.array(velocidad[0:100])]
medicion_pid=np.vstack((tiempo,np.array(velocidad[0:100]))).T
np.savetxt(f'Vel_Tiempo_PIDzn_{ki}_{kp}_{kd}.txt',medicion_pid)
    
    
plt.plot(tiempo, velocidad[0:100],'--', label = f"=")
#plt.plot(tiempo, velocidad[], '--', c='tab:red')
plt.ylabel('Vel', size=15)
plt.xlabel('Tiempo', size=15)
plt.axhline(y=setpoint, linestyle=':', c='k', label=setpoint)
plt.legend()
plt.show()



#%%
"""   ------------------------    Setpoint fijo      ------------------------""" 
setpoint = [10]
plt.figure()

tiempo = np.linspace(0,2000,num=2000)
for sp in setpoint:
    
#    kp=0.5 ; ki = 0 ; kd = 0 ; dt =10
    
    kp=K_p ; ki = K_i ; kd =K_d ; dt =10
    posicion = [] ; velocidad =[] 
    
    #Primer paso
    pos, vel = setVoltageGetData(ser, 0) #Le mando 0 volts al motor. Entonces pos,vel=0
    posicion.append(pos)
    velocidad.append(vel)
        
    #Error incial.
    e_0 = error(sp,vel) 
        
    #Errores para el termino integral.
    e_int_0 = 0 
    e_int = (e_int_0 + e_0)*Ti
        
    #Errores para el termino derivativo.
    e_prev_0 = 0 
    e_prev = e_prev_0 - e_0  
        
    u = señal_control(P(kp,e_0), I(ki,e_int) ,D(kd,e,e_0,dt))
    
    for t in tiempo:
        pos, vel = setVoltageGetData(ser, u)
        posicion.append(pos)
        velocidad.append(vel)
            
        e = error(sp, vel)
        e_int = e_int + e
        e_prev = e_prev - e
        u = señal_control(P(kp,e), I(ki,e_int),D(kd,e,e_0,dt))
    plt.plot(tiempo, velocidad[0:2000],'--', label = f"{sp}")
    plt.legend()
    reset(ser)
    time.sleep(1)     
        #Control_P(k,sp)

medicion_fijo= [tiempo,np.array(velocidad[0:2000])]
medicion_fijo=np.vstack((tiempo,np.array(velocidad[0:2000]))).T
np.savetxt(f'Vel_Tiempo_Confuerza_externa.txt',medicion_fijo)
plt.plot(tiempo, velocidad[0:2000], '--', c='tab:red')
plt.ylabel('Velocidad', size=15)
plt.xlabel('Tiempo', size=15)
plt.axhline(y=setpoint, linestyle=':', c='k', label=sp)
plt.legend()
plt.show()  
#%%
"""   ------------------------    Setpoint armonico      ------------------------"""
plt.figure()
vel = np.array([5])
tiempo = np.linspace(0,50,num=50)
for v in vel:
        kp=K_p ; ki = K_i ; kd =K_d ; dt =10
        posicion = [] ; velocidad =[] 
        
        #Primer paso
        pos, vel = setVoltageGetData(ser, 0) #Le mando 0 volts al motor. Entonces pos,vel=0
        posicion.append(pos)
        velocidad.append(vel)
        setpoint = v*tiempo[0]
        #Error incial.
        e_0 = error(setpoint,vel) 
            
        #Errores para el termino integral.
        e_int_0 = 0 
        e_int = (e_int_0 + e_0)
            
        #Errores para el termino derivativo.
        e_prev_0 = 0 
        e_prev = e_prev_0 - e_0  
            
        u = señal_control(P(kp,e_0), I(ki,e_int) ,D(kd,e,e_0,dt))
    
        for t in tiempo:
            
            setpoint = v*t 
        #    kp=0.5 ; ki = 0 ; kd = 0 ; dt =10
            
            pos, vel = setVoltageGetData(ser, u)
            posicion.append(pos)
            velocidad.append(vel)
                    
            e = error(setpoint, vel)
            e_int = e_int + e
            e_prev = e_prev - e
            u = señal_control(P(kp,e), I(ki,e_int),D(kd,e,e_0,dt))
            
        plt.plot(tiempo, velocidad[0:50],'--', label = "")
        plt.legend()
              
            #Control_P(k,sp)
        reset(ser)
        time.sleep(1)   
#
#medicion_fijo= [tiempo,np.array(velocidad[0:200])]
#medicion_fijo=np.vstack((tiempo,np.array(velocidad[0:200]))).T
#np.savetxt(f'Vel_Tiempo_Confuerza_externa.txt',medicion_fijo)
plt.plot(tiempo, velocidad[0:50], '--', c='tab:red')
plt.ylabel('Velocidad', size=15)
plt.xlabel('Tiempo', size=15)
plt.axhline(y=setpoint, linestyle=':', c='k', label="")
plt.legend()
plt.show()  