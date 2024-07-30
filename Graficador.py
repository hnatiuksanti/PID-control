"""             ------Graficador de Datos------             """

import os
import serial
import time 
import math
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from scipy.optimize import curve_fit

sns.set_style("dark")
ruta = "C:/Users/PID/mediciones/"

#%%
"""             ------Velocidad Vs Tiempo para MOTOR DC-----          """

doc = ruta +"motor/medi_vol_vmid.txt"
new_doc = ruta + "motor/Vel_vs_Volt_motorDC.png"
x , y = np.loadtxt( doc, skiprows = 0, unpack = True)  

nombre = "Velocidad"

plt.close('all')
plt.plot(x,y, 'o-', ms = 4 ,label = f"{nombre}")
plt.vlines(10, -24, 24, linestyles = 'dashed', colors = "grey" )
plt.vlines(-10, -24, 24, linestyles = 'dashed', colors = "grey" )
plt.text(13, -5, "   Zona\n      de\nsaturación")
plt.text(-18, -5, "   Zona\n      de\nsaturación")

plt.xlabel("Voltaje")
plt.ylabel("Velocidad")
plt.grid()
plt.legend(loc = (0.74,0.78))
# plt.savefig(new_doc,dpi=300)
plt.show()

indicevel = []
for i in range(len(y)): 
    if y[i]<4 and y[i]>-4:
        print(str(y[i])+" con volt = " + str(x[i]))
        indicevel.append(i)
        
        
        
        
        


#%%
"""             ------Velocidad Vs Tiempo para Barrido Kp-----          """
#[0.5,0.8,1,1.5,2,2.2,2.5,3]
new_doc = ruta + "Control_P/Vel_Tiempo_KPs2.png"
for kp in [0.8,1.5,2.2,3]:
    archivo = ruta + f"Control_P/Vel_Tiempo_{kp}.txt"
   
    x , y = np.loadtxt(archivo, skiprows = 0, unpack = True)  
    
    plt.plot(x,y, 'o-', ms = 3 ,label = f"Kp = {kp}")

plt.grid()
plt.hlines(10, 0, 100, linestyles = 'dashed', colors = "grey", label = "Setpoint")
plt.xlabel("Tiempo")
plt.ylabel("Velocidad")
plt.legend(loc=1)
# plt.savefig(new_doc,dpi=300)
plt.show()

#%%
"""             ------Velocidad Vs Tiempo para Barrido Ki-----          """
#[0.08,0.1,0.2,0.3,0.4,0.5,0.6,0.7]
new_doc = ruta + "Control_PI/Vel_Tiempo_KIs.png"
for ki in [0.1,0.2,0.3,0.4]:
    archivo = ruta + f"Control_PI/Vel_Tiempo_kp125_{ki}.txt"
   
    x , y = np.loadtxt(archivo, skiprows = 0, unpack = True)  
    
    plt.plot(x,y, 'o-', ms = 2 ,label = f"Ki = {ki}")


plt.grid()
plt.hlines(10, 0, 100, linestyles = 'dashed', colors = "grey", label = "Setpoint")
plt.text(82.6, 10.2, "Kp = 1.25", size=10,
    bbox=dict(boxstyle="round,pad=0.3", fc = "none", ec="grey"))
# plt.annotate("Kp = 1.25", xy=(42,16.2), args, kwargs)
# plt.plot(0,0,label="Kp=1.25")
plt.xlabel("Tiempo")
plt.ylabel("Velocidad")
plt.legend(loc =(0.50, 0.75), ncol = 2)
# plt.savefig(new_doc,dpi=300)
plt.show()


#%%
"""             ------Velocidad Vs Tiempo para Barrido Kd-----          """
#[0.1,0.3,0.5,1,10]
new_doc = ruta + "Control_PD/Vel_Tiempo_KDs.png"
for kd in [0.1,0.3,0.5]:
    archivo = ruta + f"Control_PD/Vel_Tiempo_kp125_kd_{kd}.txt"
   
    x , y = np.loadtxt(archivo, skiprows = 0, unpack = True)  
    
    plt.plot(x,y, 'o-', ms = 2 ,label = f"Kd = {kd}")

plt.grid()
plt.hlines(10, 0, 100, linestyles = 'dashed', colors = "grey", label = "Setpoint")
plt.text(46, 2.9, "Kp = 1.25", size=11,
    bbox=dict(boxstyle="round,pad=0.3", fc = "none", ec="grey"))
plt.xlabel("Tiempo")
plt.ylabel("Velocidad")
plt.legend(bbox_to_anchor =(0.99, 1.12), ncol = 4)
plt.savefig(new_doc,dpi=300)
plt.show()
#%%
"""             ------Velocidad Vs Tiempo para Ziegler-Nichols-----          """
#Vel_Tiempo_ZN_0.33028260812879760.093245724500802320.2924707857018749
kp = 0.3302826081287976 ; ki = 0.09324572450080232 ; 0.2924707857018749
new_doc = ruta + "Zn/Vel_Tiempo_ZN.png"  

archivo = ruta + f"Zn/Vel_Tiempo_ZN.txt"
x , y = np.loadtxt(archivo, skiprows = 0, unpack = True) 

def sigmoid (x, A, h, slope):
    return 1 / (1 + np.exp ((h - x) / slope)) *A

p, _ = curve_fit(sigmoid, x, y)

A = p[0] ; xmedio = p[1] ;  slope = p[2]  
f_ = []
for i in x: 
    f_.append(sigmoid(i, A, xmedio, slope))

plt.plot(x,y, 'o-', ms = 2 ,label = "Velocidad")
plt.plot(x, f_, "--",label="Ajuste")
plt.grid()
# plt.hlines(10, 0, 100, linestyles = 'dashed', colors = "grey", label = "Setpoint")

# plt.text(44.5, 6, "Kp = 0.33", size=10,
#     bbox=dict(boxstyle="round,pad=0.3", fc = "none", ec="grey"))
# plt.text(44.5, 4.5, "Ki = 0.09", size=10,
#     bbox=dict(boxstyle="round,pad=0.3", fc = "none", ec="grey"))
# plt.text(44.5, 3, "Kd = 0.29", size=10,
#     bbox=dict(boxstyle="round,pad=0.3", fc = "none", ec="grey"))

plt.xlabel("Tiempo")
plt.ylabel("Velocidad")
plt.xlim([0,55])
plt.legend(loc =(0.75,0.65), ncol = 1)
plt.savefig(new_doc,dpi=300)
plt.show()

#%%
"""         ------Velocidad Vs Tiempo para  PID sin el termino D-----          """

new_doc = ruta + "Control_PID_sin_D/Vel_Tiempo_PID_sin_D.png"

archivo = ruta + f"Control_PID_sin_D/Vel_Tiempo_PID_0.11_0.5_0.txt"
   
x , y = np.loadtxt(archivo, skiprows = 0, unpack = True)  
    
plt.plot(x,y, 'o-', ms = 2 ,label = "Velocidad")

plt.grid()
plt.hlines(10, 0, 100, linestyles = 'dashed', colors = "grey", label = "Setpoint")
# plt.text(46, 2.9, "Kp = 1.25", size=11,
#     bbox=dict(boxstyle="round,pad=0.3", fc = "none", ec="grey"))
plt.xlabel("Tiempo")
plt.ylabel("Velocidad")
plt.legend(loc = (0.7,0.7), ncol = 1)
# plt.savefig(new_doc,dpi=300)
plt.show()

#%%
"""         ------Velocidad Vs Tiempo para  PID -----          """

new_doc = ruta + "Control_PID/Vel_Tiempo_PID.png"

archivo = ruta + f"Control_PID/Vel_Tiempo_PID_0.11_0.5_0.1.txt"
   
x , y = np.loadtxt(archivo, skiprows = 0, unpack = True)  
    
plt.plot(x,y, 'o-', ms = 2 ,label = "Velocidad")

plt.grid()
plt.hlines(10, 0, 100, linestyles = 'dashed', colors = "grey", label = "Setpoint")
# plt.text(46, 2.9, "Kp = 1.25", size=11,
#     bbox=dict(boxstyle="round,pad=0.3", fc = "none", ec="grey"))
plt.xlabel("Tiempo")
plt.ylabel("Velocidad")
plt.legend(loc = (0.7,0.7), ncol = 1)
plt.savefig(new_doc,dpi=300)
plt.show()

#%%
"""      ------Velocidad Vs Tiempo para  PID con los kp,ki,kd de ZN -----          """
#0.09324572450080232_ 0.3302826081287976_ 0.2924707857018749
new_doc = ruta + "PID+ZN/Vel_Tiempo_PID_ZN.png"

archivo = ruta + "PID+ZN/Vel_Tiempo_PID_ZN.txt"
   
x , y = np.loadtxt(archivo, skiprows = 0, unpack = True)  
    
plt.plot(x,y, 'o-', ms = 2 ,label = "Velocidad")

plt.grid()
plt.hlines(10, 0, 100, linestyles = 'dashed', colors = "grey", label = "Setpoint")
# plt.text(46, 2.9, "Kp = 1.25", size=11,
#     bbox=dict(boxstyle="round,pad=0.3", fc = "none", ec="grey"))
plt.xlabel("Tiempo")
plt.ylabel("Velocidad")
plt.legend(loc = (0.7,0.7), ncol = 1)
plt.savefig(new_doc,dpi=300)
plt.show()

#%%
"""      ------Velocidad Vs Tiempo para  Setpoint fijo -----          """

new_doc = ruta + "SetpointFijo/Vel_Tiempo_Confuerza_externa.png"

archivo = ruta + "SetpointFijo/Vel_Tiempo_Confuerza_externa.txt"
   
x , y = np.loadtxt(archivo, skiprows = 0, unpack = True)  
    
plt.plot(x,y, '-' ,label = "Velocidad")

plt.grid()
plt.hlines(10, 0, 100, linestyles = 'dashed', colors = "grey", label = "Setpoint")
# plt.text(46, 2.9, "Kp = 1.25", size=11,
#     bbox=dict(boxstyle="round,pad=0.3", fc = "none", ec="grey"))
plt.xlim([0,1500])
plt.xlabel("Tiempo")
plt.ylabel("Velocidad")
plt.legend(loc = (0.7,0.7), ncol = 1)
# plt.savefig(new_doc,dpi=300)
plt.show()

#%%
"""      ------Velocidad Vs Tiempo para Fuerza Herni -----          """

new_doc = ruta + "FuerzaHerni/Vel_Tiempo_Confuerza_externa.png"

archivo = ruta + "FuerzaHerni/Vel_Tiempo_Confuerza_externa.txt"
   
x , y = np.loadtxt(archivo, skiprows = 0, unpack = True)  
    
plt.plot(x,y, '-' ,label = "Velocidad")

plt.grid()
plt.hlines(10, 0, 750, linestyles = 'dashed', colors = "grey", label = "Setpoint")
# plt.text(46, 2.9, "Kp = 1.25", size=11,
#     bbox=dict(boxstyle="round,pad=0.3", fc = "none", ec="grey"))
plt.xlim([0,750])
plt.xlabel("Tiempo")
plt.ylabel("Velocidad")
plt.legend(loc = 0, ncol = 1)
plt.savefig(new_doc,dpi=300)
plt.show()

#%% 
"""     Comparacion entre PID y PID con parametros de ZN    """

archivo_pid = ruta + f"Control_PID/Vel_Tiempo_PID_0.11_0.5_0.1.txt"
archivo_zn = ruta + "PID+ZN/Vel_Tiempo_PID_ZN.txt"
new_doc = ruta + "PID+ZN/Comparacion_PID_ZN.png"
pidx , pidy = np.loadtxt(archivo_pid, skiprows = 0, unpack = True)
znx , zny = np.loadtxt(archivo_zn, skiprows = 0, unpack = True)
diff = [] 
err_pid = []
err_zn = []

div_ = []

for i in range(len(pidx)): 
    diff.append(abs(pidy[i]-zny[i]))
    err_pid.append(abs(10-pidy[i]))
    err_zn.append(abs(10-zny[i]))
    div_.append(abs(zny[i])/abs(pidy[i]))
    
    
plt.close("all")
plt.grid()
# plt.plot(znx , zny, "o-", ms = 1,label = "PID con ZN")
# plt.plot(pidx , pidy, "o-",  ms = 1,label = "PID")
plt.plot(pidx,div_,"-", label= "|ZN| / |PID|")
# plt.plot(pidx , err_pid, "o-",  ms = 1,label = "Error PID")
# plt.plot(znx , err_zn, "o-", ms = 1,label = "Error ZN")

plt.hlines(1, 0, 100, linestyles = 'dashed', colors = "grey")
plt.legend(loc=(0.4,0.7))
plt.xlabel("Tiempo")
plt.ylabel("|ZN| / |PID|")
# plt.savefig(new_doc,dpi=300)
plt.show()





