import numpy as np
import math
import matplotlib.pyplot as plt
from ddboatlib import pool_to_latlong,init_figure,draw_polygon,clear,draw_ddboat,ψ0

def f(x,u):
    x,u=x.flatten(),u.flatten()
    θ,vx,vy,w,w1,w2=x[2],x[3],x[4],x[5],x[6],x[7]
    dx=2
    dy=4
    dθ=3
    dvx=0
    dvy=0
    dw=0
    dw1=0
    dw2=0
    xdot=np.array([ [dx],[dy],[dθ],[dvx],[dvy],[dw],[dw1],[dw2]])
    return xdot



p1,p2,p3,p4,p5,p6,p7=0.07,2200,3.e-05,15.e-05,0.4,5.0,5.0


ax=init_figure(-1,13,-1,21)
x0,y0,θ0,vx0,vy0,w0,w10,w20=3,3,1,10,0,0,1,1
x = np.array([[x0,y0,θ0,vx0,vy0,w0,w10,w20]]).T
dt = 0.01
for t in np.arange(0,5,dt):
    clear(ax)
    u=np.array([[9],[10]])
    x = x + dt*f(x,u) #Euler
    mx,my,θ,vx,vy,w,w1,w2=list(x[0:8,0])
    draw_ddboat(ax,mx,my,θ,w1,w2)
    plt.pause(0.001)
plt.pause(10)

