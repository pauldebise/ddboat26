import numpy as np
import matplotlib.pyplot as plt
from ddboatlib import init_figure,clear,draw_ddboat
import math


def f(x,u):
    u1,u2=list(u[0:2,0])
    θ=x[2,0]
    dx=1
    dy=2
    dθ=0
    xdot=np.array([[dx],[dy],[dθ]])
    return xdot



ax=init_figure(-1,13,-1,21)
x0,y0,θ0=0,0,0
x = np.array([[x0,y0,θ0]]).T
dt = 0.1
trajectory = []
for t in np.arange(0,2,dt):
    clear(ax)
    u=np.array([[4],[5]])
    #x = x + dt*f(x+(dt/2)*f(x,u),u)  #Runge Kutta
    x=x+dt*f(x,u) #Euler
    mx,my,θ=list(x[0:3,0])
    trajectory.append((mx,my))
    u1,u2=list(u[0:2,0])
    draw_ddboat(ax,mx,my,θ,u1,u2)
    plt.pause(0.001)
plt.pause(1)




