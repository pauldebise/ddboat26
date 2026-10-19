import numpy as np
import matplotlib.pyplot as plt
from ddboatlib import init_figure,clear,draw_ddboat,sawtooth
from numpy.linalg import det, norm
from matplotlib.pyplot import plot

def control(x,a,b):
    u=1
    return u

def f(x,u):
    θ=x[2,0]
    xdot=np.array([[np.cos(θ)], [np.sin(θ)],[u]])
    return xdot

ax=init_figure(-1,13,-1,21)
a = [(8,4),(8,16),(4,16),(4,4)]
b = [(8,16),(4,16),(4,4),(8,4)]
x = np.array([[5,5,0]]).T
dt = 0.1
i=0
for t in np.arange(0,10,dt):
    clear(ax)
    u=control(x,a,b)
    x=x+dt*f(x,u)
    draw_ddboat(ax,x[0,0],x[1,0],x[2,0],1-u,1+u)
    plot([a[0][0],b[0][0]],[a[0][1],b[0][1]],'red',1)
    plot(a[0][0],a[0][1],'go')
    plot(b[0][0],b[0][1],'ro')
    plt.pause(0.001)
plt.pause(1)
