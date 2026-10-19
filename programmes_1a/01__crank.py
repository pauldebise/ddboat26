import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt
from ddboatlib import init_figure,clear,draw_disk
from numpy.linalg import det, norm
from matplotlib.pyplot import plot


def draw_crank(x):
    θ1=x[0,0]
    θ2=x[1,0]
    z=L1*np.array([[np.cos(θ1)],[np.sin(θ1)]])
    y=z+L2*np.array([[np.cos(θ1+θ2)],[np.sin(θ1+θ2)]])
    plot( [0,z[0,0],y[0,0]],[0,z[1,0],y[1,0]],'magenta', linewidth = 2)
    draw_disk(ax,c,r,"cyan")


L1,L2 = 4,3
c = np.array([[1],[2]])
r = 4
dt = 0.05
x = np.array([[-1],[1]])
ax=init_figure(-4,8,-4,8)
for t in np.arange(0,3,dt) :
    clear(ax)
    θ1,θ2=x[0,0],x[1,0]
    w = c + r*np.array([[np.cos(t)], [np.sin(t)]])
    u = np.array([[1],[2]])
    draw_crank(x)
    draw_disk(ax,w,0.2,"blue")
    x = x + dt*u

