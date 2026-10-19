import numpy as np
import matplotlib.pyplot as plt
from ddboatlib import init_figure,clear,draw_ddboat
from numpy.linalg import det,inv
from matplotlib.pyplot import plot


def f(x,u):
    xr,yr,θr,vr=x.flatten()
    u1,u2=u.flatten()
    return np.array([[vr*np.cos(θr)],[vr*np.sin(θr)],[u1],[u2]])

def control (x):
    return np.array([[0],[0]])

ax=init_figure(-1,13,-1,21)
dt = 0.1
xa=np.array([[8],[8],[1],[1]])
xb=np.array([[5],[5],[1],[1]])
xc=np.array([[2],[2],[1],[1]])
Lx,Ly=3,5
s = np.arange(0,2*np.pi,0.01)

for t in np.arange(0,20,dt) :
    clear(ax)
    plot(Lx*np.cos(s)+6, Ly*np.sin(s)+10)
    ua=control(xa)
    ub=control(xb)
    uc=control(xc)
    draw_ddboat(ax,xa[0,0],xa[1,0],xa[2,0],0,0,True,"red")
    draw_ddboat(ax,xb[0,0],xb[1,0],xb[2,0],0,0,False,"green")
    draw_ddboat(ax,xc[0,0],xc[1,0],xc[2,0],0,0,False,"blue")
    xa = xa + dt*f(xa,ua)
    xb = xb + dt*f(xb,ub)
    xc = xc + dt*f(xc,uc)




"""
x0,y0,θ0=5,5,0
x = np.array([[x0,y0,θ0]]).T
dt = 0.1
i=0
for t in np.arange(0,100,dt):
    clear(ax)
    u=control(x,a,b)
    x=x+dt*f(x,u)
    draw_ddboat(ax,x[0,0],x[1,0],x[2,0],1-u,1+u)
    plot([a[0],b[0]],[a[1],b[1]],'red',1)
    plot(a[0],a[1],'go')
    plt.pause(0.001)
plt.pause(1)

"""



