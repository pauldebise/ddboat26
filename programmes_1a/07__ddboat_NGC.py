from ddboatlib import init_figure,clear,latlong_to_pool,pool_to_latlong,draw_ddboat,ψ0,sawtooth
import numpy as np
import matplotlib.pyplot as plt
import math
from matplotlib.pyplot import plot


def f(x,u):
    x,u=x.flatten(),u.flatten()
    θ,vx,vy,w,w1,w2=x[2],x[3],x[4],x[5],x[6],x[7]
    dx=vx*np.cos(θ)-vy*np.sin(θ)
    dy=vx*np.sin(θ)+vy*np.cos(θ)
    dθ=w
    dvx=w*vy-p5*vx*abs(vx)+p3*(w1*abs(w1)+w2*abs(w2))
    dvy=-w*vx-p6*vy*abs(vy)
    dw=-p7*w*abs(w) +p4*(w2*abs(w2)-w1*abs(w1))
    dw1=-p1*w1*abs(w1)+p2*u[0]
    dw2=-p1*w2*abs(w2)+p2*u[1]
    xdot=np.array([ [dx],[dy],[dθ],[dvx],[dvy],[dw],[dw1],[dw2]])
    return xdot

def g(x):
    mx,my,θ,vx,vy,w,w1,w2=list(x[0:8,0])
    I=math.radians(64)
    lx,ly=pool_to_latlong(mx,my)
    m1=np.sin(θ+ψ0)*np.cos(I)
    m2=np.cos(θ+ψ0)*np.cos(I)
    m3=-np.sin(I)
    return(np.array([[lx],[ly],[m1],[m2],[m3],[w]]))



p1,p2,p3,p4,p5,p6,p7=0.07,220,3.e-05,15.e-05,0.4,5.0,5.0

def control_NGC(y):
    # Navigation block
    lx,ly,m1,m2,m3,w=list(y[0:6,0])
    θ=np.arctan2(m1,m2)-ψ0
    x,y=latlong_to_pool(lx,ly)
    u=np.array([[2],[4]])
    return u

ax=init_figure(-1,13,-1,21)
x0,y0,θ0,vx0,vy0,w0,w10,w20=3,3,1,0,0,0,1,1
x = np.array([[x0,y0,θ0,vx0,vy0,w0,w10,w20]]).T
dt = 0.02
for t in np.arange(0,5,dt):
    clear(ax)
    y=g(x)
    u=control_NGC(y)
    x=x+dt*f(x,u)
    mx,my,θ,vx,vy,w,w1,w2=list(x[0:8,0])
    draw_ddboat(ax,mx,my,θ,0.02*w1,0.02*w2)
    plt.pause(0.001)
plt.pause(10)
