import numpy as np
import math
import matplotlib.pyplot as plt

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



def simu(x0,y0,θ0,vx0,vy0,w0,w10,w20,u1,u2,p1,p2,p3,p4,p5,p6,p7):
    L = []
    x = np.array([[x0,y0,θ0,vx0,vy0,w0,w10,w20]]).T
    dt = 0.01
    for t in np.arange(0,2,dt):
        u=np.array([[u1],[u2]])
        x=x+dt*f(x,u)
        mx,my,θ,vx,vy,w,w1,w2=list(x[0:8,0])
        L.append((t,mx,my,θ,vx,vy,w,w1,w2))
    return(L)


p1,p2,p3,p4,p5,p6,p7=1,1,1,1,1,1,1
u1,u2=5,5
print("p1,p2,p3,p4,p5,p6,p7",p1,p2,p3,p4,p5,p6,p7)

x0,y0,θ0,vx0,vy0,w0,w10,w20=0,3,0,0,0,0,0,0
L=simu(x0,y0,θ0,vx0,vy0,w0,w10,w20,u1,u2,p1,p2,p3,p4,p5,p6,p7) # (t,mx,my,θ,vx,vy,w,w1,w2)

plt.figure()
plt.plot([a[0] for a in L], [a[7] for a in L], label='$w1(t)$')
plt.pause(10)
