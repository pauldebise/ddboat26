
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure,pause,cla
from matplotlib.cbook import flatten
from matplotlib.patches import Polygon,Ellipse

#from matplotlib.patches import ,Rectangle,Circle, Wedge, Polygon, Arc

from matplotlib.collections import PatchCollection
import math

# αβδεθλΛμρτφψωΓ

ψ0=math.radians(356.12-360)  # orientation of the pool
long0,lat0=-0.07800405,0.84507466 # reference point of the pool [rad]
R0 = 6378137.0  # radius of Earth [meters]



def clear(ax):
    pause(0.001)
    cla()
    ax.set_xlim(ax.xmin,ax.xmax)
    ax.set_ylim(ax.ymin,ax.ymax)

def init_figure(xmin,xmax,ymin,ymax):
    fig = figure()
    ax = fig.add_subplot(111, aspect='equal')
    ax.xmin=xmin
    ax.xmax=xmax
    ax.ymin=ymin
    ax.ymax=ymax
    clear(ax)
    return ax



def draw_polygon(ax,P,col,alpha0=0.4):
    patches = []
    patches.append(Polygon(P,closed=True, edgecolor='black', facecolor='skyblue'))
    p = PatchCollection(patches, cmap=plt.cm.jet, alpha=alpha0, color=col)
    ax.add_collection(p)


def drawEllipsoid(ax,c,Q,β=1): #(x-c)^T * Q * (x-c) = β^2
    Q=((1/β)**2)*Q
    eigvals, eigvecs = np.linalg.eigh(Q)
    radii = np.sqrt(eigvals)
    radii = 1./radii
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    u, v = np.meshgrid(u, v)
    x = np.cos(u) * np.sin(v)
    y = np.sin(u) * np.sin(v)
    z = np.cos(v)
    sphere = np.stack((x, y, z), axis=0)
    E = (eigvecs @ np.diag(radii) @ sphere.reshape(3, -1)).reshape(3, *x.shape)
    ax.plot_surface(E[0]+c[0],E[1]+c[1],E[2]+c[2],color='steelblue',alpha=0.2,linewidth=0)

def draw_disk(ax,c,r,col,alph=0.7,w=1):
    #draw_disk(ax,[1,2],0.5,"blue")
    cx,cy=c[0,0],c[1,0]
    e = Ellipse(xy=np.array([[cx],[cy]]), width=2*r, height=2*r, angle=0,linewidth = w)
    ax.add_artist(e)
    e.set_clip_box(ax.bbox)
    e.set_alpha(alph)  # transparency
    e.set_facecolor(col)



def draw_ddboat(ax,mx,my,ψ,w1,w2,drawpool=True,color="blue"):
    ech=0.2
    if drawpool:
        Pool=np.array([[12,0],[12,20],[0,20],[0,0],[12,0]])
        draw_polygon(ax,Pool,"cyan")
    P=ech*np.array([[-1,5,7,7,5,-1,-1,-1],[-2,-2,-1,1,2,2,-2,-2]])
    P1=ech*np.array([[-w1-1,-1,-1],[2,2,1]])
    P2=ech*np.array([[-w2-1,-1,-1],[-2,-2,-1]])
    R=np.array([[np.cos(ψ),-np.sin(ψ)],[np.sin(ψ),np.cos(ψ)]])
    P=R@P+np.array([[mx],[my]])@np.array([[1,1,1,1,1,1,1,1]])
    P1=R@P1+np.array([[mx],[my]])@np.array([[1,1,1]])
    P2=R@P2+np.array([[mx],[my]])@np.array([[1,1,1]])
    draw_polygon(ax,P.T,color,1)
    draw_polygon(ax,P1.T,"red",1)
    draw_polygon(ax,P2.T,"red",1)


def pool_to_latlong(x,y):
    x1=np.cos(ψ0)*x+np.sin(ψ0)*y
    y1=-np.sin(ψ0)*x+np.cos(ψ0)*y
    lat=lat0+y1/R0
    long=long0+x1/(R0*np.cos(lat0))
    return lat,long


def latlong_to_pool(lat,long):
    x1=R0*np.cos(lat0)*(long-long0)
    y1=R0*(lat-lat0)
    x=np.cos(ψ0)*x1-np.sin(ψ0)*y1
    y=np.sin(ψ0)*x1+np.cos(ψ0)*y1
    return x,y

def sawtooth(x): return (x+np.pi)%(2*np.pi)-np.pi


if __name__ == "__main__":
    center = np.array([1.0, 2.0, 3.0])
    M = np.array([
        [4.0, 1.2, 0.5],
        [1.2, 2.0, 0.3],
        [0.5, 0.3, 1.0]
    ])
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    drawEllipsoid(ax,center,M)
    ax.scatter(*center, color='red', s=50)

    ax=init_figure(-1,13,-1,21)
    Pool=np.array([[12,0],[12,20],[0,20],[0,0],[12,0]])
    draw_polygon(ax,Pool,"cyan")

    plt.show()
