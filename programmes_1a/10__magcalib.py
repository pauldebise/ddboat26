import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.linalg import sqrtm,inv,eig, norm
from ddboatlib import drawEllipsoid


def Fit_Ellipsoid(x1,x2,x3):
    x1,x2,x3=np.array(x1),np.array(x2),np.array(x3)
    X = np.column_stack([x1**2,x2**2,x3**2,x1*x2,x1*x3,x2*x3,x1,x2,x3])
    p = inv(X.T@X)@X.T @ np.ones((len(x1), 1))
    p=p.flatten()
    P = np.array([[p[0],1/2*p[3],1/2*p[4]],[1/2*p[3], p[1],1/2*p[5]],[1/2*p[4], 1/2*p[5], p[2]]])
    c=np.array([[p[6]],[p[7]],[p[8]]])
    xbar=-1/2*inv(P)@c
    Q = P/(1+xbar.T@P@xbar)
    print("The matrix Q should be symmetric: \n Q=",Q)
    print("The eigen values of Q should be positive : \n eig(Q)=",eig(Q)[0])
    return Q,xbar



def mag_calib(x1,x2,x3,xnorth,xwest):
    Q,xbar=Fit_Ellipsoid(x1,x2,x3)
    I=64*3.14/180
    ynorth=np.array([[np.cos(I)],[0],[-np.sin(I)]]) # What I want when the boat points to North
    ywest=np.array([[0],[-np.cos(I)],[-np.sin(I)]]) # What I want when the boat points to West
    yup=np.cross(ynorth.flatten(), ywest.flatten()).reshape(3,1)
    Y=np.hstack([ynorth,ywest,yup])
    vnorth=sqrtm(Q)@(xnorth-xbar)
    vwest=sqrtm(Q)@(xwest-xbar)
    print("Should be equal to 1 --> norm(vnorth),norm(vwest)=",norm(vnorth),norm(vwest))
    vup=np.cross(vnorth.flatten(),vwest.flatten()).reshape(3,1)
    V=np.hstack([vnorth,vwest,vup])
    R=Y@inv(V)
    print("Should be a rotation -> R=",R)
    print("R@R.T",R@R.T)
    invA=R@sqrtm(Q)
    return invA,xbar



import numpy as np

x1,x2,x3 = np.loadtxt('magcloud.csv', delimiter=',', unpack=True)
fig = plt.figure(figsize=(8,8))
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x1,x2,x3,c="blue")
plt.pause(0.1)
data = np.loadtxt('magNorthWest.csv') # measured mag when the boat points to North and West
xnorth = data[:3].reshape(3,1)
xwest = data[3:].reshape(3,1)
print("xnorth:", xnorth)
print("xwest:", xwest)
ax.scatter(*xnorth[:,0],c="red")
ax.scatter(*xwest[:,0],c="red")

invA,xbar=mag_calib(x1,x2,x3,xnorth,xwest)
print("Ahat= ",inv(invA))
print("xbar=",xbar)
drawEllipsoid(ax,xbar,invA.T@invA)

plt.show()


