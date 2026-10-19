#Generate the date either from simulation or using a ddboat

import sys,time,csv
import numpy as np
from scipy.linalg import inv
from ddboatlib import drawEllipsoid
import matplotlib.pyplot as plt


def generate_data_with_simu(tmax,dt):
    β=46  # nominal value of the mag field [Testa] used for generated with simulation
          # The whole calibration process should not be sensitive to this value
    A=β*np.array([[4,1,2],[1,3,1],[2,1,2]])
    xbar=np.array([[1],[2],[3]])

    def generate_cloud_with_simu(tmax,dt):
        n = int(tmax/dt)
        data = np.zeros((n,3))  # mx, my, mz
        t0 = time.time()
        for i in range(n):
            t=time.time()-t0
            lx,ly=10*t,np.cos(3*t) #random latlong trajectory on the sphere
            y = np.array([[np.cos(ly)*np.cos(lx)],[np.cos(ly)*np.sin(lx)],[np.sin(ly)]])
            x=A@y+xbar
            data[i,:] = [x[0,0],x[1,0],x[2,0]]
            time.sleep(dt)
        with open("magcloud.csv", "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerows(data)

        ax.scatter(*data.T,c="blue")
        plt.pause(0.1)
        Q=inv(A)@inv(A)
        drawEllipsoid(ax,xbar,Q) #(x-xbar)^T * Q * (x-xbar) = 1
        ax.scatter(*xbar[:,0],c="magenta")

        plt.pause(0.1)

    def generate_mag_north_west_with_simu():
        I=64*3.14/180
        ynorth=np.array([[np.cos(I)],[0],[-np.sin(I)]])
        ywest=np.array([[0],[-np.cos(I)],[-np.sin(I)]])
        print("ynorth",ynorth)
        print("ywest",ywest)
        xnorth=A@ynorth+xbar
        xwest=A@ywest+xbar
        print("xnorth",xnorth)
        print("xwest",xwest)
        ax.scatter(*xnorth[:,0],c="red")
        ax.scatter(*xwest[:,0],c="red")
        with open("magNorthWest.csv", "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerows(xnorth)
            writer.writerows(xwest)

    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')
    generate_cloud_with_simu(tmax,dt)
    generate_mag_north_west_with_simu()
    plt.show()



def generate_data_with_ddboat(tmax,dt):
    sys.path.append('../drivers-ddboat-v2')
    from imu9_driver_v2 import Imu9IO
    imu = Imu9IO()

    def generate_cloud_with_ddboat(tmax,dt):
        n = int(tmax/dt)
        data = np.zeros((n,3))  # mx, my, mz
        t0 = time.time()
        for i in range(n):
            t=time.time()-t0
            x = np.array([imu.read_mag_raw()]).T  # To be used on the dd_boat
            data[i,:] = [x[0,0],x[1,0],x[2,0]]
            time.sleep(dt)
        with open("magdata.csv", "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerows(data)


tmax,dt=2,0.01  # On the dd_boat tmax=5, dt=1/50[sec]
generate_data_with_simu(tmax,dt)
#generate_data_with_ddboat()

