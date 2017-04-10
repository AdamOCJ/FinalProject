from __future__ import division,print_function
from numpy import pi, arange
from math import sqrt, sin, cos
import matplotlib.pyplot as plt

m = 0.145
C = 0.5
rho = 1.225
A = pi*(.0075/2)**2
D = (rho*C*A)/2
B = 4.1 * 10**-4
omega = 1800*2*pi/60
phi = 225*pi/180

v0 = 40.2
angle = 1*pi/180
h = .0001

xvals = []    
yvals = []
zvals = []
    
def forcex(vx,vy,vz):
    V = sqrt(vx**2 + vy**2 + vz**2)
    dragx = -1*(D/m)*V*vx
    magnusx = B*omega*(vz*sin(phi)-vy*cos(phi))
    ax = dragx + magnusx
    return ax
        
def forcey(vx,vy,vz):
    V = sqrt(vx**2 + vy**2 + vz**2)
    dragy = -1*(D/m)*V*vy
    magnusy = B*omega*vx*cos(phi)
    ay = dragy + magnusy
    return ay

def forcez(vx,vy,vz):
    V = sqrt(vx**2 + vy**2 + vz**2)
    dragz = -1*(D/m)*V*vz
    gravity = -9.8
    magnusz = B*omega*vx*sin(phi)
    az = dragz + gravity + magnusz
    return az

def RK4(x,y,z,vx,vy,vz):
    kx1 = h*forcex(vx,vy,vz)
    lx1 = h*vx
    
    ky1 = h*forcey(vx,vy,vz)
    ly1 = h*vy
    
    kz1 = h*forcez(vx,vy,vz)
    lz1 = h*vz
    
    kx2 = h*forcex(vx + .5*kx1,vy + .5*ky1,vz + .5*kz1)
    lx2 = h*(vx + .5*kx1)
    
    ky2 = h*forcey(vx + .5*kx1,vy + .5*ky1,vz + .5*kz1)
    ly2 = h*(vy + .5*ky1)
    
    kz2 = h*forcez(vx + .5*kx1,vy + .5*ky1,vz + .5*kz1)
    lz2 = h*(vz + .5*kz1)
    
    kx3 = h*forcex(vx + .5*kx2,vy + .5*ky2,vz + .5*kz2)
    lx3 = h*(vx + .5*kx2)
    
    ky3 = h*forcey(vx + .5*kx2,vy + .5*ky2,vz + .5*kz2)
    ly3 = h*(vy + .5*ky2)
    
    kz3 = h*forcez(vx + .5*kx2,vy + .5*ky2,vz + .5*kz2)
    lz3 = h*(vz + .5*kz2)
    
    kx4 = h*forcex(vx + kx3, vy + ky3,vz + kz3)
    lx4 = h*(vx + kx3)
    
    ky4 = h*forcey(vx + kx3, vy + ky3,vz + kz3)
    ly4 = h*(vy + ky3)
    
    kz4 = h*forcez(vx + kx3, vy + ky3,vz + kz3)
    lz4 = h*(vz + kz3)
    
    vx = vx + (1.0/6)*(kx1 + 2*kx2 + 2*kx3 +kx4)
    x = x + (1.0/6)*(lx1 + 2*lx2 + 2*lx3 +lx4)
    
    vy = vy + (1.0/6)*(ky1 + 2*ky2 + 2*ky3 +ky4)
    y = y + (1.0/6)*(ly1 + 2*ly2 + 2*ly3 +ly4)
    
    vz = vz + (1.0/6)*(kz1 + 2*kz2 + 2*kz3 +kz4)
    z = z + (1.0/6)*(lz1 + 2*lz2 + 2*lz3 +lz4)
    return[x,y,z,vx,vy,vz]   

x = 0.0
y = 0.0
z = 0.0
vx = v0*cos(angle)
vy = 0
vz = v0*sin(angle)

while x < 18.4404 :
    result = RK4(x,y,z,vx,vy,vz)
    x = result[0]
    y = result[1]
    z = result[2]
    vx = result[3]
    vy = result[4]
    vz = result[5]
    xvals.append(x)
    yvals.append(y)
    zvals.append(z)

plt.plot(xvals,zvals, color = 'k')
plt.plot(xvals,yvals, color = 'r')
