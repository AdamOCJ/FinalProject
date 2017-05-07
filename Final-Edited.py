from __future__ import division,print_function
from numpy import pi, arange
from math import sqrt, sin, cos, pi
import matplotlib.pyplot as plt
from visual import *
from visual.graph import *
import wx

#Constants for baseball and environment like mass of ball, air density, smoothness of ball etc.
m = 0.145
C = 0.5
rho = 1.225
A = pi*(.0075/2)**2
D = (rho*C*A)/2
B = 4.1 * 10**-4
h = .0001

#Initial Conditions
omega = 1800*2*pi/60
phi = 225*pi/180
v0 = 90.0 * .44704
angle = -1*pi/180
#For graphing coordinates
xvals = []    
yvals = []
zvals = []

#Force equations for three dimensions
def forcex((vx,vy,vz),w,p):
    V = sqrt(vx**2 + vy**2 + vz**2)
    dragx = -1*(D/m)*V*vx
    magnusx = B*w*(vz*sin(p)-vy*cos(p))
    ax = dragx + magnusx
    return ax
        
def forcey((vx,vy,vz),w,p):
    V = sqrt(vx**2 + vy**2 + vz**2)
    dragy = -1*(D/m)*V*vy
    magnusy = B*w*vx*cos(p)
    lateraly = 0.5*(sin(4*p)-0.25*sin(8*p)+0.08*sin(12*p)-0.025*sin(16*p))
    ay = dragy + magnusy + 9.8*lateraly
    return ay

def forcez((vx,vy,vz),w,p):
    V = sqrt(vx**2 + vy**2 + vz**2)
    dragz = -1*(D/m)*V*vz
    gravity = -9.8
    magnusz = -1*B*w*vx*sin(p)
    az = dragz + gravity + magnusz
    return az

#RK4 for flight of ball, inputs are postion, translational velocity, angular velocity and orientation
def RK4((x,z,y),(vx,vz,vy),w,p):
    
    y = -1*y
    vy = -1*vy
    kx1 = h*forcex((vx,vy,vz),w,p)
    lx1 = h*vx
    
    ky1 = h*forcey((vx,vy,vz),w,p)
    ly1 = h*vy
    
    kz1 = h*forcez((vx,vy,vz),w,p)
    lz1 = h*vz
    
    kx2 = h*forcex((vx + .5*kx1,vy + .5*ky1,vz + .5*kz1),w,p)
    lx2 = h*(vx + .5*kx1)
    
    ky2 = h*forcey((vx + .5*kx1,vy + .5*ky1,vz + .5*kz1),w,p)
    ly2 = h*(vy + .5*ky1)
    
    kz2 = h*forcez((vx + .5*kx1,vy + .5*ky1,vz + .5*kz1),w,p)
    lz2 = h*(vz + .5*kz1)
    
    kx3 = h*forcex((vx + .5*kx2,vy + .5*ky2,vz + .5*kz2),w,p)
    lx3 = h*(vx + .5*kx2)
    
    ky3 = h*forcey((vx + .5*kx2,vy + .5*ky2,vz + .5*kz2),w,p)
    ly3 = h*(vy + .5*ky2)
    
    kz3 = h*forcez((vx + .5*kx2,vy + .5*ky2,vz + .5*kz2),w,p)
    lz3 = h*(vz + .5*kz2)
    
    kx4 = h*forcex((vx + kx3, vy + ky3,vz + kz3),w,p)
    lx4 = h*(vx + kx3)
    
    ky4 = h*forcey((vx + kx3, vy + ky3,vz + kz3),w,p)
    ly4 = h*(vy + ky3)
    
    kz4 = h*forcez((vx + kx3, vy + ky3,vz + kz3),w,p)
    lz4 = h*(vz + kz3)
    
    vx = vx + (1.0/6)*(kx1 + 2*kx2 + 2*kx3 +kx4)
    x = x + (1.0/6)*(lx1 + 2*lx2 + 2*lx3 +lx4)
    
    vy = vy + (1.0/6)*(ky1 + 2*ky2 + 2*ky3 +ky4)
    y = y + (1.0/6)*(ly1 + 2*ly2 + 2*ly3 +ly4)
    
    vz = vz + (1.0/6)*(kz1 + 2*kz2 + 2*kz3 +kz4)
    z = z + (1.0/6)*(lz1 + 2*lz2 + 2*lz3 +lz4)
    
    
    return[(x,z,-1*y),(vx,vz,-1*vy)]   
#Start location and velocity for ball at startup
x = 0.0
y = 0.0
z = 2.0
vx = v0*cos(angle)
vy = 0
vz = v0*sin(angle)

L = 320
Hgraph = 325
#Creating the window for display and controls
w = window(width=1500, height=750,
           menus=True, title='Mechanics of Baseball',
           style=wx.SYSTEM_MENU | wx.CAPTION | wx.CLOSE_BOX)

p = w.panel
disp = display(title='Mechanics of Baseball', window=w, x=10, y=10, width=580, height=550,  center=(10,1,0), background=(0, 0, 0), forward = vector(1,-.09,-.20),
     range = 5,fov = pi/20)
#Rest button to set new parameters Go button does this

def Reset(evt):
    ball1.pos = (0,2,0)
    ball1.visible = True  
    ball1.velocity =(Speed.GetValue()*cos(Elevation.GetValue()*pi/180),Speed.GetValue()*sin(Elevation.GetValue()*pi/180),0)
    ball1.green = ball1.axis[1] + (.001*Orientation.GetValue()*pi/180)
    ball1.red = 1 + (.001*AngularVelocity.GetValue()*2*pi/60)
    fastball.visible = False
    slider.visible = False
    curveball.visible = False
    knuckleball.visible = False
    

#Exit program safely
    
def leave(evt): # called on "Exit under program control" button event
    exit()

# Defining all the buttons 
Go = wx.Button(p, label='Go', pos=(20,620))
Go.Bind(wx.EVT_BUTTON, Reset)

Exit = wx.Button(p, label='Exit ', pos=(220, 620))
Exit.Bind(wx.EVT_BUTTON, leave)

Speed = wx.Slider(p, pos=(600,50), size=(0.9*L,20), minValue=25.0, maxValue=50.0)
wx.StaticText(p, pos=(620,20), label='Set Initial Translational Velocity [55 to 112 MPH]')
Speed.SetValue(40)

Elevation = wx.Slider(p, pos=(600,100), size=(0.4*L,20), minValue=-5, maxValue=5)
wx.StaticText(p, pos=(620,70), label='Set Initial Elevation Angle [-5 to 5 degrees]')
Elevation.SetValue(-1)

Orientation = wx.Slider(p, pos=(600,150), size=(0.9*L,20), minValue=0, maxValue=360)
wx.StaticText(p, pos=(620,120), label='Set the Initial Angular Orientation')

AngularVelocity = wx.Slider(p, pos=(600,200), size=(0.9*L,20), minValue=0, maxValue=2000)
wx.StaticText(p, pos=(620,170), label='Set the Initial Angular Velocity')
AngularVelocity.SetValue(1800)

#Creating the strick zine, pitching mound and field
floor = box(pos=(9,0,0), length=13, height=.1, width=3, color= (0,1,0))
mound  = cylinder(pos= (0,0,0),axis=(0,.1,0), radius = 2.7432, color = (1,1,0))
rubber = box(pos = (0,.2,0), length = 0.1524, width = 0.6096, height = .01, color = (1,1,1))
strike_zone1 = box(pos=(18.4404,1.06,0), length=.05, height=.05, width=.4318, color= (0,1,1))
strike_zone2 = box(pos=(18.4404,.48,0), length=.05, height=.05, width=.4318,color= (0,1,1))
strike_zone3 = box(pos=(18.4404,.77,-.2159), length=.05, height=.58, width=.05, color= (0,1,1))
strike_zone4 = box(pos=(18.4404,.77,.2159), length=.05, height=.58, width=.05, color= (0,1,1))

#Creating the ball that can be adjusted
ball1 = sphere(pos = (0,2,0),velocity = (v0*cos(angle),v0*sin(angle),0), radius = 0.0365,make_trail=False, trail_type="curve")
ball1.green =1 + (.001*Orientation.GetValue()*pi/180)
ball1.red =1 + (.001*AngularVelocity.GetValue()*2.0*pi/60)
ball1.blue = 1
ball1.axis = (1,1,0)

#Creating 4 pitches for the All Pitches Demo
fastball =  sphere(visible = False, pos = (0,2,0),velocity = (42.5*cos(-2.0*pi/180), 42.5*sin(-2.0*pi/180),0), red = ((1800*2*pi/60)/1000 + 1),green = ((225.0*pi/180)/1000 + 0), blue = 0, radius = 0.0365,make_trail=False, trail_type="curve",axis = (1,0,0))
slider =  sphere(visible = False, pos = (0,2,0),velocity = (39.0*cos(-1.0*pi/180), 39.0*sin(-1.0*pi/180),0), red = ((1500*2*pi/60)/1000 + 1),green = ((0.0*pi/180)/1000 + 0), blue = 1, radius = 0.0365,make_trail=False, trail_type="curve",axis = (1,0,0))
curveball =  sphere(visible = False, pos = (0,2,0),velocity = (36.0*cos(0.5*pi/180), 36.0*sin(0.5*pi/180),0), red = ((1850*2*pi/60)/1000 + 1),green = ((46.0*pi/180)/1000 + 1), blue = 0, radius = 0.0365,make_trail=False, trail_type="curve",axis = (1,1,0))
knuckleball = sphere(visible = False, pos = (0,2,0),velocity = (31.0*cos(2.0*pi/180), 31.0*sin(2.0*pi/180),0), red = ((40*2*pi/60)/1000 + 1),green = ((30*pi/180)/1000 + 1), blue = 1, radius = 0.0365,make_trail=False, trail_type="curve",axis = (1,1,0))

balls = [ball1, fastball, slider, curveball, knuckleball]

#2nd display for real time update of initial conditions
disp2 = display(window=w,x = 600, y=230, width=350, height=150,range = 10)
SpeedLabel = text(text='Translational Velocity: ' + str(Speed.GetValue()) + ' MPH', pos = (0,3,0),color = (1,1,1),
    align='center') 
OrientationLabel = text(text='Angular Orientation Angle: ' + str(Orientation.GetValue()), pos = (0,-1,0),color = (1,1,1),
    align='center') 
ElevationLabel = text(text='Elevation Angle: ' + str(Elevation.GetValue()), pos = (0,1,0),color = (1,1,1),
    align='center')     
AngularVelocityLabel = text(text='Angular Velocity: ' + str(AngularVelocity.GetValue()), pos = (0,-3,0),color = (1,1,1),
    align='center') 

#All Pitches Demo, Reset all balls, hides the adjustable ball
def Demo(evt):
    ball1.visible = False
    fastball.visible = True
    slider.visible = True
    curveball.visible = True
    knuckleball.visible = True
    fastball.pos = (0,2,0)
    slider.pos = (0,2,0)
    curveball.pos = (0,2,0)
    knuckleball.pos = (0,2,0)
    fastball.velocity = (42.5*cos(-2.0*pi/180), 42.5*sin(-2.0*pi/180),0)
    slider.velocity = (39.0*cos(-1.0*pi/180), 39.0*sin(-1.0*pi/180),0)
    curveball.velocity = (36.0*cos(0.5*pi/180), 36.0*sin(0.5*pi/180),0)
    knuckleball.velocity = (31.0*cos(2.0*pi/180), 31.0*sin(2.0*pi/180),0)
    knuckleball.green = ((30*pi/180)/1000 + 1)
    
wx.StaticText(p, pos=(620,400), label='For All Pitches:\n Fastball = Red\n Slider = Magenta\n Curveball = Yellow\n Knuckleball = White')
 # Button for all pitches demo   
All = wx.Button(p, label='All Pitches', pos=(120,620))
All.Bind(wx.EVT_BUTTON, Demo)

#Loop that moves any ball that is visible and changes diplay with conditions in real time
while True:
    SpeedLabel.text =('Translational Velocity: '+str(Speed.GetValue()*2.23)+' MPH')
    OrientationLabel.text = ('Orientation Angle: ' + str(Orientation.GetValue())+ ' degrees')
    ElevationLabel.text = ('Elevation Angle: ' + str(Elevation.GetValue())+ ' degrees')     
    AngularVelocityLabel.text =('Angular Velocity: ' + str(AngularVelocity.GetValue()) + ' RPM')
    for pitch in balls:
        if pitch.x < 18.5 and pitch.visible == True :
            result = RK4(pitch.pos,pitch.velocity,(pitch.red-1)*1000,(pitch.green-pitch.axis[1])*1000)
            pitch.pos = result[0]
            pitch.velocity = result[1]
            if pitch.red < 1.05:
                pitch.green = ((pitch.green-pitch.axis[1])*1000 + (pitch.red-1)*1000*h*0.6)*.001 + 1
        
    rate(2000)    
    
#Used for graphing in earlier edition

#plt.plot(xvals,zvals, color = 'k')
#plt.xlabel("Baseball's Distance from Pitcher(m)")
#plt.ylabel("Baseball's Height(m)")
#plt.show()
#plt.plot(xvals,yvals, color = 'r')
#plt.xlabel("Baseball's Distance from Pitcher(m)")
#plt.ylabel("Baseball's Horizontal Displacement(m)")
#plt.show()
