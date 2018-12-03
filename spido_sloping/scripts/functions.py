import numpy as np
from control import *
from control.matlab import *  # MATLAB-like functions
from math import *
from scipy import signal
from scipy.integrate import odeint
#import symengine
import matplotlib.pyplot as plt


import matplotlib.dates as mdates
import matplotlib.pyplot as plt
from scipy.stats import norm
from sympy import Symbol, symbols, Matrix, sin, cos
from sympy import init_printing
from sympy.utilities.codegen import codegen
init_printing(use_latex=True)

from numpy import allclose             # Grab all of the NumPy functions
import numpy as np
from matplotlib.pyplot import plot, show # Grab MATLAB plotting functions
from control.matlab import tf, c2d, step, obsv  # MATLAB-like functions


import rospy
import numpy as np
import numpy.linalg as alg
import matplotlib.pyplot as plt
import pylab as pl
from pylab import *
#from scipy import *
from scipy.linalg import solve_continuous_are
import time
from nav_msgs.msg import Odometry
from spido_pure_interface.msg import cmd_drive
from std_msgs.msg import Float64, String, Int32
from sensor_msgs.msg import Imu, NavSatFix
from gazebo_msgs.msg import ModelStates
from contact_republisher.msg import contacts_msg
from scipy.linalg import sqrtm,expm,norm,block_diag
#from scipy.signal import place_poles
from mpl_toolkits.mplot3d import Axes3D
from numpy import mean,pi,cos,sin,sqrt,tan,arctan,arctan2,tanh,arcsin,arccos,\
                    exp,dot,array,log,inf, eye, zeros, ones, inf,size,\
                    arange,reshape,concatenate,vstack,hstack,diag,median,sign,sum,meshgrid,cross,linspace,append,round
from matplotlib.pyplot import *
from numpy.random import randn,rand
from numpy.linalg import inv, det, norm, eig
from scipy.linalg import sqrtm,expm,norm,block_diag
from mpl_toolkits.mplot3d import Axes3D
from math import factorial


import subprocess
import rospy
import numpy.linalg as alg
import pylab as pl
from pylab import *
from scipy import *

from scipy.linalg import solve_continuous_are
import time

from matplotlib.patches import Ellipse,Rectangle,Circle, Wedge, Polygon, Arc

from matplotlib.collections import PatchCollection
import shapely.geometry as geom
from scipy import spatial


from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import String

#from scipy.signal import cont2discrete as c2d
#from scipy.signal import step

#################################################################################
# Extended Kalman Filter Functions
#################################################################################

def kalman(x0,Gamma0,u,y,Gammaalpha,Gammabeta,A,C):
    S = C.dot(Gamma0).dot(np.transpose(C)) + Gammabeta
    Kal = Gamma0.dot(np.transpose(C)).dot(np.linalg.inv(S) )
    ytilde = y - C .dot(x0 )
    Gup = (np.eye(len(x0))-Kal .dot(C)).dot(Gamma0)
    xup = x0 + Kal.dot(ytilde)
    Gamma1 = A .dot(Gup) .dot(np.transpose(A)) + Gammaalpha
    x1 = A .dot(xup) + u
    return(x1,Gamma1)


################################################################################

def get_model_A_matrix(k,Cf,Cr,vit) :
    masse       = 880
    moment      = 86.7
    a       = 0.85
    b       = 0.85
    d       = 0.5
    
    a11=-2*(Cf+Cr)/(masse*vit)
    a12=-2*(a*Cf-b*Cr)/(masse*vit)-vit
    a13=0
    a14=0
    a21=-2*(a*Cf-b*Cr)/(vit*moment)
    a22=-2*(a*a*Cf+b*b*Cr)/(vit*moment)
    a23=0
    a24=0
    a31=1
    a32=0
    a33=0
    a34=vit
    a41=0
    a42=1
    a43=k*k*vit
    a44=0


    return np.array([   [a11,a12, a13, a14],
                        [a21,a22, a23, a24],
                        [a31,a32, a33, a34],
                        [a41,a42, a43, a44]])

################################################################################
def get_model_B_matrix(Cf,Cr) :
    masse       = 880
    moment      = 86.7
    a       = 0.85
    b       = 0.85
    d       = 0.5
    b11=2*Cf/masse
    b12=2*Cr/masse
    b21=2*a*Cf/moment
    b22=-2*b*Cr/moment
    b31=0
    b32=0
    b41=0
    b42=0
    return np.array([[b11,b12],
                        [b21,b22],
                        [b31,b32],
                        [b41,b42]])

################################################################################
def get_slipObserver_B_matrix(Vy,Vpsi,deltaf,deltar,Vx) :
    masse       = 880
    moment      = 86.7
    a       = 0.85
    b       = 0.85
    d       = 0.5
    
    b11=-2*(Vy+a*Vpsi-Vx*deltaf)/(masse*Vx)
    b12=-2*(Vy-b*Vpsi-Vx*deltar)/(masse*Vx)
    b21=-2*(a*Vy+a*a*Vpsi-a*Vx*deltaf)/(masse*moment)
    b22=-2*(b*Vy+b*b*Vpsi-b*Vx*deltar)/(masse*moment)
    b31=0
    b32=0
    b41=0
    b42=0
    return np.array([[b11,b12],
                        [b21,b22],
                        [b31,b32],
                        [b41,b42]])

################################################################################
def get_slipObserver_A_matrix(Vx,k) :
    masse       = 880
    moment      = 86.7
    a       = 0.85
    b       = 0.85
    d       = 0.5
    
    a11=0
    a12=0
    a13=0
    a14=0
    a21=0
    a22=-Vx
    a23=0
    a24=0
    a31=1
    a32=0
    a33=0
    a34=Vx
    a41=0
    a42=1
    a43=k*k*Vx
    a44=0

    return np.array([[a11,a12, a13, a14],
                        [a21,a22, a23, a24],
                        [a31,a32, a33, a34],
                        [a41,a42, a43, a44]])
                        

################################################################################
def get_slipObserver_A2_matrix(Vx,k) :
    masse       = 880
    moment      = 86.7
    a       = 0.85
    b       = 0.85
    d       = 0.5
    
    a11=0
    a12=0
    a21=0
    a22=-Vx


    return np.array([[a11,a12],
                        [a21,a22]])

################################################################################
def get_slipObserver_B2_matrix(Vy,Vpsi,deltaf,deltar,Vx) :
    masse       = 880
    moment      = 86.7
    a       = 0.85
    b       = 0.85
    d       = 0.5

    b11=-2*(-deltaf)/(masse)
    b12=-2*(deltar)/(masse)
    b21=-2*(-a*deltaf)/(moment)
    b22=-2*(-b*deltar)/(moment)

    return np.array([[b11,b12],
                        [b21,b22]])
################################################################################
def SlipObserver(Ao,Bo,C,xc,xcd,Ts,Vx) :
    Do=np.zeros((1,2))

    CorneringStiff=np.linalg.pinv(Bo).dot(xcd-Ao.dot(xc))
    Cf=int(abs(CorneringStiff[0,0]))
    Cr=int(abs(CorneringStiff[1,0]))

    return Cf,Cr
    
    
################################################################################
def SlipModel(Vx,Vy,Vpsi,deltaf,deltar) :
    masse       = 880
    moment      = 86.7
    a       = 0.85
    b       = 0.85
    d       = 0.5
    Ac=np.array([[0,-Vx],[0,0]])#SysDi.A
    Bc=np.array([[(-2*Vy-2*a*Vpsi+2*Vx*deltaf)/(masse*Vx),(-2*Vy+2*b*Vpsi-2*Vx*deltar)/(masse*Vx)],[(-2*a*Vy-2*a*a*Vpsi+2*a*Vx*deltaf)/(moment*Vx),(2*b*Vy-2*b*b*Vpsi+2*b*Vx*deltar)/(moment*Vx)]])#SysDi.A
    G=np.array([[(-4*a*Vy-4*a*a*Vpsi+4*a*Vx*deltaf)/(masse*moment*Vx),0],[0,(-4*a*Vy-4*a*a*Vpsi+4*a*Vx*deltaf)/(masse*moment*Vx)]])  #Observer gain matrix 
    c=np.array([[(a*Vy)/moment+Vpsi/masse],[(-a*Vy)/moment+Vpsi/masse]])
    return Ac,Bc,G,c

    
################################################################################
def SlipModel2(Vx,Vy,Vpsi,deltaf,deltar,k,Gamma,Tsamp) :
    masse       = 880
    moment      = 86.7
    a       = 0.85
    b       = 0.85
    d       = 0.5
    
    Ac=np.array([[0,-Vx],[0,0]])#SysObse.A
    
    Bc1=(-2*Vy-2*a*Vpsi+2*Vx*deltaf)/(masse*Vx)
    Bc2=(-2*Vy+2*b*Vpsi-2*Vx*deltar)/(masse*Vx)
    Bc3=(-2*a*Vy-2*a*a*Vpsi+2*a*Vx*deltaf)/(moment*Vx)
    Bc4=(2*b*Vy-2*b*b*Vpsi+2*b*Vx*deltar)/(moment*Vx)
    Bc=np.array([[Bc1,Bc2],[Bc3,Bc4]])#SysObse.B
    
    

    c1=Vx**0.8*(-Vy*Vy-a*a*Vpsi*Vpsi-2*a*Vpsi*Vy+2*Vx*deltaf*(Vy+a*Vpsi))
    c2=Vx**0.003*(-Vy*Vy-a*a*Vpsi*Vpsi+2*a*Vpsi*Vy+2*Vx*a*deltar*(-Vy+a*Vpsi))
    
   
    J1=Vx**0.8*(-2*Vy-2*a*Vpsi+2*Vx*deltaf)
    J2=Vx**0.8*(-2*a*a*Vpsi-2*a*Vy+2*Vx*a*deltaf)
    J3=Vx**0.003*(-2*Vy+2*a*Vpsi-2*Vx*deltar)
    J4=Vx**0.003*(-2*a*a*Vpsi+2*a*Vy+2*Vx*a*deltar)
   
    J=np.array([[J1,J2],[J3,J4]])

    co=np.array([[c1],[c2]])
    Go=J.dot(Bc)
    w, v = np.linalg.eig(Go)
    if w[0]<0 or w[1]<0:
        print('LOOOOOOOOOOOOOOxxxxxxxxxxxxxxxxxxxOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOl')
    
    X=np.array([[Vy],[Vpsi]])
    dGammadt=-Go.dot(Gamma)-Go.dot(np.linalg.pinv(Bc).dot(X)+co)
    dd=np.array([[1,0],[0,1]])
    if Gamma[0,0] < 0 and Gamma[1,0] < 0:
        dGammadt=-dd.dot(dGammadt)
    else:
        dGammadt=dd.dot(dGammadt)  
        
    
    Gamma=Gamma+Tsamp*dGammadt

    g=np.array([[1,0],[0,1]])
    CorneringStiff=g.dot(Gamma+co)
    Cf=CorneringStiff[0,0]#int(abs(CorneringStiff[0,0]))
    Cr=CorneringStiff[1,0]#int(abs(CorneringStiff[1,0]))
    
#    print('Cf=',Cf)
#    print('Cr=',Cr)

    return Cf,Cr,Gamma
################################################################################    
def SlipAngles(Vx,Vy,Vpsi,deltaf,deltar) :
    a       = 0.85
    b       = 0.85
    betaf=(Vy+a*Vpsi)/Vx-deltaf
    betar=(Vy-b*Vpsi)/Vx-deltar
#    print('betaf=',betaf*180/pi)
#    print('betar=',betar*180/pi)
    return betaf,betar
################################################################################
def ForceLater_observed(betaf,betar,Cf,Cr,k) :
    if (abs(k)>30):
        Fyf_obs       = 10000*betaf
        Fyr_obs       = 10000*betar
    else :
        Fyf_obs       = Cf*betaf
        Fyr_obs       = Cr*betar
    
    return Fyf_obs,Fyr_obs 
################################################################################
def AccAng(ay_F,ay_R) :
    l=1.7
    psidd=(ay_R-ay_F)/l
    return psidd  

################################################################################
def Sloping(Td,ax,ay,Vy,Vx,Vpsi) :
    g=9.81
    Phi=np.array([[1,g*Td,0,0],[0,g*Td,0,0],[0,0,1,-g*Td],[0,0,0,1]])
    Theta=np.array([[Td,0],[0,0],[0,Td],[0,0]])
    Cd=np.array([[1,0,0,0],[0,0,1,0]])
    u_slop=np.array([[ax+Vy*Vpsi],[ay-Vx*Vpsi]])
    y_slop=np.array([[Vy],[Vx]])
    return Phi,Theta,Cd,u_slop,y_slop
    
################################################################################
def Sloping2(ax,ay,Vy,Vx,Vpsi,alpha,theta) : 
    g=9.81
    numstates=5 # States
    h = 1.0/200.0 # Sample Rate of the Measurements is 200Hz
    dtGPS=1.0/10.0 # Sample Rate of GPS is 10Hz
    #vs, psis, dpsis, dts, xs, ys, lats, lons, axs = symbols('v \psi \dot\psi T x y lat lon a')
    
    fs = np.array([[ax+Vx*Vpsi+g*sin(alpha)],
             [ay-Vy*Vpsi-g*cos(alpha)*sin(theta)],
                    [0],[0],[0]])
             
    state = np.array([[Vx],[Vy],[Vpsi],[alpha],[theta]])
    inputt = np.array([[ax],[ay]])
    
    A_lin=np.array([[Vpsi, 0, Vx, g*cos(alpha), 0], 
                  [0, -Vpsi, -Vy, g*sin(alpha)*sin(theta), -g*cos(alpha)*cos(theta)], 
                    [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0]])
                    
    B_lin=np.array([[1, 0], [0, 1], [0, 0], [0, 0], [0, 0]])
    
    C_lin=np.array([[1, 0, 0, 0, 0], [0, 1, 0, 0, 0], [0, 0, 1, 0, 0]])
    
    D_lin=np.array([[0, 0], [0, 0], [0, 0]])
    
    
    Ak=np.eye(5,5)+h*A_lin
    
    uk=h*(fs-A_lin.dot(state)-B_lin.dot(inputt))
    
    z = np.array([[Vx],[Vy],[Vpsi]])
    
    zk=z+C_lin.dot(state)
    
    
#    Ad, Bd, Cd, Dd, dt = c2d((A_lin, B_lin, C_lin, D_lin), h, method='zoh')
 
    Gamma_slop=h*np.eye(5,5)
    Gammaalpha_slop=h*0.0000001*np.eye(5,5)
    Gammabeta_slop=h**2*np.eye(3,3)*0.0000001
    
    return state,Gamma_slop,uk,zk,Gammaalpha_slop,Gammabeta_slop,Ak,C_lin




################################################################################
def Sloping4(ax,ay,az,x_slope) :
    g=9.81
    state = x_slope
    Vx=5
    alpha=x_slope[1,0]
    inputt = np.array([[ax]])

    h = 1.0/200.0 # Sample Rate of the Measurements is 200Hz

    fs= np.array([[ax-g*sin(alpha)],
             [0]])
    
    A_lin=np.array([[0, -g*cos(alpha)],
     [0,0]])

                    
    B_lin=np.array([[1], [0]])

    C_lin=np.array([[1, 0]])

    Ak=np.eye(2,2)+h*A_lin
    
    uk=h*(fs-A_lin.dot(state)-B_lin.dot(inputt))
    
    z = np.array([[Vx]])
    
    zk=z+C_lin.dot(state)
    
    
#    Ad, Bd, Cd, Dd, dt = c2d((A_lin, B_lin, C_lin, D_lin), h, method='zoh')
 
    Gamma_slop=h*np.eye(2,2)
    Gammaalpha_slop=h*0.0000001*np.eye(2,2)
    Gammabeta_slop=h**2*np.eye(1,1)*0.0000001
    
    return state,Gamma_slop,uk,zk,Gammaalpha_slop,Gammabeta_slop,Ak,C_lin

################################################################################
def Sloping5(ax,ay,az,x_slope) :
    g=9.81
    state = x_slope
    Vy=x_slope[0,0]
    theta=x_slope[1,0]
    inputt = np.array([[ax]])

    h = 1.0/200.0 # Sample Rate of the Measurements is 200Hz

    fs= np.array([[ax-g*sin(alpha)],
             [0]])
    
    A_lin=np.array([[0, -g*cos(alpha)],
     [0,0]])

                    
    B_lin=np.array([[1], [0]])

    C_lin=np.array([[1, 0]])

    Ak=np.eye(2,2)+h*A_lin
    
    uk=h*(fs-A_lin.dot(state)-B_lin.dot(inputt))
    
    z = np.array([[Vx]])
    
    zk=z+C_lin.dot(state)
    
    
#    Ad, Bd, Cd, Dd, dt = c2d((A_lin, B_lin, C_lin, D_lin), h, method='zoh')
 
    Gamma_slop=h*np.eye(2,2)
    Gammaalpha_slop=h*0.0000001*np.eye(2,2)
    Gammabeta_slop=h**2*np.eye(1,1)*0.0000001
    
    return state,Gamma_slop,uk,zk,Gammaalpha_slop,Gammabeta_slop,Ak,C_lin


################################################################################
def Derivative(wx,wy,wz,psi,phi,theta) :
    theta_dot=cos(theta)*wx+sin(phi)*sin(theta)*wy+cos(phi)*sin(theta)*wz
    phidot=+sin(phi)*tan(theta)*wy+cos(phi)*tan(theta)*wz
    phi_rdot=cos(theta)*phidot
    psi_dot=(sin(phi)/sin(theta))*wy+(cos(phi)/cos(theta))*wz
    xdot=np.array([[phi_rdot], [theta_dot],[psi_dot]])
    return xdot

################################################################################
def Slopes_Mesur(ax,ay,az,Vy,Vx,Vpsi) :
    g=9.81
    phi_mesu=atan((ay-Vx*Vpsi)/az)
    
    xx=(-ax-Vy*Vpsi)/g

    if (fabs(xx) >= 1):
        theta_mesu= copysign(pi/2, xx)
    else:
        theta_mesu= asin(xx)


    x_mesu=np.array([[phi_mesu], [theta_mesu]])
    return x_mesu
################################################################################
def Bank_slope(x0,h,wx,wy,wz,psi,phi,theta,ax,ay,az,Vy,Vx,Vpsi) :
    xdot=Derivative(wx,wy,wz,psi,phi,theta)
    x_mesu=Slopes_Mesur(ax,ay,az,Vy,Vx,Vpsi)
    Kr=np.array([[0,0], [0,0]]) 
    ytilde = x0 - x_mesu
    xup = x0 + Kr.dot(ytilde)
    x1 = xup#+h*xdot
    return x_mesu

################################################################################
def Slopingg1(ax,ay,az,vit,x_slope,wx,wy,wz,psi,phi,theta) :
    g=9.81
    xdot=Derivative(wx,wy,wz,psi,phi,theta)
    u4=xdot[0,0]
    u5=xdot[1,0]
    state = x_slope#np.array([[Vy],[alpha],[theta]])
    Vy=x_slope[0,0]
    theta_r=x_slope[1,0]
    phi_r= x_slope[2,0]

    if Vy==0:
        Vy=0.001

    Vx=vit
    inputt = np.array([[ax],[ay],[az],[u4],[u5]])

    h = 1.0/200.0 # Sample Rate of the Measurements is 200Hz

    fs= np.array([[Vx*(ax+g*sin(theta_r))/(Vy)+ay+az*tan(phi_r)],
             [u4],
                [u5]])
    
    A_lin=np.array([[-Vx*(ax + 9.81*sin(theta_r))/Vy**2, 9.81*Vx*cos(theta_r)/Vy, az*(tan(phi_r)**2 + 1)], 
        [0, 0, 0], 
        [0, 0, 0]])

                    
    B_lin=np.array([[Vx/Vy, 1, tan(phi_r), 0, 0], [0, 0, 0, 1, 0], [0, 0, 0, 0, 1]])

    C_lin=np.array([[1, 0, 0]])

    Ak=np.eye(3,3)+h*A_lin

    
    uk=h*(fs-A_lin.dot(state)-B_lin.dot(inputt))
    
    z = np.array([[Vy]])
    
    zk=z+C_lin.dot(state)
    
    
 
    # Gamma_slop=h*np.eye(3,3)
    # Gammaalpha_slop=h*0.0000001*np.eye(3,3)
    # Gammabeta_slop=h**2*np.eye(1,1)*0.0000001
    
    return state,uk,zk,Ak,C_lin


################################################################################
def slope_estimator(ax,ay,az,vit,xhat0,Gammax0,wx,wy,wz,psi,phi,theta) :

    dt1=1/200
    g=9.81
    Vy=xhat0[0,0]
    Vpsi=xhat0[1,0]
    theta_r=xhat0[2,0]
    phi_r=xhat0[3,0]

    A1=np.array([[0,-vit, -g*sin(theta_r)*sin(phi_r),g*cos(theta_r)*cos(phi_r)], 
                    [0, 0,0,0],
                    [0, 0,0,0],
                    [0, 0,0,0]])
                    
    B1=np.eye(4,4)# array([[1, -tan(phi_r), 0,0],[0, 0, 1,0],[0, 0, 0,1]])
    
    C1=np.array([[1, 0,0,0],
                [0, 1,0,0]])

    xdot=Derivative(wx,wy,wz,psi,phi,theta)


    u1=ay
    u2=wz
    u3=xdot[0,0]
    u4=xdot[1,0]

    y1=np.array([[Vy],[Vpsi]])

    u1=np.array([[u1],[u2],[u3],[u4]])


    Gammaalpha1=dt1*0.0000001*np.eye(4,4)
    Gammabeta1=dt1**2*np.eye(2,2)*0.0000001

    xhat1,Gammax1=kalman(xhat0,Gammax0,dt1*B1.dot(u1),y1,Gammaalpha1,Gammabeta1,np.eye(4,4)+dt1*A1,C1)


    # S1 = C1.dot(Gammax0).dot(np.transpose(C1)) + Gammabeta1
    # print('S',np.shape(S1))
    # Kal = Gammax0.dot(np.transpose(C1)).dot(np.linalg.inv(S1))
    # ytilde = y1 - C1 .dot(xhat0 )
    # Gup = (np.eye(len(xhat0))-Kal .dot(C1)).dot(Gammax0)
    # xup = xhat0 + Kal.dot(ytilde)
    # AA= np.eye(4,4) + dt1*A1
    # Gammax1 = AA.dot(Gup).dot(np.transpose(AA)) + Gammaalpha1
    # xhat1 = AA.dot(xup) + (dt1*B1.dot(u1))

    return(xhat1,Gammax1)


################################################################################
def Model_slopee(vy,vpsi,ax,ay,az,vit,xhat0,Gammax0,wx,wy,wz,psi,phi,theta) : 
    g=9.81
    dt=0.1
    numstates=5 # States
    h = 1.0/200.0 # Sample Rate of the Measurements is 200Hz
    dtGPS=1.0/10.0 # Sample Rate of GPS is 10Hz
    xdot=Derivative(wx,wy,wz,psi,phi,theta)
    u1=ax
    u2=ay
    u3=wz#xdot[2,0]
    u4=wy#xdot[0,0]
    u5=wx#xdot[1,0]

    # state = x_slope#np.array([[Vy],[alpha],[theta]])
    Vy=xhat0[0,0]
    Vpsi=xhat0[1,0]
    theta_r=xhat0[2,0]
    phi_r= xhat0[3,0]
    #[u1+Vx*Vpsi+g*sin(theta_r)],
             
    fs = np.array([[u2-vy*vpsi-g*cos(theta_r)*sin(phi_r)],
                    [u3],[u4],[u5]])
             
    # state = np.array([[Vx],[Vy],[Vpsi],[alpha],[theta]])
    inputt = np.array([[u2],[u3],[u4],[u5]])
    
    A_lin=np.array([[ -vpsi, -vy, g*sin(theta_r)*sin(phi_r), -g*cos(theta_r)*cos(phi_r)], 
                    [0, 0, 0, 0], 
                    [0, 0, 0, 0], 
                    [0, 0, 0, 0]])


                    
    B_lin=np.eye(4,4)
    
    C_lin=np.array([[1, 0, 0, 0], [0, 1, 0, 0]])

    # print('ddddd',np.linalg.matrix_rank(obsv(A_lin, C_lin)))
    
    
    
    Ak=np.eye(4,4)+h*A_lin
    
    uk=h*(fs-A_lin.dot(xhat0)-B_lin.dot(inputt))
    
    z = np.array([[vy],[vpsi]])
    
    zk=z+C_lin.dot(xhat0)

    Gammaalpha1=dt*0.0000001*np.eye(4,4)
    Gammabeta1=dt**2*np.eye(2,2)*0.0000001
    
    # Gammax0=dt*0.0000001*np.eye(4,4)
    
#    Ad, Bd, Cd, Dd, dt = c2d((A_lin, B_lin, C_lin, D_lin), h, method='zoh')
 
    S1 = C_lin.dot(Gammax0).dot(np.transpose(C_lin)) + Gammabeta1
    Kal = Gammax0.dot(np.transpose(C_lin)).dot(np.linalg.inv(S1))
    ytilde = zk - C_lin .dot(xhat0 )
    Gup = (np.eye(len(xhat0))-Kal .dot(C_lin)).dot(Gammax0)
    xup = xhat0 + Kal.dot(ytilde)
    Gammax1 = Ak.dot(Gup).dot(np.transpose(Ak)) + Gammaalpha1
    xhat1 = Ak.dot(xup) + (uk)
    
    
    return xhat1,Gammax1



################################################################################
def Model_slope2(vy,ax,ay,az,vit,xhat0,Gammax0,wx,wy,wz,psi,phi,theta) : 
    g=9.81
    dt=0.1
    numstates=5 # States
    h = 1.0/200.0 # Sample Rate of the Measurements is 200Hz
    dtGPS=1.0/10.0 # Sample Rate of GPS is 10Hz
    xdot=Derivative(wx,wy,wz,psi,phi,theta)
    u1=ay
    u2=xdot[0,0]
    u3=xdot[1,0]

    # state = x_slope#np.array([[Vy],[alpha],[theta]])
    Vy=xhat0[0,0]
    theta_r=xhat0[1,0]
    phi_r= xhat0[2,0]
    #[u1+Vx*Vpsi+g*sin(theta_r)],
             
    fs = np.array([[u1-g*cos(theta_r)*sin(phi_r)],
                    [u2],
                    [u3]])
             
    # state = np.array([[Vx],[Vy],[Vpsi],[alpha],[theta]])
    inputt = np.array([[u1],[u2],[u3]])
    
    A_lin=np.array([[ 0, g*sin(theta_r)*sin(phi_r), -g*cos(theta_r)*cos(phi_r)], 
                    [0, 0, 0], 
                    [0, 0, 0]])


                    
    B_lin=np.eye(3,3)
    
    C_lin=np.array([[1, 0, 0]])

    # print('ddddd',np.linalg.matrix_rank(obsv(A_lin, C_lin)))
    
    
    
    Ak=np.eye(3,3)+h*A_lin
    
    uk=h*(fs-A_lin.dot(xhat0)-B_lin.dot(inputt))
    
    z = np.array([[vy]])
    
    zk=z+C_lin.dot(xhat0)

    Gammaalpha1=dt*0.0000001*np.eye(3,3)
    Gammabeta1=dt**2*np.eye(1,1)*0.0000001


 
    S1 = C_lin.dot(Gammax0).dot(np.transpose(C_lin)) + Gammabeta1
    Kal = Gammax0.dot(np.transpose(C_lin)).dot(np.linalg.inv(S1))
    ytilde = zk - C_lin .dot(xhat0 )
    Gup = (np.eye(len(xhat0))-Kal .dot(C_lin)).dot(Gammax0)
    xup = xhat0 + Kal.dot(ytilde)
    Gammax1 = Ak.dot(Gup).dot(np.transpose(Ak)) + Gammaalpha1
    xhat1 = Ak.dot(xup) + (uk.dot(u1))
    
    
    return xhat1,Gammax1


################################################################################
def Model_slope_New(vy,vpsi,ax,ay,az,vit,xhat0,Gammax0,wx,wy,wz,psi,phi,theta,delta) : 

    xdot=Derivative(wx,wy,wz,psi,phi,theta)
    u1=delta[0,0]
    u2=delta[1,0]
    u3=wy#xdot[0,0]
    u4=wx#xdot[1,0]

    # state = x_slope#np.array([[Vy],[alpha],[theta]])
    Vy=xhat0[0,0]
    Vpsi=xhat0[1,0]
    theta_r=xhat0[2,0]
    phi_r= xhat0[3,0]

    Cf=15000
    Cr=20000
    dt=1/200
    masse       = 880
    moment      = 86.7
    a       = 0.85
    b       = 0.85
    d       = 0.5
    g=9.81
    
    a11=-2*(Cf+Cr)/(masse*vit)
    a12=-2*(a*Cf-b*Cr)/(masse*vit)-vit
    a13=0
    a14=0
    a21=-2*(a*Cf-b*Cr)/(vit*moment)
    a22=-2*(a*a*Cf+b*b*Cr)/(vit*moment)
    a23=-g*sin(theta_r)*sin(phi_r)
    a24=g*cos(theta_r)*cos(phi_r)



    b11=2*Cf/masse
    b12=2*Cr/masse
    b21=2*a*Cf/moment
    b22=-2*b*Cr/moment
    b31=0
    b32=0
    b41=0
    b42=0


    u = np.array([[u1],[u2],[u3],[u4]])

    print ('u=',u)
    y = np.array([[vy],[vpsi]])


    fs = np.array([[a11*Vy+a12*Vpsi+b11*u1+b12*u2-g*cos(theta_r)*sin(phi_r)],
                    [a21*Vy+a22*Vpsi+b21*u1+b22*u2],
                    [u3],
                    [u4]])
    
    A_lin=np.array([[ a11, a12, a13,a14], 
                    [a21, a22, a23,a24], 
                    [0, 0, 0,0],
                    [0, 0, 0,0]])


    B_lin=np.array([[ b11, b12, 0,0], 
                    [b21, b22, 0,0], 
                    [0, 0, 1,0],
                    [0, 0, 0,1]])
    

    C_lin=np.array([[1, 0, 0,0],
                        [0, 1, 0,0]])



    Ak=np.eye(4,4)+dt*A_lin
    
    uk=dt*(fs-A_lin.dot(xhat0)-B_lin.dot(u))
    
    z = np.array([[vy],[vpsi]])
    
    zk=z+C_lin.dot(xhat0)




    Gammaalpha1=dt*0.0000001*np.eye(4,4)
    Gammabeta1=dt**2*np.eye(2,2)*0.0000001



    # S1 = C_lin.dot(Gammax0).dot(np.transpose(C_lin)) + Gammabeta1
    # Kal = Gammax0.dot(np.transpose(C_lin)).dot(np.linalg.inv(S1))
    # ytilde = zk - C_lin .dot(xhat0 )
    # Gup = (np.eye(len(xhat0))-Kal .dot(C_lin)).dot(Gammax0)
    # xup = xhat0 + Kal.dot(ytilde)
    # Gammax1 = Ak.dot(Gup).dot(np.transpose(Ak)) + Gammaalpha1
    # xhat1 = Ak.dot(xup) + (uk)



    # xhat0,Gammax0=kalman(xhat0,Gammax0,dt*B_lin.dot(u),y,Gammaalpha1,Gammabeta1,np.eye(4,4)+dt*A_lin,C_lin)

    return uk,zk,Gammaalpha1,Gammabeta1,Ak,C_lin#xhat1,Gammax1

def Luenberger(vy,vpsi,ax,ay,az,vit,xhat0,Gammax0,wx,wy,wz,psi,phi,theta,delta) :

    dt=1/200
    xdot=Derivative(wx,wy,wz,psi,phi,theta)
    u1=xdot[0,0]
    u2=xdot[1,0]

    y=Slopes_Mesur(ax,ay,az,vy,vit,vpsi)
    theta_rMesu=y[0,0]
    phi_rMesu=y[1,0]

    u=np.array([[u1],[u2]])





    A_lin=np.array([[0, 0],
                    [0, 0]])


    B_lin=np.array([[ 1,0],
                    [ 0,1]])
    

    C_lin=np.array([[1, 0],
                        [0, 1]])


    Ad=np.eye(2,2)+dt*A_lin
    Bd=dt*B_lin
    Cd=C_lin


    G=np.array([[0.2, 0],
                        [0, 0.2]])


    # Gammaalpha1=dt*0.0000001*np.eye(2,2)
    # Gammabeta1=dt**2*np.eye(2,2)*0.0000001

    # xhat1,Gammax1=kalman(xhat0,Gammax0,dt*B_lin.dot(u),y,Gammaalpha1,Gammabeta1,np.eye(2,2)+dt*A_lin,C_lin)

    xhat1=Ad.dot(xhat0)+Bd.dot(u)+G.dot(y-Cd.dot(xhat0))
    return xhat1