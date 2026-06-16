# Current state: attitude - SLERP, pos/vel - linear KF (Goal: Full EKF)

# Issues
# - Heavier yaw drift/mag issues?
# - 'Choppy' Pos and velocity estimates w/ minor x/y drift
# - Poor altitude (z) estimation/drift
# - Need to repurpose for reading raw data from log

import serial
import time
from datetime import datetime
import numpy as np
from vpython import *
import math

# Complementary filter weights
ALPHA = 0.985
ALPHAY = 0.995

# Open serial
ser = serial.Serial('COM3', 115200)

# Create log file w/ header
f = open("log.txt", "w")
f.write("Time,Pos_x,Pos_y,Pos_z,Vel_x,Vel_y,Vel_zRoll,Pitch,Yaw\n")

# Scene setup
scene.width = 1200
scene.height = 700
scene.background = color.white

# Initial camera pos
scene.camera.pos = vector(-5, 0, 0)
scene.camera.axis = vector(5, 0, 0)  # points toward origin
scene.camera.rotate(angle=-pi/2, axis=vector(1,0,0))

# Inital basis vectors (i, j, k)
xVec = arrow(pos=vector(0, 0, 0), axis=vector(1, 0, 0), color=color.red)
yVec = arrow(pos=vector(0, 0, 0), axis=vector(0, 1, 0), color=color.blue)
zVec = arrow(pos=vector(0, 0, 0), axis=vector(0, 0, 1), color=color.green)
xPerm = arrow(pos=vector(0, 0, 0), axis=vector(2, 0, 0), color=color.black, shaftwidth=0.01)
yPerm = arrow(pos=vector(0, 0, 0), axis=vector(0, 2, 0), color=color.black, shaftwidth=0.01)
zPerm = arrow(pos=vector(0, 0, 0), axis=vector(0, 0, 2), color=color.black, shaftwidth=0.01)

# Set finite logging time
collectionTime = (120) + time.time()

# Set initial conditions
tPrev, ztPrev = time.time(), time.time()
gpsInit = False
headingInit = False
q = np.array([1, 0, 0, 0]) # quaternion initialization
x_n = np.array([0, 0, 0, 0, 0, 0]) # KF state initialization (x, y, z, vx, vy, vz)

# EKF matrices
# Process noise covariance
Q = np.diag([0.01, 0.01, 0.01, 0.1, 0.1, 0.1])
# Error covariance
P = np.eye(6) * 500
# Measurement matrices (currently seperate for partial updates)
H_x = np.array([[1, 0, 0, 0, 0, 0]])
H_y = np.array([[0, 1, 0, 0, 0, 0]])
H_z = np.array([[0, 0, 1, 0, 0, 0]])
H_vx = np.array([[0, 0, 0, 1, 0, 0]])
H_vy = np.array([[0, 0, 0, 0, 1, 0]])
H_vz = np.array([[0, 0, 0, 0, 0, 1]])

# Measurement covariance
R_xy = np.array([[5.0]])   # position noise
R_z  = np.array([[10.0]])  # altitude noisier
R_vxy = np.array([[5.0]])   # velocity noise (xy)
R_vz  = np.array([[20.0]])  # velocity noisierer (z)

# Calculate rotation matrix from quaternion for basis visualization
def rotation_matrix(q):
    w, x, y, z = q

    Rot = np.array([
        [1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(w*y + x*z)],
        [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(w*x + y*z), 1 - 2*(x**2 + y**2)]
    ])
    
    return Rot

# Get new quaternion from gyro data integration
def quatIntegrate(q, gyro, deltaT):
    wx, wy, wz = gyro[0], -gyro[1], -gyro[2]

    w, x, y, z = q

    # q_dot = 1/2 * q * [0, wx, wy, wz]
    dw = (-x*wx - y*wy - z*wz) / 2
    dx = (w*wx + y*wz - z*wy) / 2
    dy = (w*wy - x*wz + z*wx) / 2
    dz = (w*wz + x*wy - y*wx) / 2

    q_new = np.array([w + dw*deltaT, x + dx*deltaT, y + dy*deltaT, z + dz*deltaT])

    q_new = q_new / np.linalg.norm(q_new)

    return q_new

# Euler angles to quaternion
def eulerToQuat(roll, pitch, yaw):
    cr, sr = np.cos(roll/2), np.sin(roll/2)
    cp, sp = np.cos(pitch/2), np.sin(pitch/2)
    cy, sy = np.cos(yaw/2), np.sin(yaw/2)

    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy

    return np.array([w, x, y, z])

# Quaternion to euler angles
def quatToEuler(q):
    w, x, y, z = q

    roll = np.atan2(2*(w*x + y*z), 1-2*(x**2 + y**2))
    pitch = np.arcsin(np.clip(2*(w*y - x*z), -1.0, 1.0))
    yaw = np.atan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))

    return roll, pitch, yaw

# Get attitude estimate from accel data (gravity)
def accelAngle(accel):
    ax, ay, az = accel
    pitch = math.atan2(ax, az)
    roll = math.atan2(-ay, sqrt(ax**2 + az**2))

    return eulerToQuat(roll, pitch, 0)

# Filter for attitude quaternions
def slerp(q0, q1, h):
    dot = np.clip(np.dot(q0, q1), -1.0, 1.0)

    if dot < 0:
        q1 = -q1
        dot = -dot

    theta = np.arccos(dot)

    if theta < 1e-6:
        return q0

    q = (np.sin((1-h)*theta)/np.sin(theta) * q0) + (np.sin(h*theta)/np.sin(theta) * q1)

    return q

# Get heading from magnetometer and current attitude estimate
def getHeading(xMag, yMag, zMag, q):
    roll, pitch, _ = quatToEuler(q)
    Bx = xMag * np.cos(pitch) + zMag * np.sin(pitch)
    By = xMag * np.sin(roll) * np.sin(pitch) + yMag * np.cos(roll) - zMag * np.sin(roll) * np.cos(pitch)

    heading = math.atan2(-By, Bx) * 180.0 / math.pi
    if heading < 0: 
        heading += 360.0

    return heading

# Get cartesian coordinates from GPS latitude and longitude
def gpsToCart(lat, lon, lat0, lon0):
    r = 111111 # m/deg lat

    dy = (lat-lat0) * r
    dx = (lon-lon0) * r * np.cos(np.radians(lat0))

    return dy, dx

# Get velocity based on GPS speed and course
def getGPSVel(speed, course):

    velx = speed * np.sin(course)
    vely = speed * np.cos(course)

    return velx, vely

# Partial state update with Kalman Filter (issues with inconsistent GPS data)
def kfUpdate(x_n, P, H, R, z):            
    S = H @ P @ H.T + R                   
    K = P @ H.T @ np.linalg.inv(S)       
    y = z - H @ x_n                       
    x_n += K @ y                          
    P = (np.eye(len(x_n)) - K @ H) @ P    
    return x_n, P

while time.time() < collectionTime:
    if ser.in_waiting > 0:
        # Read line
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        values = line.split(",")

        # Get good gps connection
        if len(values) == 16:
            if not gpsInit and (float(values[-1]) < 2.0 and float(values[-1]) > 0.1):
                    # Set initial values
                    lat0, lon0, alt0 = values[9:12]
                    lat0 = float(lat0)
                    lon0 = float(lon0)
                    alt0 = float(alt0)
                    gpsInit = True
                    accel0 = np.array([float(values[0]), float(values[1]), float(values[2])])
                    gpsX = float(values[9])
                    gpsY = float(values[10])
                    gpsZ = float(values[11])
                    
            elif gpsInit:
                # Get values
                # Format: ax,ay,az,gx,gy,gz,magx,magy,magz,lat,lon,alt,speed,course,sats,hdop
                values = [float(x) for x in values[0:15]]
                accel = np.array(values[0:3]) 
                gyro = np.array(values[3:6])
                deltaT = time.time() - tPrev

                #-------------------------------------------------------------------
                #[][][][][][][][][][][][]Actual Calculations[][][][][][][][][][][][]
                #-------------------------------------------------------------------

                # Update gyro quaternion
                q = quatIntegrate(q, gyro, deltaT)
                # Update accel quaternion
                q1 = accelAngle(accel)
                # Add yaw to accel quaternion since can't estimate yaw
                rollA, pitchA, _ = quatToEuler(q1)
                _, _, yawG = quatToEuler(q)
                q1 = eulerToQuat(rollA, pitchA, yawG)

                # Fuse gyro and accel quaternions
                q = slerp(q, q1, 1-ALPHA)

                # Get magnetometer heading
                heading = getHeading(values[6], values[7], values[8], q) * -1

                # Issues with initial yaw estimations
                #--------------------------------------
                if not headingInit:
                    heading0 = heading
                heading -= heading0
                heading = heading * math.pi / 180

                roll, pitch, _ = quatToEuler(q)
                q1 = eulerToQuat(roll, pitch, heading)

                # Initialize quaternion from mag on first frame
                if not headingInit:
                    q = q1
                    headingInit = True
                else:
                    # Correct yaw toward mag
                    q_mag = eulerToQuat(roll, pitch, heading)
                    q = slerp(q, q1, 1 - ALPHAY)
                #--------------------------------------

                # Adjust for rotation and gravity
                Rot = rotation_matrix(q)
                accel = np.dot(Rot.T, accel) - accel0

                # KF prediction
                F = np.array([
                    [1, 0, 0, deltaT, 0, 0],
                    [0, 1, 0, 0, deltaT, 0],
                    [0, 0, 1, 0, 0, deltaT],
                    [0, 0, 0, 1, 0, 0],
                    [0, 0, 0, 0, 1, 0],
                    [0, 0, 0, 0, 0, 1]])
                
                B = np.array([
                    [(deltaT**2)/2, 0, 0],
                    [0, (deltaT**2)/2, 0],
                    [0, 0, (deltaT**2)/2],
                    [deltaT, 0, 0],
                    [0, deltaT, 0],
                    [0, 0, deltaT]])
                
                x_n = F@x_n + B@accel
                P = F@P@(F.T) + Q

                # get gps update
                gX, gY = gpsToCart(values[9], values[10], lat0, lon0)

                # KF gps correction
                if gX != gpsX:
                    gpsX = gX
                    x_n, P = kfUpdate(x_n, P, H_x, R_xy, np.array([gpsX]))
                    vx, vy = getGPSVel(values[12], values[13])
                    x_n, P = kfUpdate(x_n, P, H_vx, R_vxy, np.array([vx]))
                    x_n, P = kfUpdate(x_n, P, H_vy, R_vxy, np.array([vy]))

                if gY != gpsY:
                    gpsY = gY
                    x_n, P = kfUpdate(x_n, P, H_y, R_xy, np.array([gpsY]))
                    vx, vy = getGPSVel(values[12], values[13])
                    x_n, P = kfUpdate(x_n, P, H_vx, R_vxy, np.array([vx]))
                    x_n, P = kfUpdate(x_n, P, H_vy, R_vxy, np.array([vy]))

                if values[11] != gpsZ:
                    x_n, P = kfUpdate(x_n, P, H_vz, R_z, np.array([gpsZ - alt0]))
                    dtZ = time.time() - ztPrev

                    if dtZ > 0.5:
                        vs = (values[11] - gpsZ) / dtZ
                        x_n, P = kfUpdate(x_n, P, H_z, R_z, np.array([gpsZ - alt0]))
                        ztPrev = time.time()

                    gpsZ = values[11]

                #-------------------------------------------------------------------
                #[][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][]:)
                #-------------------------------------------------------------------

                # Update basis vectors with rotation
                xVec.axis = vector(*Rot[:,0])
                yVec.axis = vector(*Rot[:,1])
                zVec.axis = vector(*Rot[:,2])
                
                roll, pitch, yaw = quatToEuler(q)

                roll *= 180/math.pi
                pitch *= 180/math.pi
                yaw *= 180/math.pi

                if roll < 0:
                    roll += 360
                if pitch < 0:
                    pitch += 360
                if yaw < 0:
                    yaw += 360

                # Update log
                timeObj = datetime.fromtimestamp(time.time())
                f.write(f"{timeObj.strftime("%H:%M:%S.%f")[:-4]},{x_n[0]:.3f},{x_n[1]:.3f},{x_n[2]:.3f},{x_n[3]:.3f},{x_n[4]:.3f},{x_n[5]:.3f},{roll:.3f},{pitch:.3f},{yaw:.3f}\n")

                tPrev = time.time()
    rate(60)
f.close()