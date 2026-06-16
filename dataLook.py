# Data stream testing

import serial
import time
import numpy as np

# Open serial
ser = serial.Serial('COM3', 115200)

# Set initial conditions
timePrev = time.time()
roll, pitch, yaw = 0, 0, 0

def rotation_matrix(roll, pitch, yaw):
    # Trig calculations
    cr = np.cos(roll)
    sr = np.sin(roll)

    cp = np.cos(pitch)
    sp = np.sin(pitch)

    cy = np.cos(yaw)
    sy = np.sin(yaw)

    # Pitch matrix
    Rx = np.array([
        [1, 0, 0],
        [0, cp, -sp],
        [0, sp, cp]
    ])

    # Roll matrix
    Ry = np.array([
        [cr, 0, sr],
        [0, 1, 0],
        [-sr, 0, cr]
    ])

    # Yaw matrix
    Rz = np.array([
        [cy, -sy, 0],
        [sy, cy, 0],
        [0, 0, 1]
    ])

    # Combine
    return Rx @ Ry @ Rz

def euler(gyro, roll, pitch, yaw, deltaT):
    # Rates from IMU
    bodyRates = np.array([gyro[0], -gyro[1], -gyro[2]])
    # Trig calculations
    sRoll = np.sin(roll)
    cRoll = np.cos(roll)
    tPitch = np.tan(pitch)
    scPitch = 1/np.cos(pitch)
    sPitch = np.sin(pitch)

    # IMU rates to euler rates
    M = np.array([
        [1, sRoll*tPitch, cRoll*tPitch],
        [0, cRoll, -sPitch],
        [0, sRoll*scPitch, cRoll*scPitch]
    ])

    # Integrate for euler angles
    rates = np.dot(M, bodyRates)
    angles = np.array([roll, pitch, yaw]) + rates * deltaT

    return angles[0], angles[1], angles[2]

while True:
    if ser.in_waiting > 0:
        # Read line
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        values = line.split(",")   

        if len(values) == 16:
            # Get values (ax,ay,az,gx,gy,gz,heading,lat,lon,alt,speed,course,sats,hdop)
            values = [float(x) for x in values]
            accel = np.array(values[0:3])
            gyro = np.array(values[3:6])
            deltaT = time.time() - timePrev

            #----------------------------------------------------------------------------------------------
            # Get euler angles
            roll, pitch, yaw = euler(gyro, roll, pitch, yaw, deltaT)

            # Update basis vectors with rotation
            R = rotation_matrix(roll, pitch, yaw)

            #----------------------------------------------------------------------------------------------
            # Print and update

            # print(f"{roll},{pitch},{yaw},{values[7]},{values[8]}")
            print(values)

            timePrev = time.time()