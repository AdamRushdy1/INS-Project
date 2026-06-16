# Plot state log from live INS

import matplotlib.pyplot as plt

init = False

f = open("log.txt")

f.readline()

lines = f.readlines()

t = []
roll = []
pitch = []
yaw = []
xPos = []
yPos = []
zPos = []
xVel = []
yVel = []
zVel = []

for line in lines:
    vals = line.split(",")
    vals[9] = vals[9].strip()
    if not init:
        t0 = vals[0]
        h, m, s = t0.split(":")
        t0 = float(h)*3600 + float(m)*60 + float(s)
        init = True

    h, m, s = vals[0].split(":")
    t.append(float(h)*3600 + float(m)*60 + float(s) - t0)

    roll.append(float(vals[7]))
    pitch.append(float(vals[8]))
    yaw.append(float(vals[9]))
    
    xPos.append(float(vals[1]))
    yPos.append(float(vals[2]))
    zPos.append(float(vals[3]))
    xVel.append(float(vals[4]))
    yVel.append(float(vals[5]))
    zVel.append(float(vals[6]))

fig, ax = plt.subplots(5, 1, figsize=(9, 9))

ax[0].plot(t, roll)
ax[0].set_title('Roll')
ax[0].set_ylim([0, 360])

ax[1].plot(t, pitch)
ax[1].set_title('Pitch')
ax[1].set_ylim([0, 360])

ax[2].plot(t, yaw)
ax[2].set_title('Yaw')

ax[3].plot(t, xPos, label="X")
ax[3].plot(t, yPos, label="Y")
ax[3].plot(t, zPos, label="Z")
ax[3].legend()
ax[3].set_title('Position')

ax[4].plot(t, xVel, label="X")
ax[4].plot(t, yVel, label="Y")
ax[4].plot(t, zVel, label="Z")
ax[4].legend()
ax[4].set_title('Velocity')

plt.tight_layout()
plt.show()