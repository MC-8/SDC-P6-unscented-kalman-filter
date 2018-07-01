import matplotlib.pyplot as plt
import numpy as np

NIS_radar_f = open('NIS_radar.txt', 'r')
NIS_radar = []
for val in NIS_radar_f.read().split():
    NIS_radar.append(float(val))
NIS_radar_f.close()
xs = np.linspace(1,1,len(NIS_radar))
plt.figure(1)
plt.plot(NIS_radar)
plt.plot(xs*7.815)

plt.show()


NIS_laser_f = open('NIS_laser.txt', 'r')
NIS_laser = []
for val in NIS_laser_f.read().split():
    NIS_laser.append(float(val))
NIS_laser_f.close()
plt.figure(2)
ax = plt.plot(NIS_laser)
xs = np.linspace(1,1,len(NIS_laser))
plt.plot(NIS_laser)
plt.plot(xs*5.991)
plt.show()
