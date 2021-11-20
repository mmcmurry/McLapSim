import Car, Powertrain
from utils import *
from Sim import *
import numpy as np

# Create tracks
autoxTrack = readTrackFile("Tracks/2017 autocross map.txt")
enduroTrack = readTrackFile("Tracks/2017 endurance map.txt")
skidpad_track = [[8.12, 360, 0]]

# Create Powertrain object
R62006_curve = np.matrix([[3000, 12000], [4125.0, 18642.0], [6000, 35000], [8000, 42500]])
R62006_curve = np.matrix([[3000, 13800], [4125, 21438], [6000, 40250], [9000, 54625]])
powertrain = Powertrain(R62006_curve, [2.583, 2.0, 1.67, 1.444, 1.286, 1.150], 42.0/13.0, 0.2286)

# Create Car object
m_car = 234.0   # kg
m_driver = 75.0 # kg
m_aero = 12     # kg
mu = 1.31985
a_brake17 = 1.2 * 9.81 # m/s^2
a_brake18 = 1.3 * 9.81 # m/s^2
CLA = 2     # m^2
CDA = 1     # m^2
DAir = 1.23 # kg/m^3
Jinx = Car("Jinx", m_car+m_driver+m_aero, powertrain, mu, a_brake, CLA, CDA, DAir)

# Simulate car
findDynamicTimes(Jinx, True, autoxTrack, enduroTrack)