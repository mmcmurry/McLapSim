from utils import *
from math import pi

class Car:
    def __init__(self, name, mCar, P, mu, a_brake, cla, cda, DAir):
        self.name = name        # Just the name of the instance
        self.mCar = mCar        # Mass of the car with fuel and driver, in kilograms
        self.power = P          # Average power of the engine, calculated using acceleration event times, in Watts
        self.mu = mu            # Tire coefficient of friction
        self.a_brake = a_brake  # Average longitudinal acceleration when braking
        self.CLA = cla          # Coefficient of lift * area (negative for downforce)
        self.CDA = cda          # Coefficient of drag * area (positive)
        self.DAir = DAir        # Ambient air density (kg/m^3)
        self.dt = 0.001        
    
    def getDownforce(self, v):
        # Returns downforce (N), generally > 0
        # Inputs: v = velocity (m/s); D = air density (kg/m^3)
        return -0.5 * self.CLA * self.DAir * v**2

    def getDrag(self, v):
        # Returns drag (N), generally > 0
        # Inputs: v = velocity (m/s); D = air density (kg/m^3)
        return 0.5 * self.CDA * self.DAir * v**2
    
    def findCorneringSpeed(self, radius):
        # Returns the maximum speed in meters per second that the car could go through a corner of given radius
        # Finds speed where centripetal force = cornering force.
        return sqrt(self.mu*self.mCar*9.81 / (self.mCar/radius - 0.5*self.mu*self.DAir*self.CLA))
        
    def findCorneringTime(self, radius, theta):
        # Returns time to go through a corner in seconds given the radius of the turn in meters and the angle of the turn
        # Uses Car.findCorneringSpeed() to find velocity through turn, then computes time through turn with t = d/v
        v = self.findCorneringSpeed(radius)
        radius = radius
        d = 2*pi*radius * theta/360
        return d/v
    
    def findStraightTime(self, v_i, v_f, length):
        # Figures out how long it takes to go a specified distance starting and ending at specified speeds using numerical integration
        t = 0
        d = 0
        a = 0
        v = v_i
        self.drivetrain.selectGear(v_i)
        
        # While d is less than the length of the straight minus the distance required to slow down, accelerate
        while(d < length - self.findBrakingDistance(v, v_f)):
            self.drivetrain.selectGear(v)

            #Velocity Verlet
            a_prev = a
            d += v*self.dt + (0.5*a_prev*self.dt**2)
            
            # Some calculations to figure out how much throttle to use
            max_force = self.drivetrain.getWheelForce() - self.getDrag(v) # The most force that the engine can supply minus drag
            max_traction = self.mu * (self.mass*9.81 + self.getDownforce(v)/2) # The most friction the tires can supply without slipping
            throttle_percent = max_traction/max_force # Set the 'throttle' so that the car doesn't spin the tires
            throttle_percent = saturate(throttle_percent, 0, 1)
            
            a = (max_force * throttle_percent) / self.mass
            a_avg = (a + a_prev)/2
            v += a_avg*self.dt
            t += self.dt
            
            #Print stuff for debugging
            print("Time: " + str(t))
            print("RPM: " + str(self.drivetrain.rpm) + "\tPower: " + str(self.drivetrain.getPower()) + "\tTorque: " + str(self.drivetrain.getTorque()) + "\tThrottle: " + str(throttle_percent))
            print("Distance: " + str(d) + "\tVelocity: " + str(v) + "\tAccel: " + str(a_avg) + "\n")
                
        # The previous loop exited, so now it is time to slow down by subtracting the braking acceleration from velocity
        #************This needs to be changed (probably dramatically) to switch to a force based model
        while(d<length):
            v -= self.a_brake*self.dt
            d += v*self.dt
            t += self.dt
        
        return t

    def findBrakingDistance(self, v, v_f):
        # Finds distance required to slow from v to v_f using a formula derived using some basic calculus. Contact Matt McMurry for more info
        # Enter -1 for v_f to not include braking (for use in Car.findStraightTime)
        if v_f == -1:
            return 0
        else:
            dv = v-v_f
            return (dv/self.a_brake) * ((dv/2) + v_f)