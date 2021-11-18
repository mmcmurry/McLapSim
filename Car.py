from utils import *

class Car:
    def __init__(self, name, m, P, mu, width, a_brake, df, drag):
        self.name = name # Just the name of the instance
        self.mass = m # Mass of the car with fuel and driver, in kilograms
        self.power = P # Average power of the engine, calculated using acceleration event times, in Watts
        self.mu = mu # Coefficient of static friction
        self.a_brake = a_brake # Longitudinal acceleration under peak braking
        self.df = df # A length 3 array with quadratic coefficients
        self.df += findVertex(df[0],df[1],df[2])[::-1]
        self.drag = drag # Same as self.df ... [a, b, c]
        self.drag += findVertex(drag[0],drag[1],drag[2])[::-1]
        self.width = width # Width of the car in meters
        self.dt = 0.001        
    
    def getDownforce(self, velocity):
        # Returns downforce amount (positive) in Newtons given a velocity using a linear/quadratic model of downforce
        # If v is higher than vertex of downforce parabola, use quadratic model of downforce. Otherwise use linear
        if (velocity >= self.df[4]):
            return self.df[0]*velocity**2 + self.df[1]*velocity + self.df[2]
        # If v is lower than the apex, use the linear model
        else:
            return velocity*self.df[3]/self.df[4]

    def getDrag(self, velocity):
        # Returns drag force (positive) in Newtons given a velocity using a linear/quadratic model of drag
        # Basically identical to Car.getDownforce()
        if (velocity >= self.drag[4]):
            return self.drag[0]*velocity**2 - self.drag[1]*velocity + self.drag[2]
        else:
            return velocity*self.drag[3]/self.drag[4]
    
    def findCorneringSpeed(self, radius):
        # Returns the maximum speed in meters per second that the car could go through a corner of given radius
        # Finds speed where centripetal force = cornering force. This formula can be reduced into a quadratic and solved

        radius = radius + 0.5*self.width
        
        a = self.mass/radius - self.mu*self.df[0]
        b = self.mu*self.df[1]
        c = -1*self.mu*(self.mass*9.81 + self.df[2])
        
        v = max(quadraticFormula(a,b,c))

        if v > self.df[4]:
            return v
        else:
            a = 1
            b = -radius/self.mass * self.mu * self.df[3]/self.df[4]
            c = -radius * self.mass * self.mu * 9.81
            return max(quadraticFormula(a,b,c))

    def findCorneringTime(self, radius, theta):
        # Returns time to go through a corner in seconds given the radius of the turn in meters and the angle of the turn
        # Uses Car.findCorneringSpeed() to find velocity through turn, then computes time through turn with t = d/v
        v = self.findCorneringSpeed(radius)
        radius = radius + 0.5*self.width
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
			max_traction = self.mu * (self.mass*9.81 + self.getDownforce(v)) # The most friction the tires can supply without slipping
			throttle_percent = max_traction/max_force # Set the 'throttle' so that the car doesn't spin the tires
			if throttle_percent > 1: # Don't let throttle_percent go over 1 or under 0
				throttle_percent = 1
			elif throttle_percent < 0:
				throttle_percent = 0
			
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