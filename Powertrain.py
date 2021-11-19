class Powertrain:
	def __init__(self, power_curve, gear_ratios, crank_sprocket_ratio, tire_radius):
		self.power_curve = power_curve # A list of length 2 lists that are points on the power curve
		self.min_rpm = power_curve[0][0]
		self.max_rpm = power_curve[len(power_curve)-1][0]
		self.min_power = power_curve[0][1]
		self.max_power = power_curve[len(power_curve)-1][1]
		self.gear_ratios = gear_ratios
		self.crank_sprocket_ratio = crank_sprocket_ratio
		self.tire_radius = tire_radius
		
		for i in range(len(power_curve)-1):
			rise = power_curve[i][1]-power_curve[i+1][1]
			run = power_curve[i][0]-power_curve[i+1][0]
			slope = rise/run
			power_curve[i].append(slope) # The slope from point i to point i+1
		
		self.rpm = self.min_rpm
		self.gear = 1
	
	def overrev(self):
		if self.rpm > self.max_rpm:
			return True
		else:
			return False
	
	def underrev(self):
		if self.rpm < self.min_rpm:
			return True
		else:
			return False
	
	def setRPM(self, velocity):
	# Given velocity, find RPM based on gearing. Haven't been able to get this to put out realistic numbers
		self.rpm = velocity/self.tire_radius * self.gear_ratios[self.gear-1] * self.crank_sprocket_ratio * 60 / (2*pi)
		if self.rpm < self.min_rpm and self.gear == 1: # Don't let the engine 'stall'
			self.rpm = self.min_rpm
		
		return self.rpm
	
	def selectGear(self, velocity):
	# Given velocity, select the correct gear based on minimum and maximum rpms
		self.setRPM(velocity)
		while(self.overrev() or self.underrev()):
			# Upshift if revs are too high and not in final gear
			if self.overrev():
				if self.gear != len(self.gear_ratios):
					self.gear += 1
				else:
					#print("Redline in 6th!")
					return self.gear
			# Downshift if revs are too low and not in 1st gear		
			elif self.underrev():
				if self.gear != 1:
					self.gear -= 1
				else:
					#print("Engine stalling in 1st!")
					return self.gear
			self.setRPM(velocity)
					
		print("Gear: " + str(self.gear))
		return self.gear	
	
	def getPower(self):
	# Returns power in Watts based on current rpm of the engine. Uses linear interpolation between points in power_curve
	#!!! Doesn't give any power if less than min rpm !!!
	#y=m(x-x1)+y1
		power = 0
		for i in range(len(self.power_curve)-1):
			if self.rpm >= self.power_curve[i][0] and self.rpm < self.power_curve[i+1][0]:
				power = self.power_curve[i][2] * (self.rpm - self.power_curve[i][0]) + self.power_curve[i][1]
			
		return power
	
	def getTorque(self):
	# Finds torque in N*m from power at current rpm 
		torque = self.getPower()/(self.rpm*((2*pi)/60))
		return torque 
	
	def getWheelForce(self):
	# Converts torque from the engine to force on the ground
		force = self.getTorque() * self.gear_ratios[self.gear-1] * self.crank_sprocket_ratio / self.tire_radius
		return force 