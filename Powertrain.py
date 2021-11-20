from math import pi
import numpy as np
from utils import *

class Powertrain:
	def __init__(self, PEngineTable, rGearRatios, rFinalDriveRatio, rTire):
		self.PEngineTable = PEngineTable # A numpy matrix with col0=nEngine, col1=PEngine
		self.nEngineMin = PEngineTable[0,1]
		self.nEngineMax = PEngineTable[0,-1]
		self.PEngineMin = PEngineTable[:,1].min()
		self.PEngineMax = PEngineTable[:,1].max()
		self.rGearRatios = rGearRatios
		self.rFinalDriveRatio = rFinalDriveRatio
		self.rTire = rTire
		
		# for i in range(len(PEngineTable)-1):
		# 	dP_dn = (PEngineTable[i][1]-PEngineTable[i+1][1]) / (PEngineTable[i][0]-PEngineTable[i+1][0])
		# 	PEngineTable[i].append(dP_dn) # The slope from point i to point i+1
		
		self.rpm = self.nEngineMin
		self.gear = 1
	
	def overrev(self):
		return self.rpm > self.nEngineMax
	
	def underrev(self):
		return self.rpm < self.nEngineMin
	
	def setRPM(self, velocity):
	# Given velocity, find RPM based on gearing. Haven't been able to get this to put out realistic numbers
		self.rpm = velocity/self.rTire * self.rGearRatios[self.gear-1] * self.rFinalDriveRatio * 60 / (2*pi)
		if self.rpm < self.nEngineMin and self.gear == 1: # Don't let the engine 'stall'
			self.rpm = self.nEngineMin
		if self.rpm > self.nEngineMax and self.gear == len(self.rGearRatios)-1: # Don't let the engine overrev
			self.rpm = self.nEngineMax
		
		return self.rpm
	
	def selectGear(self, velocity):
	# Given velocity, select the correct gear based on minimum and maximum rpms
		self.setRPM(velocity)
		# Upshift if revs are too high and not in final gear
		if self.overrev():
			if self.gear != len(self.rGearRatios):
				self.gear += 1
				self.setRPM(velocity)
				return self.gear
			else:
				#print("Redline in 6th!")
				return self.gear
		
		# Downshift if revs are too low and not in 1st gear		
		elif self.underrev():
			if self.gear != 1:
				self.gear -= 1
				self.setRPM(velocity)
				return self.gear
			else:
				#print("Engine stalling in 1st!")
				return self.gear
			
		#print("Gear: " + str(self.gear))
		return self.gear	
	
	def getPower(self):
	# Returns power in Watts based on current rpm of the engine. Uses linear interpolation between points in PEngineTable
	#!!! Doesn't give any power if less than min rpm !!!
	#y=m(x-x1)+y1
		return interp2(self.PEngineTable[:,0], self.PEngineTable[:,1], self.rpm, False)
		# power = 0
		# for i in range(len(self.PEngineTable)-1):
		# 	if self.rpm >= self.PEngineTable[i][0] and self.rpm < self.PEngineTable[i+1][0]:
		# 		power = self.PEngineTable[i][2] * (self.rpm - self.PEngineTable[i][0]) + self.PEngineTable[i][1]
			
		# return power
	
	def getTorque(self):
	# Finds torque in N*m from power at current rpm 
		torque = self.getPower()/(self.rpm*((2*pi)/60))
		return torque 
	
	def getWheelForce(self):
	# Converts torque from the engine to force on the ground
		force = self.getTorque() * self.rGearRatios[self.gear-1] * self.rFinalDriveRatio / self.rTire
		return force 