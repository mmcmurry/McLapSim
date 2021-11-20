def findLapTime(car, track, ax_bool=None):
    # Calculates a lap time given a track. ax_bool = true will make it so that the car accelerates to the finish, rather than slow down as if there was a turn
    # Tracks are 2d lists, where each inner list is a turn made up of [radius, degrees, distance to next turn]. The direction of the turn doesn't matter
    t = 0
    if not ax_bool:
        ax_bool = False
    # Finds corner velocities for vor every turn (for calculating straight times) and finds cornering times and adds
    # them to the total lap time
    corner_speeds = []
    for turn in track:
        corner_speeds.append(car.findCorneringSpeed(turn[0]))
        t += car.findCorneringTime(turn[0], turn[1])
        
    # Iterate through turns to find time spent in the straights
    for turn in range(len(track)):
        if turn == len(track)-1:
            next_turn = 0
        else:
            next_turn = turn + 1
        
        # If it's an autocross track, don't slow down at the end
        if ax_bool and next_turn == 0:
            t += car.findStraightTime(corner_speeds[turn], -1, track[turn][2])
        else:
            t += car.findStraightTime(corner_speeds[turn], corner_speeds[next_turn], track[turn][2])
        
    return t
            
def findDynamicTimes(self, p_bool, autox_track, endurance_track):
    # Calculates times in acceleration, skidpad, autocross, and endurance using car parameters and points data from Lincoln 2017
    times = []
    times.append(round(self.findStraightTime(0, -1, 75), 3))
    times.append(round(self.findCorneringTime(8.12, 360), 3))
    times.append(round(self.findLapTime(autox_track, True), 3))
    times.append(round(self.findLapTime(endurance_track) * 16, 3)) # Endurance is 16 laps, so just multiply the time of one lap times 16
    if p_bool:
        print(self.name + ' dynamic event times: ')
        print('Accel: ' + str(times[0]) + '\tSkidpad: ' + str(times[1]) + '\tAutoX: ' + str(times[2]) + '\tEnduro: ' + str(times[3]) + '\n')
    return times