import math
import random
import asyncio
import Sensors
from collections import Counter

'''
Group 4
Marino Oliveros Blanco (1668563)
Alejandra Reinares Guerreros (1665499)
Andreu Gascón Marzo (1670919)
Pere Mayol Carbonell (1669503)
'''

class DoNothing:
    """
    Does nothing
    """
    def __init__(self, a_agent):
        self.a_agent = a_agent
        self.rc_sensor = a_agent.rc_sensor
        self.i_state = a_agent.i_state

    async def run(self):
        print("Doing nothing")
        await asyncio.sleep(1)
        return True

class ForwardDist:
    """
    Moves forward till it finds an obstacle, then stops.
    """
    STOPPED = 0
    MOVING = 1
    END = 2

    def __init__(self, a_agent):
        self.a_agent = a_agent
        self.rc_sensor = a_agent.rc_sensor
        self.i_state = a_agent.i_state
        self.state = self.STOPPED

    async def run(self):
        try:
            while True:
                if self.state == self.STOPPED:
                    # Start moving
                    await self.a_agent.send_message("action", "mf")
                    self.state = self.MOVING
                elif self.state == self.MOVING:
                    sensor_hits = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.HIT]
                    if any(ray_hit == 1 for ray_hit in sensor_hits):
                        self.state = self.END
                        await self.a_agent.send_message("action", "stop")
                    else:
                        await asyncio.sleep(0)
                elif self.state == self.END:
                    break
                else:
                    print("Unknown state: " + str(self.state))
                    return False
        except asyncio.CancelledError:
            print("***** TASK Forward CANCELLED")
            await self.a_agent.send_message("action", "stop")
            self.state = self.STOPPED

class Turn:
    """
    Randomly selects a degree of turn between 10 and 360, along with a direction (left or right),
    and executes the turn accordingly. Upon completion, selects a new turn and repeats.
    """
    SELECTING = 0
    TURNING = 1
    
    def __init__(self, a_agent):
        self.a_agent = a_agent
        self.rc_sensor = a_agent.rc_sensor
        self.i_state = a_agent.i_state
        self.state = self.SELECTING
        self.target_degrees = 0
        self.target_rotation = 0
        self.direction = ""
        self.turn_command = ""
    
    async def run(self):
        try:
            while True:
                if self.state == self.SELECTING:
                    # Select random degree between 10 and 360
                    self.target_degrees = random.randint(10, 360)
                    # Randomly select direction (left or right)
                    if random.random() < 0.5:
                        self.direction = "left"
                        self.turn_command = "tl"
                        # Calculate target rotation for left turn
                        self.target_rotation = (self.i_state.rotation["y"] - self.target_degrees) % 360
                    else:
                        self.direction = "right"
                        self.turn_command = "tr"
                        # Calculate target rotation for right turn
                        self.target_rotation = (self.i_state.rotation["y"] + self.target_degrees) % 360
                    
                    print(f"Turning {self.target_degrees} degrees to the {self.direction}")
                    await self.a_agent.send_message("action", self.turn_command)
                    self.state = self.TURNING
                
                elif self.state == self.TURNING:
                    current_rotation = self.i_state.rotation["y"]
                    
                    # Check if we've reached the target rotation (with a small tolerance)
                    if self.direction == "left":
                        # For left turns, we need to handle the wraparound from 0 to 360
                        if (abs(current_rotation - self.target_rotation) < 3 or 
                            (current_rotation > 357 and self.target_rotation < 3)):
                            await self.a_agent.send_message("action", "nt")  # Stop turning
                            self.state = self.SELECTING  # Select a new turn
                    else:  # right turn
                        # For right turns, also handle wraparound
                        if (abs(current_rotation - self.target_rotation) < 3 or 
                            (current_rotation < 3 and self.target_rotation > 357)):
                            await self.a_agent.send_message("action", "nt")  # Stop turning
                            self.state = self.SELECTING  # Select a new turn
                            
                await asyncio.sleep(0.1)  # Small delay to prevent busy waiting
                
        except asyncio.CancelledError:
            print("***** TASK Turn CANCELLED")
            await self.a_agent.send_message("action", "nt")  # Stop turning
            self.state = self.SELECTING


class RandomRoam:
    """
    The Drone moves around following a particular direction for a certain duration,
    then changes direction, decides whether to stop, and resumes movement accordingly
    based on predetermined probabilities.
    """
    DECIDING = 0
    MOVING = 1
    TURNING = 2
    STOPPED = 3
    
    def __init__(self, a_agent):
        self.a_agent = a_agent
        self.rc_sensor = a_agent.rc_sensor
        self.i_state = a_agent.i_state
        self.state = self.DECIDING
        self.movement_time = 0
        self.movement_start = 0
        self.turn_target = 0
        self.turn_direction = ""
        self.turn_command = ""
        self.last_command_time = 0
    
    async def run(self):
        try:

            while True:
                current_time = asyncio.get_event_loop().time()
                
                        
                if self.state == self.DECIDING:
                    # Probabilities for different actions
                    action_prob = random.random()
                    
                    if action_prob < 0.7:  # 60% chance to move forward
                        self.state = self.MOVING
                        # Move for 2-5 seconds
                        self.movement_time = random.uniform(2.0, 5.0)
                        self.movement_start = current_time
                        #print(f"Moving forward for {self.movement_time:.1f} seconds")
                        await self.a_agent.send_message("action", "mf")
                        
                    else:  # 30% chance to turn
                        self.state = self.TURNING
                        # Turn 30-180 degrees
                        turn_degrees = random.randint(30, 180)
                        # Choose direction
                        if random.random() < 0.5:
                            self.turn_direction = "left"
                            self.turn_command = "tl"
                            self.turn_target = (self.i_state.rotation["y"] - turn_degrees) % 360
                        else:
                            self.turn_direction = "right"
                            self.turn_command = "tr"
                            self.turn_target = (self.i_state.rotation["y"] + turn_degrees) % 360
                        
                        #print(f"Turning {turn_degrees} degrees {self.turn_direction}")
                        await self.a_agent.send_message("action", self.turn_command)
                        
                elif self.state == self.MOVING:
                        # If movement time elapsed, stop and decide next action
                        if (current_time - self.movement_start) >= self.movement_time:
                            await self.a_agent.send_message("action", "stop")
                            self.state = self.DECIDING

                elif self.state == self.TURNING:
                    current_rotation = self.i_state.rotation["y"]
                    
                    # Check if we've reached the target rotation (with tolerance)
                    if abs(current_rotation - self.turn_target) < 5 or (abs(current_rotation - self.turn_target) > 355):
                        await self.a_agent.send_message("action", "nt")  # Stop turning
                        self.state = self.DECIDING
                
                # Small delay to prevent busy waiting
                await asyncio.sleep(0.1)
                
        except asyncio.CancelledError:
            #print("***** TASK RandomRoam CANCELLED")
            await self.a_agent.send_message("action", "stop")
            await self.a_agent.send_message("action", "nt")
            self.state = self.DECIDING



class Avoid:
    """
    The agent moves forward and, upon detecting nearby obstacles, determines a safe direction and rotates to avoid collisions.
    """
    MOVING = 0
    AVOIDING = 1
    
    def __init__(self, a_agent):
        self.a_agent = a_agent
        self.rc_sensor = a_agent.rc_sensor
        self.i_state = a_agent.i_state
        self.state = self.MOVING
        self.avoid_direction = ""
        self.avoid_start_rotation = 0
        self.safe_distance = 1.0
        self.last_command_time = 0  #Record the time of the last command sent to control action frequenc

    async def run(self):
        try:
            while True:
                # Check elapsed time to throttle command rate (avoid spamming Unity)
                current_time = asyncio.get_event_loop().time()
                
                # Only proceed if enough time has passed since last command (avoids conflicts with unity)
                if current_time - self.last_command_time < 0.1:  # Send commands every 0.1s
                    await asyncio.sleep(0.01)
                    continue
                    
                self.last_command_time = current_time
                
                #Retrieve sensor data: obstacle hits, distances, and angles
                sensor_hits = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.HIT]
                sensor_distances = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.DISTANCE]
                sensor_angles = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.ANGLE]
                
                if self.state == self.MOVING:
                    await self.a_agent.send_message("action", "mf")

                    obstacle_info = []
                    for i, (hit, dist) in enumerate(zip(sensor_hits, sensor_distances)):
                        if hit and 0.1 < dist < self.safe_distance:
                            obstacle_info.append((sensor_angles[i], dist))
                    
                    #If obstacles are detected, prepare for avoidance
                    if obstacle_info:
                        await self.a_agent.send_message("action", "stop")
                        
                        #Categorize obstacles to left and right based on angle
                        left_obstacles = [info for info in obstacle_info if info[0] < 0]
                        right_obstacles = [info for info in obstacle_info if info[0] > 0]
                        
                        #Compute average distance to obstacles on each side
                        left_avg_dist = sum(d for _,d in left_obstacles)/len(left_obstacles) if left_obstacles else float('inf')
                        right_avg_dist = sum(d for _,d in right_obstacles)/len(right_obstacles) if right_obstacles else float('inf')
                        
                        # Choose avoidance direction based on which side has more clearance
                        if left_avg_dist > right_avg_dist:
                            self.avoid_direction = "left"
                        else:
                            self.avoid_direction = "right"
                            
                        self.avoid_start_rotation = self.i_state.rotation["y"]
                        self.state = self.AVOIDING
                
                elif self.state == self.AVOIDING:

                    #Send continuous turn command in the chosen avoidance direction
                    await self.a_agent.send_message("action", "tl" if self.avoid_direction == "left" else "tr")
                    
                    #Rotation check
                    current_rot = self.i_state.rotation["y"]
                    rot_diff = abs(current_rot - self.avoid_start_rotation)
                    rot_diff = min(rot_diff, 360 - rot_diff)
                    
                    if rot_diff >= 45:
                        await self.a_agent.send_message("action", "nt")
                        
                        # clearance check
                        front_clear = not any(
                            hit and dist < self.safe_distance
                            for i, (hit, dist) in enumerate(zip(sensor_hits, sensor_distances))
                            if abs(sensor_angles[i]) < 45
                        )
                        
                        if front_clear:
                            return True
                        else:
                            #Continue avoiding from new angle
                            self.avoid_start_rotation = current_rot
                
                await asyncio.sleep(0.01)  # Small delay

        #------------------------------------------------------------------       
        except asyncio.CancelledError:
            await self.a_agent.send_message("action", "nt")
            await self.a_agent.send_message("action", "stop")
            raise
        #------------------------------------------------------------------


class FollowAstronaut:
    '''
    Represents an agent following an astronaut using sensor data to navigate.
    '''

    MOVING = 0 
    TURNING = 1 
    RIGHT = 1 
    LEFT = -1 

    def __init__(self, a_agent):
        self.a_agent = a_agent
        self.i_state = a_agent.i_state
        self.rc_sensor = a_agent.rc_sensor
        self.rotation_amount = None
        self.accumulated_rotation = 0
        self.prev_rotation = 0
        self.direction = self.RIGHT
        self.state = self.MOVING
        
    async def run(self):
        '''
        Uses asyncio to execute the action of following an astronaut
        '''
        try:
   
            while True: 
                # Check if the agent is in the MOVING state
                if self.state == self.MOVING:
                    # Check if any ray hits an astronaut using the detection sensor
                    if self.rc_sensor.sensor_rays[Sensors.RayCastSensor.HIT][self.a_agent.det_sensor]:
                        # Determine the turn angle based on the detected astronaut position
                        if(self.a_agent.det_sensor<5):
                            #The angle that the critter turns depends on the angle of the ray that detects the astronaut
                            #-----------------------------------------------------------------------
                            turning_angle = -90 + self.a_agent.det_sensor * (90 / 5)
                            #-----------------------------------------------------------------------
                            await self.a_agent.send_message("action", "tl") #Turn left

                        elif(self.a_agent.det_sensor>5):
                            turning_angle = self.a_agent.det_sensor * (90 / 5)
                            await self.a_agent.send_message("action", "tr") #Turn right
                        
                        else:
                            turning_angle = 0 #Used to avoid errors
                            await self.a_agent.send_message("action", "mf") # Move forward

                        await asyncio.sleep(0.15) # Small delay to prevent busy waiting
                        await self.a_agent.send_message("action", "mf")

                    self.prev_rotation = self.i_state.rotation["y"]
                    self.accumulated_rotation = 0
                    self.state = self.TURNING

                #If the agent is in the TURNING state
                elif self.state == self.TURNING:
                    current_rotation = self.i_state.rotation["y"]

                    if self.direction == self.RIGHT:
                        new_rotation = (current_rotation - self.prev_rotation + 360) % 360 #This avoids negative values
                        self.accumulated_rotation += new_rotation

                    elif self.direction == self.LEFT:
                        new_rotation = (self.prev_rotation - current_rotation + 360) % 360
                        self.accumulated_rotation += new_rotation

                    self.prev_rotation = current_rotation

                    if self.accumulated_rotation >= abs(turning_angle):
                        await self.a_agent.send_message("action", "nt") #Send "nt" for no turn
                        self.accumulated_rotation = 0 #Reset accumulated rotation
                        self.direction = self.RIGHT #Reset direction to default (right)
                        self.state = self.MOVING #Set state back to MOVING
                        return True
                    
                    await asyncio.sleep(0)

        except asyncio.CancelledError:
            print("***** TASK Follow CANCELLED")
            await self.a_agent.send_message("action", "nt")