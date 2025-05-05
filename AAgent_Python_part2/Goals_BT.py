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
        Moves forward till it finds an obstacle. Then stops.
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
    
    async def run(self):
        try:

            print('__________________________RETURNING TO RANDOM ROAM')

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
                        print(f"Moving forward for {self.movement_time:.1f} seconds")
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
                        
                        print(f"Turning {turn_degrees} degrees {self.turn_direction}")
                        await self.a_agent.send_message("action", self.turn_command)
                        
                    '''
                    elif self.state == self.MOVING:
                        # Check if obstacle detected within a safe distance (0.5 units)
                        sensor_hits = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.HIT]
                        sensor_distances = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.DISTANCE]
                        
                        obstacle_detected = False
                        for i, hit in enumerate(sensor_hits):
                            if hit and sensor_distances[i] < 1.00:
                                obstacle_detected = True
                                break
                        
                        # If obstacle detected or movement time elapsed, stop and decide next action
                        if obstacle_detected or (current_time - self.movement_start) >= self.movement_time:
                            await self.a_agent.send_message("action", "stop")
                            self.state = self.DECIDING
                    '''
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
            print("***** TASK RandomRoam CANCELLED")
            await self.a_agent.send_message("action", "stop")
            await self.a_agent.send_message("action", "nt")
            self.state = self.DECIDING



class Avoid:
    """
    The Drone advances while avoiding obstacles, ensuring it does not collide with objects,
    including exterior walls.
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
        self.safe_distance = 1.0  # Safe distance to obstacles
    
    async def run(self):
        try:
            # Start moving forward
            self.state = self.MOVING
            
            while True:
                print("RETURNING TO AVOID")
                await self.a_agent.send_message("action", "mf")
                # Get sensor data
                sensor_hits = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.HIT]
                sensor_distances = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.DISTANCE]
                sensor_angles = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.ANGLE]
                
                
                if self.state == self.MOVING:
                    print("enter moving state")
                    # Check for obstacles within safe distance

                    obstacle_detected = False
                    center_ray_index = len(sensor_hits) // 2
                    
                    # Create a list of tuples (angle, distance) for rays that detected obstacles
                    obstacle_info = []
                    for i, hit in enumerate(sensor_hits):
                        
                        if hit and sensor_distances[i] < self.safe_distance and sensor_distances[i] > 0:
                        
                            obstacle_info.append((sensor_angles[i], sensor_distances[i]))
                            obstacle_detected = True
                    
                    if obstacle_detected:
                        # Stop moving forward
                        print ("obstacle detected")
                        
                        await self.a_agent.send_message("action", "stop")
                        
                        # Determine which direction to turn based on obstacle positions
                        left_obstacles = [info for info in obstacle_info if info[0] < 0]
                        right_obstacles = [info for info in obstacle_info if info[0] > 0]
                        
                        # Calculate average distances on each side
                        left_avg_dist = sum([info[1] for info in left_obstacles]) / len(left_obstacles) if left_obstacles else float('inf')
                        right_avg_dist = sum([info[1] for info in right_obstacles]) / len(right_obstacles) if right_obstacles else float('inf')
                        
                        # Decide which way to turn
                        if left_avg_dist > right_avg_dist:
                            self.avoid_direction = "left"
                            print("Obstacle detected! Turning left")
                        else:
                            self.avoid_direction = "right"
                            print("Obstacle detected! Turning right")
                        
                        self.avoid_start_rotation = self.i_state.rotation["y"]
                        self.state = self.AVOIDING
                
                elif self.state == self.AVOIDING:
                    # Continue turning until at least 45° rotation is achieved
                    current_rotation = self.i_state.rotation["y"]
                    rotation_diff = abs(current_rotation - self.avoid_start_rotation)
                    
                    # Handle wraparound
                    if rotation_diff > 180:
                        rotation_diff = 360 - rotation_diff

                    if rotation_diff < 45:
                        # Keep turning in the same direction
                        await self.a_agent.send_message("action", "tl" if self.avoid_direction == "left" else "tr")
                    else:
                        # Stop turning
                        await self.a_agent.send_message("action", "nt")

                        # Recheck sensor data
                        sensor_hits = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.HIT]
                        sensor_distances = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.DISTANCE]

                        front_rays_clear = True
                        center_ray_index = len(sensor_hits) // 2
                        check_range = 1
                        for i in range(center_ray_index - check_range, center_ray_index + check_range + 1):
                            if 0 <= i < len(sensor_hits):
                                if sensor_hits[i] and sensor_distances[i] < self.safe_distance:
                                    front_rays_clear = False
                                    break

                        if front_rays_clear:
                            print("CLEARED")
                            await self.a_agent.send_message("action", "tl")
                            await asyncio.sleep(0.1)
                            return True
                        else:
                            # Restart avoidance turn with updated rotation base
                            self.avoid_start_rotation = current_rotation

                
        except asyncio.CancelledError:
            print("***** TASK Avoid CANCELLED")
            await self.a_agent.send_message("action", "stop")
            await self.a_agent.send_message("action", "nt")
            self.state = self.MOVING




class FollowAstronaut:
    '''
    Description: Class that represents the action of following an astronaut
    '''

    # Class constants
    MOVING = 0 # Moving state
    TURNING = 1 # Turning state
    RIGHT = 1 # Turn right
    LEFT = -1 # Turn left

    def __init__(self, a_agent):
        '''
        init method for FollowAstronaut class
        Input: a_agent: Agent object, the agent that will execute the action (in this case, follow an astronaut)
        '''
        # get the agent object
        self.a_agent = a_agent
        # get the agent's sensors
        self.rc_sensor = a_agent.rc_sensor
        # get the agent's internal state
        self.i_state = a_agent.i_state
        # set the rotation amount (by default None)
        self.rotation_amount = None
        # set the previous rotation
        self.prev_rotation = 0
        # set the accumulated rotation
        self.accumulated_rotation = 0
        # set the direction to turn (by default right)
        self.direction = self.RIGHT
        # set the state of the agent to MOVING
        self.state = self.MOVING
        
    async def run(self):
        '''
        Use asyncio to run the action of following an astronaut
        Output: True, when the action is done, in this case, after the agent has followed the astronaut
        '''
        # try to run the action
        try:
   
            while True:
                # if the agent is in the MOVING state
                if self.state == self.MOVING:
                    # Check if any of the rays hits an astronaut, using the detection sensor that we get from the BTCritter.py's detect_astronaut class
                    if self.rc_sensor.sensor_rays[Sensors.RayCastSensor.HIT][self.a_agent.det_sensor]:
                        # if the sensor that detects the astronaut is on the left side of the agent (0-4)
                        if self.a_agent.det_sensor < 5:
                            # Turn left by the angle given by the sensor that detects the astronaut,
                            # if the sensor is 0, turn left by -90 degrees, if it is 4, turn left by -18 degrees
                            turn_angle = -90 + self.a_agent.det_sensor * (90 / 5)
                            # Send the message "tl" to the agent (turn left)
                            await self.a_agent.send_message("action", f"tl")
                        # if the sensor that detects the astronaut is on the right side of the agent (6-10)
                        elif  self.a_agent.det_sensor >5:
                            # Turn right by the angle given by the sensor that detects the astronaut,
                            # if the sensor is 6, turn right by 18 degrees, if it is 10, turn right by 90 degrees
                            turn_angle = self.a_agent.det_sensor * (90 / 5)
                            # Send the message "tr" to the agent (turn right)
                            await self.a_agent.send_message("action", f"tr")
                        # if the sensor that detects the astronaut is in the middle of the agent (5), just move forward, no need to turn
                        else:
                            #set the turn angle to 0 because, if not, we'd get an error of referencing before assignment
                            turn_angle = 0
                            # Send the message "mf" to the agent (move forward)
                            await self.a_agent.send_message("action", f"mf")
                        #set a sleep time to wait for the agent to turn  or move forward
                        await asyncio.sleep(0.15)
                        #go forward another time (this was added empirically to make the agent follow the astronaut better)
                        await self.a_agent.send_message("action", "mf")

                    """
                    # If the agent is not in front of an astronaut, but is inside this action, means that the agent recently saw the astronaut
                    #so we go in the direction that we followed the astronaut last time, this is part of the bonus task.
                    else:
                        # Send the message "mf" to the agent (move forward) a couple of times
                        await self.a_agent.send_message("action", "mf")
                        await self.a_agent.send_message("action", "mf")
                    """

                    #set previous rotation of the agent to the current rotation 
                    self.prev_rotation = self.i_state.rotation["y"]
                    #set the accumulated rotation to 0
                    self.accumulated_rotation = 0
                    #set the state to TURNING
                    self.state = self.TURNING
                # if the agent is in the TURNING state
                elif self.state == self.TURNING:
                    # Get the current rotation of the agent in the y-axis
                    current_rotation = self.i_state.rotation["y"]
                    # If the direction is right
                    if self.direction == self.RIGHT:
                        # Calculate the rotation change by subtracting the previous rotation from the current rotation and adding 360 to avoid
                        # negative values, then take the modulo 360 so we get the correct value and never does more than an entire rotation (360 degrees)
                        rotation_change = (current_rotation - self.prev_rotation + 360) % 360
                        # Update the accumulated rotation by adding the rotation change
                        self.accumulated_rotation += rotation_change
                    # If the direction is left
                    elif self.direction == self.LEFT:
                        # Calculate the rotation change by subtracting the current rotation from the previous rotation and adding 360 to avoid
                        # negative values, then take the modulo 360 so we get the correct value and never does more than an entire rotation (360 degrees)
                        rotation_change = (self.prev_rotation - current_rotation + 360) % 360
                        # Update the accumulated rotation by adding the rotation change
                        self.accumulated_rotation += rotation_change
                    # Set the previous rotation to the current rotation
                    self.prev_rotation = current_rotation
                    # If the accumulated rotation is greater than or equal to the rotation amount
                    if self.accumulated_rotation >= abs(turn_angle):
                        # Send the message "nt" to the agent (no turn) as the turn is done
                        await self.a_agent.send_message("action", "nt")
                        # Reset the accumulated rotation to 0
                        self.accumulated_rotation = 0
                        # Set the direction to right (default)
                        self.direction = self.RIGHT
                        # Set the state to MOVING
                        self.state = self.MOVING
                        # Return True when the action is done
                        return True
                    # Sleep for 0 seconds and keep running the action
                    await asyncio.sleep(0)

        # If the action is cancelled
        except asyncio.CancelledError:
            # Print a message to the terminal
            print("***** TASK Follow CANCELLED")
            # Send the message "nt" to the agent (no turn)
            await self.a_agent.send_message("action", "nt")