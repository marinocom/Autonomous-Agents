import math
import random
import asyncio
import Sensors
from collections import Counter

"""
Group 4
Marino Oliveros Blanco (1668563)
Alejandra Reinares Guerreros (1665499)
Andreu Gascón Marzo (1670919)
Pere Mayol Carbonell (1669503)
"""

class AvoidCritter:
    """
    Detects and avoids critters (MantaRays) by moving away from them
    """
    
    SEARCHING = 0    # Looking for critters
    SCAPING = 1      # Moving away from a detected critter
    SAFE = 2         # Successfully avoided critter
    
    def __init__(self, a_agent):
        self.a_agent = a_agent
        self.rc_sensor = a_agent.rc_sensor
        self.i_state = a_agent.i_state
        self.state = self.SEARCHING
        self.target_critter = None
        self.target_index = -1
        self.last_rotation_check = 0
        self.rotation_check_interval = 0.3
        self.scape_start_time = 0
        self.scape_duration = 2.0  # How long to move away from critter
        
    async def run(self):
        try:
            print("Starting critter avoidance behavior")
            while True:
                if self.state == self.SEARCHING:
                    # Find a critter in the sensor data
                    sensor_obj_info = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.OBJECT_INFO]
                    
                    for index, value in enumerate(sensor_obj_info):
                        if value and value["tag"] == "CritterMantaRay":
                            self.target_critter = value
                            self.target_index = index
                            print(f"Found Critter at sensor index {index}, switching to SCAPING mode")
                            self.state = self.SCAPING
                            self.scape_start_time = asyncio.get_event_loop().time()
                            await self.a_agent.send_message("action", "mb")  # Start moving backward immediately
                            break
                    
                    if self.state == self.SEARCHING:
                        await asyncio.sleep(0.1)
                        continue
                
                elif self.state == self.SCAPING:
                    current_time = asyncio.get_event_loop().time()
                    
                    # Check if we've been escaping long enough
                    if current_time - self.scape_start_time > self.scape_duration:
                        print("Successfully avoided critter")
                        await self.a_agent.send_message("action", "stop")  # STOP MOVING
                        self.state = self.SAFE
                        break
                    
                    # Get current sensor data
                    sensor_obj_info = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.OBJECT_INFO]
                    
                    # Check if critter is still visible
                    critter_still_visible = False
                    for index, value in enumerate(sensor_obj_info):
                        if value and value["tag"] == "CritterMantaRay":
                            critter_still_visible = True
                            self.target_index = index
                            break
                    
                    # Adjust direction if critter is still visible
                    if critter_still_visible:
                        center_index = len(sensor_obj_info) // 2
                        if current_time - self.last_rotation_check >= self.rotation_check_interval:
                            self.last_rotation_check = current_time
                            
                            if self.target_index < center_index:
                                # Critter on left -> turn right
                                print("Critter on left, turning right to escape")
                                await self.a_agent.send_message("action", "tr")
                                await asyncio.sleep(0.2)
                                await self.a_agent.send_message("action", "nt")
                            else:
                                # Critter on right -> turn left
                                print("Critter on right, turning left to escape")
                                await self.a_agent.send_message("action", "tl")
                                await asyncio.sleep(0.2)
                                await self.a_agent.send_message("action", "nt")
                
                elif self.state == self.SAFE:
                    break
                
                await asyncio.sleep(0.1)
            
            return True
            
        except asyncio.CancelledError:
            print("***** TASK AvoidCritter CANCELLED")
            await self.a_agent.send_message("action", "stop")
            await self.a_agent.send_message("action", "nt")
            return False


class FlowerCollector:
    """
    Moves the astronaut to a detected flower and collects it with improved targeting
    """
    SEARCHING = 0    # Looking for a flower
    PURSUING = 1     # Moving toward a known flower
    COLLECTING = 2   # Actively collecting a flower
    END = 3          # Collection complete
    
    def __init__(self, a_agent):
        self.a_agent = a_agent
        self.rc_sensor = a_agent.rc_sensor
        self.i_state = a_agent.i_state
        self.state = self.SEARCHING
        self.target_flower = None
        self.collection_started = False
        self.last_rotation_check = 0
        self.rotation_check_interval = 0.3  # Check rotation every 0.3 seconds (cambiar si no es poco eficiente)
        
    async def run(self):
        try:
            print("Starting flower collection behavior")
            while True:
                if self.state == self.SEARCHING:
                    # Find a flower in the sensor data
                    sensor_obj_info = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.OBJECT_INFO]
                    flower_found = False
                    
                    for index, value in enumerate(sensor_obj_info):
                        if value and value["tag"] == "AlienFlower":
                            self.target_flower = value
                            self.target_index = index
                            flower_found = True
                            print(f"Found flower target at sensor index {index}, switching to pursuit mode")
                            self.state = self.PURSUING
                            break
                    
                    if not flower_found:
                        #No flower detected->failure
                        print("No flower detected during search")
                        return False
                
                elif self.state == self.PURSUING:
                    # Find the flower in current sensor data
                    sensor_obj_info = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.OBJECT_INFO]
                    flower_index = -1
                    
                    for index, value in enumerate(sensor_obj_info):
                        if value and value["tag"] == "AlienFlower":
                            flower_index = index
                            self.target_flower = value  # Update target to current sensor data
                            break
                    
                    if flower_index >= 0:
                        # Get sensor data for direction and distance
                        sensor_distances = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.DISTANCE]
                        distance = sensor_distances[flower_index]
                        
                        # Check if we're close enough to collect
                        if distance < 1.5: 
                            print(f"Close to flower (distance: {distance}), collecting...")
                            self.state = self.COLLECTING
                            self.collection_started = asyncio.get_event_loop().time()
                            await self.a_agent.send_message("action", "mf")  # Keep moving to collect
                        else:
                            # We need to turn to face the flower properly
                            center_index = len(sensor_obj_info) // 2
                            
                            # Only check rotation periodically
                            current_time = asyncio.get_event_loop().time()
                            if current_time - self.last_rotation_check >= self.rotation_check_interval:
                                self.last_rotation_check = current_time
                                
                                # Determine if we need to turn and which direction
                                if flower_index < center_index - 1:
                                    # Flower is to the left, turn left
                                    print(f"Flower at index {flower_index}, turning left to align (center: {center_index})")
                                    await self.a_agent.send_message("action", "tl")
                                    await asyncio.sleep(0.1)
                                    await self.a_agent.send_message("action", "mf")
                                elif flower_index > center_index + 1:
                                    # Flower is to the right, turn right
                                    print(f"Flower at index {flower_index}, turning right to align (center: {center_index})")
                                    await self.a_agent.send_message("action", "tr")
                                    await asyncio.sleep(0.1)
                                    await self.a_agent.send_message("action", "mf")
                                else:
                                    # Flower is centered, move forward
                                    print(f"Flower aligned at index {flower_index}, moving forward (distance: {distance})")
                                    await self.a_agent.send_message("action", "nt")  # Stop any turning
                                    await asyncio.sleep(0.05)
                                    await self.a_agent.send_message("action", "mf")
                    else: 
                        # If we've lost the the flower, go into search mode
                        print("Lost sight of flower, searching...")
                        # Do a small turn to try to find the flower again
                        await self.a_agent.send_message("action", "tl")
                        await asyncio.sleep(0.5)
                        await self.a_agent.send_message("action", "nt")
                        self.state = self.SEARCHING
                
                elif self.state == self.COLLECTING:
                    # Check if the flower is still in our sensor data
                    flower_still_exists = False
                    sensor_obj_info = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.OBJECT_INFO]
                    
                    for index, value in enumerate(sensor_obj_info):
                        if value and value["tag"] == "AlienFlower":
                            flower_still_exists = True
                            break
                    
                    # Check if collection completed (flower disappeared or timeout)
                    current_time = asyncio.get_event_loop().time()
                    if not flower_still_exists or (current_time - self.collection_started) > 2.0:
                        await self.a_agent.send_message("action", "stop")
                        print("Flower collected successfully!")
                        self.state = self.END
                    
                elif self.state == self.END:
                    break
                
                await asyncio.sleep(0.1)
            
            return True
            
        except asyncio.CancelledError:
            print("***** TASK FlowerCollector CANCELLED")
            await self.a_agent.send_message("action", "stop")
            await self.a_agent.send_message("action", "nt")  # Para de girar
            return False


class ReturnToBase: # RIGHT NOW IT USES TELEPORT
    """
    Returns to the base outpost using NavMesh or teleport
    """
    MOVING = 0
    ARRIVED = 1
    
    def __init__(self, a_agent, use_teleport=True):
        self.a_agent = a_agent
        self.rc_sensor = a_agent.rc_sensor
        self.i_state = a_agent.i_state
        self.state = self.MOVING
        self.use_teleport = use_teleport
        
    async def run(self):
        try:
            if self.state == self.MOVING:
                # Choose navigation method
                if self.use_teleport:
                    print("Teleporting to base")
                    await self.a_agent.send_message("action", "teleport_to,Base")
                else:
                    print("Walking to base using NavMesh")
                    await self.a_agent.send_message("action", "walk_to,Base")
                
                self.state = self.ARRIVED
            
            # Wait until we've arrived at the base
            while self.state == self.ARRIVED:
                if self.i_state.currentNamedLoc == "Base" and not self.i_state.onRoute:
                    print("Arrived at base")
                    break
                await asyncio.sleep(0.5) # Can be reduced for more fluency
            
            return True
            
        except asyncio.CancelledError:
            print("***** TASK ReturnToBase CANCELLED")
            return False


class UnloadFlowers:
    """
    Unloads flowers at the base
    """
    def __init__(self, a_agent):
        self.a_agent = a_agent
        self.i_state = a_agent.i_state
        
    async def run(self):
        try:
            # Check if we're at base
            if self.i_state.currentNamedLoc != "Base":
                print("Not at base, can't unload")
                return False
                
            # Check if we have flowers to unload
            flower_count = 0
            for item in self.i_state.myInventoryList:
                if item["name"] == "AlienFlower":
                    flower_count = item["amount"]
                    break
                    
            if flower_count == 0:
                print("No flowers to unload")
                return False
                
            # Unload flowers
            print(f"Unloading {flower_count} flowers")
            await self.a_agent.send_message("action", f"leave,AlienFlower,{flower_count}")
            
            # Wait for unload to complete
            await asyncio.sleep(2)
            
            return True
            
        except asyncio.CancelledError:
            print("***** TASK UnloadFlowers CANCELLED")
            return False


class RandomWander:
    """
    Wanders randomly to search for flowers with improved continuous movement
    """
    DECIDING = 0
    MOVING = 1
    TURNING = 2
    
    def __init__(self, a_agent):
        self.a_agent = a_agent
        self.rc_sensor = a_agent.rc_sensor
        self.i_state = a_agent.i_state
        self.state = self.DECIDING
        self.movement_time = 0
        self.movement_start = 0
        self.turn_target = 0
        self.direction = ""
        self.last_state_change = 0
        self.min_state_duration = 2.0  # Minimum time to stay in a state (seconds)
        self.last_flower_check = 0
        self.flower_check_interval = 0.2  # Check for flowers every 0.2 seconds
        
    async def run(self):
        try:
            print("Starting random wander behavior")
            # Start by moving forward
            await self.a_agent.send_message("action", "mf")
            self.state = self.MOVING
            self.movement_start = asyncio.get_event_loop().time()
            self.movement_time = random.uniform(5.0, 8.0)  # Longer movement time
            self.last_state_change = self.movement_start
            
            while True:
                current_time = asyncio.get_event_loop().time()
                time_in_state = current_time - self.last_state_change
                
                # Frequently check for flowers (highest priority)
                if current_time - self.last_flower_check >= self.flower_check_interval:
                    self.last_flower_check = current_time
                    flower_detected = self.check_for_flowers()
                    if flower_detected:
                        # Stop current movement when flower detected
                        await self.a_agent.send_message("action", "stop")
                        await self.a_agent.send_message("action", "nt")
                        print("Flower detected during wandering, exiting wander behavior")
                        return True
                
                # Check for obstacle detection
                obstacle_detected = self.check_for_obstacles()
                
                if self.state == self.MOVING:
                    # Continue moving forward until obstacle or time elapsed
                    if obstacle_detected:
                        print("Obstacle detected, changing direction")
                        await self.a_agent.send_message("action", "stop")
                        await asyncio.sleep(0.1)
                        self.state = self.TURNING
                        self.last_state_change = current_time
                        
                        # Choose turn direction based on sensor data
                        turn_direction = self.choose_turn_direction()
                        turn_degrees = random.randint(45, 120)
                        
                        if turn_direction == "left":
                            self.direction = "left"
                            self.turn_target = (self.i_state.rotation["y"] - turn_degrees) % 360
                            await self.a_agent.send_message("action", "tl")
                        else:
                            self.direction = "right"
                            self.turn_target = (self.i_state.rotation["y"] + turn_degrees) % 360
                            await self.a_agent.send_message("action", "tr")
                    
                    elif time_in_state >= self.movement_time:
                        # Time to change things up a bit
                        if random.random() < 0.3:  # 30% chance to turn
                            print("Changing direction randomly during wander")
                            await self.a_agent.send_message("action", "stop")
                            await asyncio.sleep(0.1)
                            self.state = self.TURNING
                            self.last_state_change = current_time
                            
                            # Random turn
                            turn_degrees = random.randint(20, 90)
                            if random.random() < 0.5:
                                self.direction = "left"
                                self.turn_target = (self.i_state.rotation["y"] - turn_degrees) % 360
                                await self.a_agent.send_message("action", "tl")
                            else:
                                self.direction = "right"
                                self.turn_target = (self.i_state.rotation["y"] + turn_degrees) % 360
                                await self.a_agent.send_message("action", "tr")
                        else:
                            # Continue moving but update timer
                            print("Continuing forward movement")
                            self.movement_time = random.uniform(5.0, 8.0)
                            self.movement_start = current_time
                            self.last_state_change = current_time
                            # Ensure we're still moving
                            await self.a_agent.send_message("action", "mf")
                
                elif self.state == self.TURNING:
                    current_rotation = self.i_state.rotation["y"]
                    
                    # Check if rotation complete (with 10 degree tolerance)
                    angle_diff = min(abs(current_rotation - self.turn_target), 
                                    360 - abs(current_rotation - self.turn_target))
                    
                    if angle_diff < 10 or time_in_state > 3.0:  # Timeout after 3 seconds
                        print("Turn complete or timeout, resuming forward movement")
                        await self.a_agent.send_message("action", "nt")  # Stop turning
                        await asyncio.sleep(0.1)
                        
                        # Resume forward movement
                        self.state = self.MOVING
                        self.movement_time = random.uniform(5.0, 8.0)
                        self.movement_start = current_time
                        self.last_state_change = current_time
                        await self.a_agent.send_message("action", "mf")
                
                await asyncio.sleep(0.1)
                
        except asyncio.CancelledError:
            print("***** TASK RandomWander CANCELLED")
            await self.a_agent.send_message("action", "stop")
            await self.a_agent.send_message("action", "nt")
            return False
    
    def check_for_obstacles(self):
        """Check if there's an obstacle in front of the agent"""
        sensor_hits = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.HIT]
        sensor_distances = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.DISTANCE]
        sensor_obj_info = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.OBJECT_INFO]
        
        # Only consider obstacles in front of the agent (central sensors)
        center_indices = [len(sensor_hits) // 2 - 1, len(sensor_hits) // 2, len(sensor_hits) // 2 + 1]
        
        for i in center_indices:
            if i >= 0 and i < len(sensor_hits):
                if sensor_hits[i] and sensor_distances[i] < 1.2:
                    # Don't consider flowers as obstacles
                    if sensor_obj_info[i] and sensor_obj_info[i].get("tag") != "AlienFlower":
                        return True
        
        return False
    
    def choose_turn_direction(self):
        """Choose turn direction based on sensor data"""
        sensor_hits = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.HIT]
        sensor_distances = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.DISTANCE]
        
        # Calculate average distance on left and right sides
        left_count = 0
        left_total_dist = 0
        right_count = 0
        right_total_dist = 0
        
        for i in range(len(sensor_hits)):
            if sensor_hits[i]:
                if i < len(sensor_hits) // 2:  # Left side
                    left_count += 1
                    left_total_dist += sensor_distances[i]
                else:  # Right side
                    right_count += 1
                    right_total_dist += sensor_distances[i]
        
        # Calculate average distances
        left_avg = left_total_dist / left_count if left_count > 0 else 999
        right_avg = right_total_dist / right_count if right_count > 0 else 999
        
        # Turn toward the direction with more space
        return "right" if left_avg < right_avg else "left"
    
    def check_for_flowers(self):
        """Check if there's a flower in the sensor data"""
        sensor_obj_info = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.OBJECT_INFO]
        
        for value in sensor_obj_info:
            if value and value["tag"] == "AlienFlower":
                return True
        
        return False










'''
CRITTERS SCENARIO (below)
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
        self.safe_distance = 3.0
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
                    
                    if rot_diff >= 10:
                        await self.a_agent.send_message("action", "nt")
                        
                        # clearance check
                        front_clear = not any(
                            hit and dist < self.safe_distance
                            for i, (hit, dist) in enumerate(zip(sensor_hits, sensor_distances))
                            if abs(sensor_angles[i]) < 15
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
                            turning_angle = -90 + self.a_agent.det_sensor * (90 / 5)
                            await self.a_agent.send_message("action", "tl") #Turn left

                        elif(self.a_agent.det_sensor>5):    
                            turning_angle = self.a_agent.det_sensor * (90 / 5)
                            await self.a_agent.send_message("action", "tr") #Turn right
                        
                        else:
                            turning_angle = 0 #Used to avoid errors
                            await self.a_agent.send_message("action", "mf") # Move forward

                        await asyncio.sleep(0.02) # Small delay to prevent busy waiting
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
            #print("***** TASK Follow CANCELLED")
            await self.a_agent.send_message("action", "nt")