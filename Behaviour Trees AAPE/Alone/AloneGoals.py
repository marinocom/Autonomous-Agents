import math
import random
import asyncio
import Sensors
from collections import Counter

"""
Alejandra Reinares Guerreros (1665499)
Marino Oliveros Blanco (1668563)
Andreu Gascón Marzo (1670919)
Pere Mayol Carbonell (1669503)
"""

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