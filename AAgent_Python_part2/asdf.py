class Avoid:
    """
    Improved avoidance while keeping original structure
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
        self.last_command_time = 0  # NEW: Track command timing

    async def run(self):
        try:
            while True:
                current_time = asyncio.get_event_loop().time()
                
                # Only proceed if enough time has passed since last command (NEW)
                if current_time - self.last_command_time < 0.1:  # Send commands every 0.1s
                    await asyncio.sleep(0.01)
                    continue
                    
                self.last_command_time = current_time
                
                # Get sensor data (unchanged from original)
                sensor_hits = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.HIT]
                sensor_distances = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.DISTANCE]
                sensor_angles = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.ANGLE]
                
                if self.state == self.MOVING:
                    await self.a_agent.send_message("action", "mf")  # Continuous movement
                    
                    # Improved obstacle detection (similar structure but more reliable)
                    obstacle_info = []
                    for i, (hit, dist) in enumerate(zip(sensor_hits, sensor_distances)):
                        if hit and 0.1 < dist < self.safe_distance:  # Added minimum distance
                            obstacle_info.append((sensor_angles[i], dist))
                    
                    if obstacle_info:  # If any obstacles found
                        await self.a_agent.send_message("action", "stop")
                        
                        # Original turn decision logic (preserved)
                        left_obstacles = [info for info in obstacle_info if info[0] < 0]
                        right_obstacles = [info for info in obstacle_info if info[0] > 0]
                        
                        left_avg_dist = sum(d for _,d in left_obstacles)/len(left_obstacles) if left_obstacles else float('inf')
                        right_avg_dist = sum(d for _,d in right_obstacles)/len(right_obstacles) if right_obstacles else float('inf')
                        
                        if left_avg_dist > right_avg_dist:
                            self.avoid_direction = "left"
                            print("Obstacle detected! Turning left")
                        else:
                            self.avoid_direction = "right"
                            print("Obstacle detected! Turning right")
                            
                        print(f"Turning {self.avoid_direction} (left avg: {left_avg_dist:.2f}, right avg: {right_avg_dist:.2f})")
                        self.avoid_start_rotation = self.i_state.rotation["y"]
                        self.state = self.AVOIDING
                
                elif self.state == self.AVOIDING:
                    # Continuous turning command (NEW)
                    await self.a_agent.send_message("action", "tl" if self.avoid_direction == "left" else "tr")
                    
                    # Original rotation check (unchanged)
                    current_rot = self.i_state.rotation["y"]
                    rot_diff = abs(current_rot - self.avoid_start_rotation)
                    rot_diff = min(rot_diff, 360 - rot_diff)
                    
                    if rot_diff >= 45:  # Minimum turn achieved
                        await self.a_agent.send_message("action", "nt")
                        
                        # Original clearance check (unchanged)
                        front_clear = not any(
                            hit and dist < self.safe_distance
                            for i, (hit, dist) in enumerate(zip(sensor_hits, sensor_distances))
                            if abs(sensor_angles[i]) < 45  # Front cone only
                        )
                        
                        if front_clear:
                            print("Avoidance complete")
                            return True  # Success!
                        else:
                            # Continue avoiding from new angle
                            self.avoid_start_rotation = current_rot
                
                await asyncio.sleep(0.01)  # Small delay
                
        except asyncio.CancelledError:
            await self.a_agent.send_message("action", "nt")
            await self.a_agent.send_message("action", "stop")
            raise