class Avoid:
    MOVING = 0
    AVOIDING = 1
    STOPPING = 2  # New state for controlled stopping
    
    def __init__(self, a_agent):
        self.a_agent = a_agent
        self.rc_sensor = a_agent.rc_sensor
        self.i_state = a_agent.i_state
        self.state = self.MOVING
        self.avoid_direction = ""
        self.avoid_start_rotation = 0
        self.safe_distance = 3.0  # Start with 3.0, adjust as needed
        self.emergency_distance = 1.5  # Immediate stop distance
        self.command_interval = 0.05  # Faster reaction time
        self.last_command_time = 0
        self.speed_reduction = 0.7  # Reduce movement speed
        
    async def run(self):
        try:
            while True:
                current_time = asyncio.get_event_loop().time()
                
                if current_time - self.last_command_time < self.command_interval:
                    await asyncio.sleep(0.01)
                    continue
                    
                self.last_command_time = current_time
                
                sensor_hits = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.HIT]
                sensor_distances = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.DISTANCE]
                sensor_angles = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.ANGLE]
                
                if self.state == self.MOVING:
                    # Slower forward movement
                    await self.a_agent.send_message("action", "mf")
                    await asyncio.sleep(0.05 * self.speed_reduction)
                    
                    # Check for obstacles with different severity levels
                    critical_hits = []
                    warning_hits = []
                    
                    for i, (hit, dist) in enumerate(zip(sensor_hits, sensor_distances)):
                        if hit and dist > 0:  # Valid hit
                            if dist < self.emergency_distance:
                                critical_hits.append((sensor_angles[i], dist))
                            elif dist < self.safe_distance:
                                warning_hits.append((sensor_angles[i], dist))
                    
                    if critical_hits:
                        # Immediate stop for critical distances
                        await self.a_agent.send_message("action", "stop")
                        self.state = self.STOPPING
                        self.stop_start_time = current_time
                    elif warning_hits:
                        # Prepare avoidance for warning distances
                        await self.a_agent.send_message("action", "stop")
                        left_obstacles = [info for info in warning_hits if info[0] < 0]
                        right_obstacles = [info for info in warning_hits if info[0] > 0]
                        
                        left_avg_dist = sum(d for _,d in left_obstacles)/len(left_obstacles) if left_obstacles else float('inf')
                        right_avg_dist = sum(d for _,d in right_obstacles)/len(right_obstacles) if right_obstacles else float('inf')
                        
                        self.avoid_direction = "left" if left_avg_dist > right_avg_dist else "right"
                        self.avoid_start_rotation = self.i_state.rotation["y"]
                        self.state = self.AVOIDING
                
                elif self.state == self.STOPPING:
                    # Ensure complete stop before turning
                    if current_time - self.stop_start_time > 0.2:
                        # After stopping, switch to avoiding
                        left_hits = [i for i, (hit, dist) in enumerate(zip(sensor_hits, sensor_distances)) 
                                   if hit and dist < self.safe_distance and sensor_angles[i] < 0]
                        right_hits = [i for i, (hit, dist) in enumerate(zip(sensor_hits, sensor_distances)) 
                                    if hit and dist < self.safe_distance and sensor_angles[i] > 0]
                        
                        self.avoid_direction = "left" if len(left_hits) < len(right_hits) else "right"
                        self.avoid_start_rotation = self.i_state.rotation["y"]
                        self.state = self.AVOIDING
                
                elif self.state == self.AVOIDING:
                    # Turn more aggressively
                    turn_amount = 60  # degrees
                    await self.a_agent.send_message("action", "tl" if self.avoid_direction == "left" else "tr")
                    
                    current_rot = self.i_state.rotation["y"]
                    rot_diff = abs(current_rot - self.avoid_start_rotation)
                    rot_diff = min(rot_diff, 360 - rot_diff)
                    
                    if rot_diff >= turn_amount:
                        await self.a_agent.send_message("action", "nt")
                        
                        # Verify clearance before resuming
                        front_clear = True
                        for i, (hit, dist) in enumerate(zip(sensor_hits, sensor_distances)):
                            if hit and abs(sensor_angles[i]) < 45 and dist < self.safe_distance * 1.2:
                                front_clear = False
                                break
                        
                        if front_clear:
                            self.state = self.MOVING
                        else:
                            # Need to avoid again from new angle
                            self.avoid_start_rotation = current_rot
                            turn_amount += 15  # Increase turn angle each time
                
                await asyncio.sleep(0.01)
                
        except asyncio.CancelledError:
            await self.a_agent.send_message("action", "nt")
            await self.a_agent.send_message("action", "stop")
            raise