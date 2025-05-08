import asyncio
import py_trees as pt
from py_trees import common
import Sensors
import Goals_BT

"""
Alejandra Reinares Guerreros (1665499)
Marino Oliveros Blanco (1668563)
Andreu Gascón Marzo (1670919)
Pere Mayol Carbonell (1669503)
"""


class BN_DetectCritter(pt.behaviour.Behaviour):
    '''
    Behavior node that detects the presence of an astronaut in the environment. It uses the agent's raycast sensor to scan for astronauts.
    '''    
    def __init__(self, aagent):
        self.my_goal = None
        print("Initializing BN_DetectCritter")
        super(BN_DetectCritter, self).__init__("BN_DetectCritter")
        self.my_agent = aagent
        #Variable to store the sensor index
        self.my_agent.det_sensor = None

    def initialise(self):
        pass

    def update(self):
        '''
        This checks if the raycast sensor detects an astronaut or not
        '''
        #Retrieve the object information from the raycast sensor rays
        sensor_obj_info = self.my_agent.rc_sensor.sensor_rays[Sensors.RayCastSensor.OBJECT_INFO]

        for index, value in enumerate(sensor_obj_info):
            if value:
                if value["tag"] == "CritterMantaRay": #If the object hit is an astronaut
                    print("BN_DetectCritter completed with SUCCESS")
                    self.my_agent.det_sensor = index #Store the index of the sensor that detected the astronaut
                    return pt.common.Status.SUCCESS
        return pt.common.Status.FAILURE

    def terminate(self, new_status: common.Status):
        pass


class BN_AvoidCritter(pt.behaviour.Behaviour):
    '''
    Behavior node that makes the agent follow an astronaut detected in the environment. Spawns a FollowAstronaut goal as an asynchronous task.
    '''
    def __init__(self, aagent):
        self.my_goal = None
        print("Initializing BN_AvoidCritter")
        super(BN_AvoidCritter, self).__init__("BN_AvoidCritter")
        self.my_agent = aagent

    def initialise(self):
        self.my_goal = asyncio.create_task(Goals_BT.AvoidCritter(self.my_agent).run())

    def update(self):
        if not self.my_goal.done():
            return pt.common.Status.RUNNING
        else:
            if self.my_goal.result():
                print("BN_AvoidCritter completed with SUCCESS")
                return pt.common.Status.SUCCESS
            else:
                print("BN_AvoidCritter completed with FAILURE")
                return pt.common.Status.FAILURE

    def terminate(self, new_status: common.Status):
        self.logger.debug("Terminate BN_AvoidCritter")
        self.my_goal.cancel()


class BN_DetectAlienFlower(pt.behaviour.Behaviour):
    def __init__(self, aagent):
        super(BN_DetectAlienFlower, self).__init__("BN_DetectAlienFlower")
        self.my_agent = aagent
        self.rc_sensor = aagent.rc_sensor
        self.last_check_time = 0
        self.check_interval = 0.3  # Check every 0.3 seconds
        self.found_flower = False
        self.detection_persistence = 1.0  # Once detected, keep status for this many seconds
        self.last_detection_time = 0

    def initialise(self):
        self.last_check_time = 0  # Reset timer on initialise
        self.found_flower = False
        self.last_detection_time = 0

    def update(self):
        current_time = asyncio.get_event_loop().time()
        
        # If we recently detected a flower, maintain SUCCESS status for a period
        # This prevents rapid switching between behaviors & FLOWERS
        if self.found_flower and (current_time - self.last_detection_time < self.detection_persistence):
            return pt.common.Status.SUCCESS
        
        # Limit check frequency
        if current_time - self.last_check_time < self.check_interval:
            if self.found_flower:
                return pt.common.Status.SUCCESS
            else:
                return pt.common.Status.FAILURE
        
        self.last_check_time = current_time
        
        # Check for alien flowers in sensor data
        flower_detected = False
        sensor_obj_info = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.OBJECT_INFO]
        
        for index, value in enumerate(sensor_obj_info):
            if value and value["tag"] == "AlienFlower":
                flower_detected = True
                self.found_flower = True
                self.last_detection_time = current_time
                print(f"BN_DetectAlienFlower: Flower detected at sensor index {index}")
                return pt.common.Status.SUCCESS
        
        # Only reset found_flower if we've checked and didn't find any
        self.found_flower = False
        return pt.common.Status.FAILURE

    def terminate(self, new_status: common.Status):
        pass


class BN_CollectFlower(pt.behaviour.Behaviour):
    def __init__(self, aagent):
        self.my_goal = None
        super(BN_CollectFlower, self).__init__("BN_CollectFlower")
        self.my_agent = aagent

    def initialise(self):
        # Using the improved FlowerCollector class
        self.my_goal = asyncio.create_task(Goals_BT.FlowerCollector(self.my_agent).run())

    def update(self):
        if not self.my_goal.done():
            return pt.common.Status.RUNNING
        else:
            if self.my_goal.result():
                return pt.common.Status.SUCCESS
            else:
                return pt.common.Status.FAILURE

    def terminate(self, new_status: common.Status):
        self.my_goal.cancel()


class BN_RandomWander(pt.behaviour.Behaviour):
    def __init__(self, aagent):
        self.my_goal = None
        super(BN_RandomWander, self).__init__("BN_RandomWander")
        self.my_agent = aagent
        self.started = False

    def initialise(self):
        # Using the improved RandomWander class
        self.my_goal = asyncio.create_task(Goals_BT.RandomWander(self.my_agent).run())
        self.started = True

    def update(self):
        if not self.started:
            self.initialise()
            
        if not self.my_goal.done():
            return pt.common.Status.RUNNING
        else:
            if self.my_goal.result():
                return pt.common.Status.SUCCESS
            else:
                return pt.common.Status.FAILURE

    def terminate(self, new_status: common.Status):
        if self.my_goal and not self.my_goal.done():
            self.my_goal.cancel()


class BN_CheckInventoryFull(pt.behaviour.Behaviour):
    def __init__(self, aagent, max_flowers=2):
        super(BN_CheckInventoryFull, self).__init__("BN_CheckInventoryFull")
        self.my_agent = aagent
        self.i_state = aagent.i_state
        self.max_flowers = max_flowers
        self.last_check_time = 0
        self.check_interval = 2.0  # Only check every 2 seconds

    def initialise(self):
        pass

    def update(self):
        # Only check inventory periodically to avoid spamming
        current_time = asyncio.get_event_loop().time()
        if current_time - self.last_check_time < self.check_interval:
            return self.status  # Return current status without changing
        
        self.last_check_time = current_time
        
        # Count flowers in inventory
        flower_count = 0
        for item in self.i_state.myInventoryList:
            if item["name"] == "AlienFlower":
                flower_count = item["amount"]
                break
        
        if flower_count >= self.max_flowers:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

    def terminate(self, new_status: common.Status):
        pass


class BN_ReturnToBase(pt.behaviour.Behaviour):
    def __init__(self, aagent, use_teleport=True):
        self.my_goal = None
        super(BN_ReturnToBase, self).__init__("BN_ReturnToBase")
        self.my_agent = aagent
        self.use_teleport = use_teleport

    def initialise(self):
        # Using the improved ReturnToBase class
        self.my_goal = asyncio.create_task(Goals_BT.ReturnToBase(self.my_agent, self.use_teleport).run())

    def update(self):
        if not self.my_goal.done():
            return pt.common.Status.RUNNING
        else:
            if self.my_goal.result():
                return pt.common.Status.SUCCESS
            else:
                return pt.common.Status.FAILURE

    def terminate(self, new_status: common.Status):
        if self.my_goal and not self.my_goal.done():
            self.my_goal.cancel()


class BN_UnloadFlowers(pt.behaviour.Behaviour):
    def __init__(self, aagent):
        self.my_goal = None
        super(BN_UnloadFlowers, self).__init__("BN_UnloadFlowers")
        self.my_agent = aagent

    def initialise(self):
        # Using the improved UnloadFlowers class
        self.my_goal = asyncio.create_task(Goals_BT.UnloadFlowers(self.my_agent).run())

    def update(self):
        if not self.my_goal.done():
            return pt.common.Status.RUNNING
        else:
            if self.my_goal.result():
                return pt.common.Status.SUCCESS
            else:
                return pt.common.Status.FAILURE

    def terminate(self, new_status: common.Status):
        if self.my_goal and not self.my_goal.done():
            self.my_goal.cancel()


class BTAlone:
    def __init__(self, aagent):
        self.aagent = aagent
        print("Initializing BTAlone behavior tree")

        # Define the behavior tree structure

        #Avoid critters
        avoid_critter = pt.composites.Sequence(name="Sequence_avoid_critters", memory=False)
        avoid_critter.add_children([BN_DetectCritter(aagent), BN_AvoidCritter(aagent)])
        
        # Handle flower collection
        collect_flower = pt.composites.Sequence(name="Sequence_collect_flower", memory=False)
        collect_flower.add_children([BN_DetectAlienFlower(aagent), BN_CollectFlower(aagent)])
        
        # Handle returning to base when inventory is full
        return_to_base = pt.composites.Sequence(name="Sequence_return_to_base", memory=False)
        return_to_base.add_children([
            BN_CheckInventoryFull(aagent), 
            BN_ReturnToBase(aagent), 
            BN_UnloadFlowers(aagent)
        ])
        
        # Main behaviors - using Priority Selector with memory=True
        # This prevents rapid switching between behaviors
        self.root = pt.composites.Selector(name="Selector_main_behaviors", memory=False)
        self.root.add_children([
            avoid_critter,
            return_to_base,  # First priority: return to base if inventory full
            collect_flower,  # Second priority: collect flower if one is detected
            BN_RandomWander(aagent)  # Third priority: wander randomly
        ])

        # Create the behavior tree
        self.behaviour_tree = pt.trees.BehaviourTree(self.root)
        
        # Print the tree structure
        print(pt.display.ascii_tree(self.root))

    # Function to set invalid state for a node and its children recursively
    def set_invalid_state(self, node):
        node.status = pt.common.Status.INVALID
        for child in node.children:
            self.set_invalid_state(child)

    def stop_behaviour_tree(self):
        # Setting all the nodes to invalid, we force the associated asyncio tasks to be cancelled
        self.set_invalid_state(self.root)

    async def tick(self):
        self.behaviour_tree.tick()
        # Use faster tick rate (0.2 seconds) for more responsive behavior/changed to 0.5 because of clunkiness
        # This is especially important for flower detection and pursuit
        await asyncio.sleep(0.5)