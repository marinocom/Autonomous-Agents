import asyncio
import py_trees as pt
from py_trees import common
import Sensors

"""
Alejandra Reinares Guerreros (1665499)
Marino Oliveros Blanco (1668563)
Andreu Gascón Marzo (1670919)
Pere Mayol Carbonell (1669503)
"""

class BN_DetectFrozen(pt.behaviour.Behaviour):
    def __init__(self, aagent):
        self.my_goal = None
        super(BN_DetectFrozen, self).__init__("BN_DetectFrozen")
        self.my_agent = aagent
        self.i_state = aagent.i_state
        self.last_check_time = 0
        self.check_interval = 2.0  # requency of checks

    def initialise(self):
        pass

    def update(self):
        # Limit check frequency
        current_time = asyncio.get_event_loop().time()
        if current_time - self.last_check_time < self.check_interval:
            return self.status
        
        self.last_check_time = current_time
        
        if self.i_state.isFrozen:
            return pt.common.Status.SUCCESS
        return pt.common.Status.FAILURE

    def terminate(self, new_status: common.Status):
        pass


class BN_DoNothing(pt.behaviour.Behaviour):
    def __init__(self, aagent):
        self.my_agent = aagent
        self.my_goal = None
        super(BN_DoNothing, self).__init__("BN_DoNothing")

    def initialise(self):
        from Goals_BT import DoNothing
        self.my_goal = asyncio.create_task(DoNothing(self.my_agent).run())

    def update(self):
        if not self.my_goal.done():
            return pt.common.Status.RUNNING
        else:
            if self.my_goal.result():
                return pt.common.Status.SUCCESS
            else:
                return pt.common.Status.FAILURE

    def terminate(self, new_status: common.Status):
        # Finishing the behaviour, therefore we have to stop the associated task
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
        import AloneGoals  # Replace with your actual module name
        self.my_goal = asyncio.create_task(AloneGoals.FlowerCollector(self.my_agent).run())

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
        import AloneGoals  # Replace with your actual module name
        self.my_goal = asyncio.create_task(AloneGoals.RandomWander(self.my_agent).run())
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
        import AloneGoals  # Replace with your actual module name
        self.my_goal = asyncio.create_task(AloneGoals.ReturnToBase(self.my_agent, self.use_teleport).run())

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
        import AloneGoals  # Replace with your actual module name
        self.my_goal = asyncio.create_task(AloneGoals.UnloadFlowers(self.my_agent).run())

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

        # Define the behavior tree structure with improved memory settings
        
        # Handle frozen state - memory=True to remember state
        frozen = pt.composites.Sequence(name="Sequence_frozen", memory=True)
        frozen.add_children([BN_DetectFrozen(aagent), BN_DoNothing(aagent)])
        
        # Handle flower collection
        collect_flower = pt.composites.Sequence(name="Sequence_collect_flower", memory=True)
        collect_flower.add_children([BN_DetectAlienFlower(aagent), BN_CollectFlower(aagent)])
        
        # Handle returning to base when inventory is full
        return_to_base = pt.composites.Sequence(name="Sequence_return_to_base", memory=True)
        return_to_base.add_children([
            BN_CheckInventoryFull(aagent), 
            BN_ReturnToBase(aagent), 
            BN_UnloadFlowers(aagent)
        ])
        
        # Main behaviors - using Priority Selector with memory=True
        # This prevents rapid switching between behaviors
        main_behaviors = pt.composites.Selector(name="Selector_main_behaviors", memory=True)
        main_behaviors.add_children([
            return_to_base,  # First priority: return to base if inventory full
            collect_flower,  # Second priority: collect flower if one is detected
            BN_RandomWander(aagent)  # Third priority: wander randomly
        ])
        
        # Root selector with memory=False to allow switching between frozen and normal states
        self.root = pt.composites.Selector(name="Selector_root", memory=False)
        self.root.add_children([frozen, main_behaviors])
        
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