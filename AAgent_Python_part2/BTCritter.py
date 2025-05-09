import asyncio
import random
import py_trees
import py_trees as pt
from py_trees import common
import Goals_BT
import Sensors


class BN_DetectObstacle(pt.behaviour.Behaviour):
    '''
    Behavior node responsible for detecting obstacles in the environment. It uses the critter's raycast sensor to identify nearby objects (excluding astronauts).
    '''
    def __init__(self, aagent):
        self.my_goal = None
        super(BN_DetectObstacle, self).__init__("BN_DetectObstacle")
        self.my_agent = aagent

    def initialise(self):
        pass

    def update(self):
        '''
        Called every tick to evaluate whether an obstacle is detected. It checks the agent's raycast sensors for any nearby objects within a specific distance, ignoring astronauts.
        '''
        #Retrieve sensor information: detected objects, their distances, and angles
        sensor_obj_info = self.my_agent.rc_sensor.sensor_rays[Sensors.RayCastSensor.OBJECT_INFO]
        sensor_distances = self.my_agent.rc_sensor.sensor_rays[Sensors.RayCastSensor.DISTANCE]
        
        #Iterate over detected objects from sensor data
        for index, obj in enumerate(sensor_obj_info):
            if obj:
                #If the object it hits is not an astronaut
                if(obj["tag"]!="Astronaut" and sensor_distances[index]<1.0):
                    return pt.common.Status.SUCCESS
        return pt.common.Status.FAILURE

    def terminate(self, new_status: common.Status):
        pass

class BN_Avoid(pt.behaviour.Behaviour):
    '''
    Behavior node that makes the agent avoid obstacles in its environment. It uses the agent's raycast sensor and executes an avoidance maneuver asynchronously.
    '''
    def __init__(self, aagent):
        self.my_goal = None
        super(BN_Avoid, self).__init__("BN_Avoid")
        self.my_agent = aagent

    def initialise(self):
        self.my_goal = asyncio.create_task(Goals_BT.Avoid(self.my_agent).run())

    def update(self):
        if not self.my_goal.done():
            return pt.common.Status.RUNNING
        else:
            if self.my_goal.result():
                return pt.common.Status.SUCCESS
            else:
                return pt.common.Status.FAILURE

    def terminate(self, new_status: common.Status):
        self.logger.debug("Terminate BN_Avoid")
        self.my_goal.cancel()


class BN_DetectAstronaut(pt.behaviour.Behaviour):
    '''
    Behavior node that detects the presence of an astronaut in the environment. It uses the agent's raycast sensor to scan for astronauts.
    '''    
    def __init__(self, aagent):
        self.my_goal = None
        print("CRITTER: Initializing BN_DetectAstronaut")
        super(BN_DetectAstronaut, self).__init__("BN_DetectAstronaut")
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
                if value["tag"] == "Astronaut": #If the object hit is an astronaut
                    #print("BN_DetectAstronaut completed with SUCCESS")
                    self.my_agent.det_sensor = index #Store the index of the sensor that detected the astronaut
                    return pt.common.Status.SUCCESS
        return pt.common.Status.FAILURE

    def terminate(self, new_status: common.Status):
        pass


class BN_FollowAstronaut(pt.behaviour.Behaviour):
    '''
    Behavior node that makes the agent follow an astronaut detected in the environment. Spawns a FollowAstronaut goal as an asynchronous task.
    '''
    def __init__(self, aagent):
        self.my_goal = None
        #print("Initializing BN_FollowAstronaut")
        super(BN_FollowAstronaut, self).__init__("BN_FollowAstronaut")
        self.my_agent = aagent

    def initialise(self):
        self.my_goal = asyncio.create_task(Goals_BT.FollowAstronaut(self.my_agent).run())

    def update(self):
        if not self.my_goal.done():
            return pt.common.Status.RUNNING
        else:
            if self.my_goal.result():
                #print("BN_FollowAstronaut completed with SUCCESS")
                return pt.common.Status.SUCCESS
            else:
                #print("BN_FollowAstronaut completed with FAILURE")
                return pt.common.Status.FAILURE

    def terminate(self, new_status: common.Status):
        self.logger.debug("Terminate BN_FollowAstronaut")
        self.my_goal.cancel()


class BN_RandomRoam(pt.behaviour.Behaviour):
    def __init__(self, aagent):
        self.my_goal = None
        super(BN_RandomRoam, self).__init__("BN_RandomRoam")
        self.logger.debug("Initializing BN_RandomRoam")
        self.my_agent = aagent

    def initialise(self):
        self.logger.debug("Create Goals_BT.BN_RandomRoam task")
        self.my_goal = asyncio.create_task(Goals_BT.RandomRoam(self.my_agent).run())

    def update(self):
        if not self.my_goal.done():
            return pt.common.Status.RUNNING
        else:
            if self.my_goal.result():
                self.logger.debug("BN_RandomRoam completed with SUCCESS")
                return pt.common.Status.SUCCESS
            else:
                self.logger.debug("BN_RandomRoam completed with FAILURE")
                return pt.common.Status.FAILURE

    def terminate(self, new_status: common.Status):
        self.logger.debug("Terminate BN_RandomRoam")
        self.my_goal.cancel()




class BTCritter:
    def __init__(self, aagent):

        #Create a sequence node for astronaut detection and following behavior
        det_astro = pt.composites.Sequence(name="Detect_Follow", memory=False)
        det_astro.add_children([BN_DetectAstronaut(aagent), BN_FollowAstronaut(aagent)]) #Add astronaut detection and follow behaviors to the sequence

        #Create a sequence node for obstacle detection and avoidance
        det_avoid = pt.composites.Sequence(name="Detect_Avoid", memory=False)
        det_avoid.add_children([BN_DetectObstacle(aagent), BN_Avoid(aagent)]) #Add the detect obstacle and avoid behaviors as children of the sequence

        #Create the root selector node that prioritizes behaviors based on order
        self.root = pt.composites.Selector(name="Selector", memory=False)
        self.root.add_children([det_astro, det_avoid, BN_RandomRoam(aagent)]) #Add sequences and roaming behavior to the selector

        self.behaviour_tree = pt.trees.BehaviourTree(self.root)

    def set_invalid_state(self, node):
        """
        Recursively sets a node and all its children to INVALID state. This forces any associated asyncio tasks to be cancelled.
        """
        node.status = pt.common.Status.INVALID
        for child in node.children:
            self.set_invalid_state(child)

    def stop_behaviour_tree(self):
        """
        Stops the behavior tree execution by invalidating all nodes. Effectively cancels ongoing behaviors and resets tree state.
        """
        self.set_invalid_state(self.root)

    async def tick(self):
        self.behaviour_tree.tick()
        await asyncio.sleep(0)
