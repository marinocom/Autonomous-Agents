import asyncio
import random
import py_trees
import py_trees as pt
from py_trees import common
import Goals_BT
import Sensors


class BN_DoNothing(pt.behaviour.Behaviour):
    def __init__(self, aagent):
        self.my_agent = aagent
        self.my_goal = None
        # print("Initializing BN_DoNothing")
        super(BN_DoNothing, self).__init__("BN_DoNothing")

    def initialise(self):
        self.my_goal = asyncio.create_task(Goals_BT.DoNothing(self.my_agent).run())

    def update(self):
        if not self.my_goal.done():
            return pt.common.Status.RUNNING
        else:
            if self.my_goal.result():
                # print("BN_DoNothing completed with SUCCESS")
                return pt.common.Status.SUCCESS
            else:
                # print("BN_DoNothing completed with FAILURE")
                return pt.common.Status.FAILURE

    def terminate(self, new_status: common.Status):
        # Finishing the behaviour, therefore we have to stop the associated task
        self.my_goal.cancel()

class BN_DetectObstacle(pt.behaviour.Behaviour):
    '''
    Description: Behaviour that detects an obstacle in the environment
                 using the raycast sensor of the critter
    '''
    def __init__(self, aagent):
        '''
        init method for BN_DetectObstacle
        '''
        #Set the goal to None
        self.my_goal = None
        #Print a message to the terminal
        print("Initializing BN_DetectObstacle")
        #Call the parent constructor
        super(BN_DetectObstacle, self).__init__("BN_DetectObstacle")
        #get the agent
        self.my_agent = aagent

    def initialise(self):
        '''
        initialise method for BN_DetectObstacle, does nothing, pass
        '''
        pass

    def update(self):
        '''
        update method for BN_DetectObstacle:
        checks if the raycast sensor detects an obstacle in the environment or not
        '''
        #Get the object information from the raycast sensor
        sensor_obj_info = self.my_agent.rc_sensor.sensor_rays[Sensors.RayCastSensor.OBJECT_INFO]
        sensor_distances = self.my_agent.rc_sensor.sensor_rays[Sensors.RayCastSensor.DISTANCE]
        sensor_angles = self.my_agent.rc_sensor.sensor_rays[Sensors.RayCastSensor.ANGLE]
        
        #Iterate through the object information
        for index, value in enumerate(sensor_obj_info):
            #If there is a hit with an object
            if value: 
                #If the object it hits is not an astronaut
                if value["tag"] != "Astronaut" and sensor_distances[index] <    1.0:
                    # an obstacle is detected, print a message to the terminal
                    print("BN_DetectObstacle completed with SUCCESS")
                    #Return success
                    return pt.common.Status.SUCCESS
        #Return failure if no obstacle is detected
        return pt.common.Status.FAILURE

    def terminate(self, new_status: common.Status):
        '''
        terminate method for BN_DetectObstacle, does nothing, pass
        '''
        pass

class BN_Avoid(pt.behaviour.Behaviour):
    '''
    Description: Behaviour that makes the agent avoid an obstacle in the environment
                 using the raycast sensor of the critter
    '''
    def __init__(self, aagent):
        '''
        init method for BN_Avoid
        '''
        #Set the goal to None
        self.my_goal = None
        #Print a message to the terminal
        #print("Initializing BN_Avoid")
        #Call the parent constructor
        super(BN_Avoid, self).__init__("BN_Avoid")
        #get the agent
        self.my_agent = aagent

    def initialise(self):
        '''
        initialise method for BN_Avoid, creates a task to avoid the obstacle
        '''
        self.my_goal = asyncio.create_task(Goals_BT.Avoid(self.my_agent).run())

    def update(self):
        '''
        update method for BN_Avoid:
        checks if the goal is done, if it is, checks if the goal was successful or not
        '''
        #Check if the goal is not done
        if not self.my_goal.done():
            #Return running
            return pt.common.Status.RUNNING
        #If the goal is done
        else:
            #Check if the goal was successful
            if self.my_goal.result():
                #print("BN_Avoid completed with SUCCESS")
                #Return success
                return pt.common.Status.SUCCESS
            #If the goal was not successful
            else:
                #print("BN_Avoid completed with FAILURE")
                #Return failure
                return pt.common.Status.FAILURE

    def terminate(self, new_status: common.Status):
        '''
        terminate method for BN_Avoid by cancelling the goal
        '''
        # we have to stop the associated task
        self.logger.debug("Terminate BN_Avoid")
        self.my_goal.cancel()


class BN_DetectAstro(pt.behaviour.Behaviour):
    '''
    Description: Behaviour that detects an astronaut in the environment
                 using the raycast sensor of the critter
    '''    
    def __init__(self, aagent):
        '''
        init method for BN_DetectAstro
        '''
        #Set the goal to None
        self.my_goal = None
        #Print a message to the terminal
        print("Initializing BN_DetectAstro")
        #Call the parent constructor
        super(BN_DetectAstro, self).__init__("BN_DetectAstro")
        #get the agent
        self.my_agent = aagent
        #Create a variable to store the sensor index, initialized to None
        self.my_agent.det_sensor = None

    def initialise(self):
        '''
        initialise method for BN_DetectAstro, does nothing, pass
        '''
        pass

    def update(self):
        '''
        update method for BN_DetectAstro:
        checks if the raycast sensor detects an astronaut in the environment or not
        '''
        #Get the object information from the raycast sensor
        sensor_obj_info = self.my_agent.rc_sensor.sensor_rays[Sensors.RayCastSensor.OBJECT_INFO]
        #Iterate through the object information
        for index, value in enumerate(sensor_obj_info):
            #If there is a hit with an object
            if value:  
                #If the object it hits is an astronaut
                if value["tag"] == "Astronaut": 
                    # an astronaut is detected, print a message to the terminal
                    print("BN_DetectAstro completed with SUCCESS")
                    #Set the sensor index to the index of the astronaut
                    self.my_agent.det_sensor = index
                    #Return success
                    return pt.common.Status.SUCCESS
        #Return failure if no astronaut is detected
        return pt.common.Status.FAILURE

    def terminate(self, new_status: common.Status):
        '''
        terminate method for BN_DetectAstro, does nothing, pass
        '''
        pass


class BN_FollowAstro(pt.behaviour.Behaviour):
    '''
    Description: Behaviour that makes the agent follow an astronaut in the environment
    '''
    def __init__(self, aagent):
        '''
        init method for BN_FollowAstro
        '''
        #Set the goal to None
        self.my_goal = None
        #Print a message to the terminal
        print("Initializing BN_FollowAstro")
        #Call the parent constructor
        super(BN_FollowAstro, self).__init__("BN_FollowAstro")
        #get the agent
        self.my_agent = aagent

    def initialise(self):
        '''
        initialise method for BN_FollowAstro, creates a task to follow the astronaut
        '''
        self.my_goal = asyncio.create_task(Goals_BT.FollowAstronaut(self.my_agent).run())

    def update(self):
        '''
        update method for BN_FollowAstro:
        checks if the goal is done, if it is, checks if the goal was successful or not
        '''
        #Check if the goal is not done
        if not self.my_goal.done():
            #Return running
            return pt.common.Status.RUNNING
        #If the goal is done
        else:
            #Check if the goal was successful
            if self.my_goal.result():
                print("BN_FollowAstro completed with SUCCESS")
                #Return success
                return pt.common.Status.SUCCESS
            #If the goal was not successful
            else:
                print("BN_FollowAstro completed with FAILURE")
                #Return failure
                return pt.common.Status.FAILURE

    def terminate(self, new_status: common.Status):
        '''
        terminate method for BN_FollowAstro by cancelling the goal
        '''
        # we have to stop the associated task
        self.logger.debug("Terminate BN_FollowAstro")
        self.my_goal.cancel()


class BN_RandomRoam(pt.behaviour.Behaviour):
    def __init__(self, aagent):
        self.my_goal = None
        # print("Initializing BN_ForwardRandom")
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
                # print("BN_RandomRoam completed with SUCCESS")
                return pt.common.Status.SUCCESS
            else:
                self.logger.debug("BN_RandomRoam completed with FAILURE")
                # print("BN_RandomRoam completed with FAILURE")
                return pt.common.Status.FAILURE

    def terminate(self, new_status: common.Status):
        # Finishing the behaviour, therefore we have to stop the associated task
        self.logger.debug("Terminate BN_RandomRoam")
        self.my_goal.cancel()




class BTCritter:
    def __init__(self, aagent):

        #Create the detect avoid sequence with the detect obstacle and avoid behaviours
        det_avoid = pt.composites.Sequence(name="Detect_Avoid", memory=True)
        #Add the detect obstacle and avoid behaviours to the sequence as children
        det_avoid.add_children([BN_DetectObstacle(aagent), BN_Avoid(aagent)])

        #Create the detect follow sequence with the detect astronaut and follow astronaut behaviours
        det_astro = pt.composites.Sequence(name="Detect_Follow", memory=True)
        #Add the detect astronaut and follow astronaut behaviours to the sequence as children
        det_astro.add_children([BN_DetectAstro(aagent), BN_FollowAstro(aagent)])

        #Create the root selector with the detect flower, detect astronaut, detect avoid, and roaming behaviours
        self.root = pt.composites.Selector(name="Selector", memory=False)
        #Add the detect flower, detect astronaut, detect avoid, and roaming behaviours to the selector as children
        self.root.add_children([det_astro, det_avoid, BN_RandomRoam(aagent)])

        self.behaviour_tree = pt.trees.BehaviourTree(self.root)

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
        await asyncio.sleep(0)
