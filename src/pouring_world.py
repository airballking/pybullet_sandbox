import pybullet as p
import pybullet_data
import numpy as np
import rospy

class PouringWorld(object):
    def __init__(self):
        self.physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
        self.period = 0.01
        self.gravity_vector = np.array([0,0,-9.81])
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
        p.setGravity(self.gravity_vector[0], self.gravity_vector[1], self.gravity_vector[2])
        p.setRealTimeSimulation(0)
        planeId = p.loadURDF("plane.urdf")
        self.mug_id = p.loadURDF("/home/gbartels/ros/giskard/src/pybullet_sandbox/data/mug/mug.urdf", [0,0,1])

    def __del__(self):
        print('Cleaning up')
        p.disconnect()

    def run(self):
        p.setTimeStep(self.period, self.physicsClient)
        rate = rospy.Rate(1.0/self.period)
        while not rospy.is_shutdown():
            self.compensate_gravity(self.mug_id)
            p.stepSimulation(self.physicsClient)
            rate.sleep()

    def compensate_gravity(self, object_id, link_id = -1):
        mass = p.getDynamicsInfo(object_id, link_id)[0]
        p.applyExternalForce(object_id, link_id, (-mass * self.gravity_vector).tolist(), [0, 0, 0], p.LINK_FRAME)

if __name__ == '__main__':
    rospy.init_node("joint_state_bag_comdr")
    my_world = PouringWorld()
    my_world.run()