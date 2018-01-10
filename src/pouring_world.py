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
        p.setTimeStep(self.period, self.physicsClient)
        planeId = p.loadURDF("plane.urdf")
        self.mug_id = p.loadURDF("/home/gbartels/ros/giskard/src/pybullet_sandbox/data/mug/mug.urdf", [0,0,1])

    def __del__(self):
        print('Cleaning up')
        p.disconnect()

    def place_object(self, object_id, goal_pos, thresh):
        my_world.loop(lambda object_id: self.move_object(object_id, goal_pos),
                      lambda: not self.goal_reached(object_id, goal_pos, thresh))

    def loop(self, loop_fn, loop_cnd):
        rate = rospy.Rate(1.0 / self.period)
        while not rospy.is_shutdown() and loop_cnd():
            loop_fn(self.mug_id)

            p.stepSimulation(self.physicsClient)
            rate.sleep()

    def move_object(self, object_id, goal_pos):
        f_control = self.goal_attraction_force(object_id, goal_pos)
        self.apply_force(f_control, object_id)

    def goal_attraction_force(self, object_id, goal_pos):
        cur_pos = np.array(p.getBasePositionAndOrientation(object_id)[0])
        cur_vel = np.array(p.getBaseVelocity(object_id)[0])
        return (goal_pos - cur_pos) + self.gravity_compensation_force(object_id) - cur_vel

    def goal_reached(self, object_id, goal_pos, thresh):
        cur_pos = np.array(p.getBasePositionAndOrientation(object_id)[0])
        return np.linalg.norm(cur_pos - goal_pos) < abs(thresh)

    def compensate_gravity(self, object_id, link_id = -1):
        """ Compensates gravity for a particular object.

        :param object_id: ID number of the particular object.
        :param link_id: ID number of the link to use for calculation.
        :return: nothing.
        """

        f_gravity_compensation = self.gravity_compensation_force(object_id, link_id)
        self.apply_force(f_gravity_compensation, object_id, link_id)

    def gravity_compensation_force(self, object_id, link_id = -1):
        """ Calculates the forces required to cancel out gravity for a particular object.

        :param object_id: ID number of the particular object.
        :param link_id: ID number of the link to use for calculation
        :return: 3D force to cancel gravity as numpy array.
        """
        mass = p.getDynamicsInfo(object_id, link_id)[0]
        return - mass * self.gravity_vector

    def apply_force(self, force, object_id, link_id = -1):
        """ Applies a 3D force on a particular object.

        :param force: 3D force to apply as a numpy array.
        :param object_id: ID number of the particular object.
        :param link_id: ID number of the link at which to apply.
        :return: nothing
        """
        p.applyExternalForce(object_id, link_id, force.tolist(), [0, 0, 0], p.LINK_FRAME)

if __name__ == '__main__':
    rospy.init_node("joint_state_bag_comdr")
    my_world = PouringWorld()
    conv_thresh = 0.01
    while not rospy.is_shutdown():
        my_world.place_object(my_world.mug_id, np.array([1,0,1]), conv_thresh)
        my_world.place_object(my_world.mug_id, np.array([1,1,1]), conv_thresh)
        my_world.place_object(my_world.mug_id, np.array([0,1,1]), conv_thresh)
        my_world.place_object(my_world.mug_id, np.array([0,0,1]), conv_thresh)