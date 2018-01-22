import interfaces as iface
import PyKDL as kdl

class FlyingObjectController(iface.CallbackInterface):
    def __init__(self, object_name, goal, goal_twist=kdl.Twist(), p_gain=1.0, d_gain=1.0):
        self._object_name = object_name
        self._goal_pose = goal
        self._goal_twist = goal_twist
        self._p_gain = p_gain
        self._d_gain = d_gain

    def call(self, world):
        control_law = self.gravity_compensation(world) + self.pi_controller(world)
        world.apply_wrench(self._object_name, control_law)

    def gravity_compensation(self, world):
        m = world.get_object_mass(self._object_name)
        g = world.get_gravity()
        return kdl.Wrench(-m * g, kdl.Vector())

    def pi_controller(self, world):
        # get current state
        x = world.get_object_pose(self._object_name)
        xdot = world.get_object_twist(self._object_name)

        # calculate error terms
        e = kdl.diff(x, self._goal_pose)
        edot = self._goal_twist - xdot

        # calculate control wrench and apply it
        control_law = self._p_gain * e + self._d_gain * edot
        # return kdl.Wrench(control_law.vel, control_law.rot)
        return kdl.Wrench(control_law.vel, kdl.Vector()) # TODO: fix rot control