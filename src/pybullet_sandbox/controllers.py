import interfaces as iface
import PyKDL as kdl

class FlyingObjectController(iface.CallbackInterface):
    def __init__(self, object_name, goal):
        self._object_name = object_name
        self._goal = goal

    def call(self, world):
        control_law = self.gravity_compensation(world) + self.pi_controller(world)
        world.apply_wrench(self._object_name, control_law)

    def gravity_compensation(self, world):
        m = world.get_object_mass(self._object_name)
        g = world.get_gravity()
        return kdl.Wrench(-m * g, kdl.Vector())

    def pi_controller(self, world):
        x = world.get_object_pose(self._object_name)
        xdot = world.get_object_twist(self._object_name)
        e = kdl.diff(x, self._goal)
        edot = -xdot # FIXME: get desired velocity from interpolator
        control_law = e + edot # FIXME: include some gains
        return kdl.Wrench(control_law.vel, control_law.rot)