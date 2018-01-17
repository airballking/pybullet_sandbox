import pybullet as p
import pybullet_data as pdata
import PyKDL as kdl
from conversions import kdl_vector_to_list

#
# Exceptions
#

class WorldException(Exception):
    pass

class ObjectNameException(WorldException):
    pass

#
# Default values
#

default_use_gui = True
default_gravity = kdl.Vector(0,0,-9.81)
default_time_step = 0.02
default_urdf_path = pdata.getDataPath()

default_world_config = {
    'use_gui' : default_use_gui,
    'gravity' : default_gravity,
    'time_step' : default_time_step,
    'urdf_path' : default_urdf_path,
}

#
# Actual class implementations
#

class World(object):
    def __init__(self, use_gui=default_use_gui, gravity=default_gravity,
                 time_step=default_time_step, urdf_path=default_urdf_path):
        self._object_name_index = {}
        self._object_type_index = {}
        if use_gui:
            self._client_id = p.connect(p.GUI)
        else:
            self._client_id = p.connect(p.DIRECT)
        self.set_gravity(gravity)
        self.set_time_step(time_step)
        p.setAdditionalSearchPath(urdf_path)

    def __del__(self):
        p.disconnect()

    def add_urdf_object(self, filename, name, type, pose=kdl.Frame()):
        if name in self._object_name_index:
            raise ObjectNameException("The object name '" + name + "' is already taken.")

        if not type in self._object_type_index:
            self._object_type_index[type] = []

        id = p.loadURDF(filename, kdl_vector_to_list(pose.p), pose.M.GetQuaternion())

        self._object_name_index[name] = id
        self._object_type_index[type].append(id)

    def get_object_with_name(self, name):
        return self._object_name_index[name]

    def get_objects_with_type(self, type):
        return self._object_type_index[type]

    def get_object_names(self):
        return self._object_name_index.keys()

    def get_object_types(self):
        return self._object_type_index.keys()

    def get_time_step(self):
        return self.__time_step

    def set_time_step(self, time_step):
        self.__time_step = time_step
        p.setTimeStep(self.__time_step)

    def get_gravity(self):
        return self.__gravity

    def set_gravity(self, gravity):
        self.__gravity = gravity
        p.setGravity(*(kdl_vector_to_list(self.__gravity)))

    def step(self, time_step=None):
        if not time_step is None:
            self.set_time_step(time_step)
        p.stepSimulation(self._client_id)

    def get_object_mass(self, object_name):
        return p.getDynamicsInfo(self.get_object_with_name(object_name), -1)[0]

    def get_object_pose(self, object_name):
        trans, rot = p.getBasePositionAndOrientation(self.get_object_with_name(object_name))
        return kdl.Frame(kdl.Rotation.Quaternion(*rot), kdl.Vector(*trans))

    def get_object_twist(self, object_name):
        trans, rot = p.getBaseVelocity(self.get_object_with_name(object_name))
        return kdl.Twist(kdl.Vector(*trans), kdl.Vector(*rot))

    def apply_force(self, object_name, force):
        """ Applies a 3D force on a particular object.

        :param object_name: Name identifier of the object.
        :param force: 3D force to apply as a KDL Vector.
        :return: nothing
        """
        object_id = self.get_object_with_name(object_name)
        force_list = kdl_vector_to_list(force)
        link_id = -1
        p.applyExternalForce(object_id, link_id, force_list, [0, 0, 0], p.LINK_FRAME)

