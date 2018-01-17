import time
import pybullet_sandbox.world as pw
import pybullet_sandbox.simulation as sim
import pybullet_sandbox.controllers as ctrl
import rospkg
import PyKDL as kdl

my_world = pw.World()
my_world.add_urdf_object('plane.urdf', 'ground_plane_1', 'floor')
mug_file = rospkg.RosPack().get_path("pybullet_sandbox") + "/data/mug/mug.urdf"
mug_init_pose = kdl.Frame(kdl.Vector(0.1, 0.2, 0.75))
my_world.add_urdf_object(mug_file, 'mug_1', 'mug', mug_init_pose)

my_sim = sim.Simulation(my_world)
mug_goal_pose = kdl.Frame(kdl.Rotation.RotX(3.14/4.0), kdl.Vector(0.3, 0.4, 0.75))
my_sim.register_pre_callback("hovering_controller", ctrl.FlyingObjectController("mug_1", mug_goal_pose))

while True:
    my_sim.step()
    time.sleep(my_sim.get_time_step())