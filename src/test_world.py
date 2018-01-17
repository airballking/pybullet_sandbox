from __future__ import print_function
import time
import pybullet_sandbox.world as pw
import pybullet_sandbox.simulation as sim
import pybullet_sandbox.interfaces as iface

class PrintCallback(iface.CallbackInterface):
    def __init__(self, message):
        self._message = message

    def call(self, _):
        print(self._message)

my_world = pw.World()
my_world.add_urdf_object('plane.urdf', 'ground_plane', 'building_structure')

my_sim = sim.Simulation(my_world)
my_sim.register_pre_callback("bla", PrintCallback("pre callback"))
my_sim.register_post_callback("blub", PrintCallback("post callback"))

print("object names: ", my_world.get_object_names())
print("object types: ", my_world.get_object_types())

for i in range(0,1):
    print(time.clock())
    my_sim.run(0.1)
    print(time.clock())