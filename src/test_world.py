from __future__ import print_function
import time
import pybullet_sandbox.world as pw
import pybullet_sandbox.simulation as sim

my_world = pw.World()
my_world.add_urdf_object('plane.urdf', 'ground_plane', 'building_structure')

my_sim = sim.Simulation(my_world)
my_sim.register_pre_callback("bla", lambda w: print("bla"))
my_sim.register_post_callback("blub", lambda w: print("blub"))

print("object names: ", my_world.get_object_names())
print("object types: ", my_world.get_object_types())

for i in range(0,1):
    print(time.clock())
    my_sim.run(0.1)
    print(time.clock())