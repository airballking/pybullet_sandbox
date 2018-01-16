import time
import pybullet_sandbox.world as pw
import pybullet_sandbox.simulation as sim

my_world = pw.World()
my_world.add_urdf_object('plane.urdf', 'ground_plane', 'building_structure')

my_sim = sim.Simulation(my_world)

print "object names: ", my_world.get_object_names()
print "object types: ", my_world.get_object_types()

while True:
    my_sim.step()
    time.sleep(my_sim.get_time_step())
