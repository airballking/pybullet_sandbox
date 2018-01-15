import pybullet as p
import time
from math import cos, sin, sqrt

import pybullet_data
import rospkg

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
filename = rospkg.RosPack().get_path("pybullet_sandbox")+"/data/mug/mug.urdf"
boxId = p.loadURDF(filename)
box_pose = p.getBasePositionAndOrientation(boxId)


sphere_path = rospkg.RosPack().get_path("pybullet_sandbox")+"/data/sphere.urdf"
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
# sphere packing algorithm taken from wiki: https://en.wikipedia.org/wiki/Close-packing_of_equal_spheres

r = 0.002
p1 = 7
d = 3
print 4*p1*p1*(d)
for i in range(-p1, p1):
    for j in range(-p1, p1):
        for k in range(0, d):
            x = r * (2*i + (j+k) % 2)
            y = r * sqrt(3) * (j + (k % 2)/3.0)
            z = r * sqrt(6) * 2 * k / 3.0

            p.loadURDF(sphere_path, [x,y,z+0.01])
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
print "starting sim"
p.setRealTimeSimulation(1)
while True:
    time.sleep(5)

p.disconnect()