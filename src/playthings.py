import pybullet as p
import time
import pybullet_data
import rospkg

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
filename = rospkg.RosPack().get_path("my_pkg")+"/data/mug/mug.urdf"
boxId = p.loadURDF(filename)

p.setRealTimeSimulation(1)
while True:
    time.sleep(5)

p.disconnect()