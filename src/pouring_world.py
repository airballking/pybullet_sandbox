import pybullet as p
import pybullet_data
import rospy

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
p.setGravity(0,0,-9.81)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
mug_id = p.loadURDF("/home/gbartels/ros/giskard/src/pybullet_sandbox/data/mug/mug.urdf")
# mug_id = p.loadSDF("data/mug/mug.sdf", cubeStartPos, cubeStartOrientation)

p.setRealTimeSimulation(1)

rospy.sleep(5)
print('Time is up')
p.disconnect()
