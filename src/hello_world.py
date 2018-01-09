import pybullet as p
import pybullet_data
import rospy

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("r2d2.urdf",cubeStartPos, cubeStartOrientation)

#p.stepSimulation() # step simulation once, no timestep argument available
p.setRealTimeSimulation(1) # enables real-time simulation
                           # probably uses this this thread, somehow

cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
rospy.sleep(5)
print('Time is up')
p.disconnect()