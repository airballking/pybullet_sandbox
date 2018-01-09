# I copied this benchmarking script from this github issue:
#   https://github.com/bulletphysics/bullet3/issues/1416
#

import pybullet as p
import pybullet_data

import time
import os.path as path
import argparse

parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)

parser.add_argument('--max', help='Use maximal coordinates (rigidbody and constraints) instead of default reduced coordinates, 0 = no, 1 = yes',type=int,  default=0)

parser.add_argument('--mode', help='Connection mode: 1=GUI, 2=DIRECT, 3=SHARED_MEMORY, 4=UDP, 5=TCP (default = 2, DIRECT)',type=int,  default=2)
parser.add_argument('--iterations', help='Number of constraint solver iterations (default = 10)',type=int,  default=10)


args = parser.parse_args()

useMaximalCoordinates = args.max 
print("--max="+str(useMaximalCoordinates)+" (useMaximalCoordinates)")

cid = -1
print("--mode="+str(args.mode) + "(connection mode)")
if (args.mode==1):
	cid=p.connect(p.GUI)
	print("GUI")
if (args.mode==2):
        cid=p.connect(p.DIRECT)
	print("DIRECT")
if (args.mode==3):
	cid=p.connect(p.SHARED_MEMORY)
	print("SHARED_MEMORY")
if (args.mode==4):
	cid=p.connect(p.UDP, "localhost")
	print("UDP")
if (args.mode==5):
	cid=p.connect(p.TCP, "localhost")
	print("TCP")
if (cid<0):
	print("Cannot connect to physics server")
	exit(0)

p.resetSimulation()

print("--iterations="+str(args.iterations)+" (constraint solver iterations)")
p.setPhysicsEngineParameter(numSolverIterations=args.iterations)

logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "createSphereAndSimTimings.json")

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
monastryId = concaveEnv = p.createCollisionShape(p.GEOM_MESH, fileName="samurai_monastry.obj",
                                                 flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
orn = p.getQuaternionFromEuler([1.5707963,0,0])
p.createMultiBody (0,monastryId, baseOrientation=orn, useMaximalCoordinates=useMaximalCoordinates)

sphereRadius = 0.05
colSphereId = p.createCollisionShape(p.GEOM_SPHERE,radius=sphereRadius)
colBoxId = p.createCollisionShape(p.GEOM_BOX,halfExtents=[sphereRadius,sphereRadius,sphereRadius])

mass = 1
visualShapeId = -1


n_objects = 0
t0 = time.clock()

for i in range (5):
	for j in range (5):
		for k in range (5):

			if (k&2):
				sphereUid = p.createMultiBody(mass,colSphereId,visualShapeId,[-i*2*sphereRadius,j*2*sphereRadius,k*2*sphereRadius+1],useMaximalCoordinates=useMaximalCoordinates)
			else:
				sphereUid = p.createMultiBody(mass,colBoxId,visualShapeId,[-i*2*sphereRadius,j*2*sphereRadius,k*2*sphereRadius+1], useMaximalCoordinates=useMaximalCoordinates)

			p.changeDynamics(sphereUid,-1,spinningFriction=0.001, rollingFriction=0.001,linearDamping=0.0)
			n_objects += 1


t_objects = time.clock() - t0
t_object = t_objects / float(n_objects)


p.setGravity(0,0,-10)
p.setRealTimeSimulation(0)
p.setTimeStep(0.005)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

t0 = time.clock()
n_steps = 1000

for t in xrange(n_steps):
	p.stepSimulation()

t_simulation = time.clock() - t0
t_step = t_simulation / float(n_steps)
total = t_simulation + t_object

p.stopStateLogging(logId)

p.disconnect()

time.sleep(2)

print
print
print
print
print "-------------------------------------------------"
print '                 \t  Iter  \t Total  '
print 'Object Creation  \t{0:.3e}s\t{1:.3f}s'.format(t_object, t_objects)
print 'Simulation       \t{0:.3e}s\t{1:.3f}s'.format(t_step, t_simulation)
print "-------------------------------------------------"
print 'Simulation       \t        \t{0:.3f}s'.format(total)

