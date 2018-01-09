import pybullet as p
import pybullet_data

import time

useMaximalCoordinates = 0

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF

p.loadSDF("stadium.sdf",useMaximalCoordinates=useMaximalCoordinates)
monastryId = concaveEnv = p.createCollisionShape(p.GEOM_MESH, fileName="samurai_monastry.obj",
                                                 flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
orn = p.getQuaternionFromEuler([1.5707963, 0, 0])
p.createMultiBody(0, monastryId, baseOrientation=orn)

sphereRadius = 0.05
colSphereId = p.createCollisionShape(p.GEOM_SPHERE, radius=sphereRadius)
colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=[sphereRadius, sphereRadius, sphereRadius])

mass = .01
visualShapeId = -1
use_boxes = True

for i in range(5):
    for j in range(5):
        for k in range(5):
             if (k & 2) and use_boxes:
                 sphereUid = p.createMultiBody(mass, colSphereId, visualShapeId,
                                               [-i * 2.5 * sphereRadius, j * 2.5 * sphereRadius, k * 2.5 * sphereRadius + 1],
                                               useMaximalCoordinates=useMaximalCoordinates)
             else:
                 sphereUid = p.createMultiBody(mass, colBoxId, visualShapeId,
                                               [-i * 2.5 * sphereRadius, j * 2.5 * sphereRadius, k * 2.5 * sphereRadius + 1],
                                               useMaximalCoordinates=useMaximalCoordinates)



             p.changeDynamics(sphereUid, -1, spinningFriction=0.001, rollingFriction=0.001, linearDamping=0.0, restitution=0.1)

p.setGravity(0, 0, -10)
p.setRealTimeSimulation(1)

while (1):
    keys = p.getKeyboardEvents()
# print(keys)
time.sleep(0.01)