from bulletphysics import *

broadphase = DbvtBroadphase()
collisionConfiguration = DefaultCollisionConfiguration()
dispatcher = CollisionDispatcher(collisionConfiguration)

solver = SequentialImpulseConstraintSolver()

world = DiscreteDynamicsWorld(dispatcher, broadphase, solver,collisionConfiguration)

world.setGravity( Vector3(0, -10, 0) )

ground = StaticPlaneShape(Vector3(0,1,0),1)
fall = BoxShape(Vector3(10, 10, 10));

#ground_motionstate = DefaultMotionState( Transform(Quaternion(0,0,0,1), Vector3(0,-10,0)) )
ground_motionstate = DefaultMotionState( Transform(Quaternion(0.0, 0.0, 0.25881904510252074, 0.9659258262890683), Vector3(50,-10, 0)) )

# 0 is the mass
ground_rigidbody_info = RigidBodyConstructionInfo(0, ground_motionstate, ground, Vector3(0,0,0))

ground_rigidbody = RigidBody(ground_rigidbody_info)

world.addRigidBody(ground_rigidbody)

# box A
fall_motionstate = DefaultMotionState( Transform(Quaternion(0,0,0,1), Vector3(120, 150,0)) )

fall_mass = 100

local_inertia = Vector3(0,0,0)

fall.calculateLocalInertia(fall_mass, local_inertia)

# 1 is the mass, the last vector is the Inertia
fall_rigidbody_info = RigidBodyConstructionInfo(fall_mass, fall_motionstate, fall, local_inertia)

fall_rigidbody = RigidBody(fall_rigidbody_info)

world.addRigidBody(fall_rigidbody)

trans = Transform()

for i in range(0, 450):
    world.stepSimulation(1/60.0, 1)
    fall_motionstate.getWorldTransform(trans)
    print trans.getOrigin().getY()
    base = FreeCAD.Vector(trans.getOrigin().getX(),trans.getOrigin().getY(),trans.getOrigin().getZ() )
    rot = trans.getRotation()
    rotation = FreeCAD.Rotation( rot.getX(), rot.getY(), rot.getZ(), rot.getW() )
    FreeCAD.ActiveDocument.Box.Placement = FreeCAD.Placement(base, rotation)
    FreeCAD.Gui.updateGui()


world.removeRigidBody(fall_rigidbody)
world.removeRigidBody(ground_rigidbody)
