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
#ground_motionstate = DefaultMotionState( Transform(Quaternion(0.0, 0.0, 0.25881904510252074, 0.9659258262890683), Vector3(50,-10, 0)) )
ground_motionstate = DefaultMotionState( Transform(Quaternion(0.0, 0.0, 0.043619387365336, 0.9990482215818578)
, Vector3(0,-10,0)) )
# 0 is the mass
ground_rigidbody_info = RigidBodyConstructionInfo(0, ground_motionstate, ground, Vector3(0,0,0))

ground_rigidbody = RigidBody(ground_rigidbody_info)

world.addRigidBody(ground_rigidbody)

motion_obj = []
def createBox(x,y,z):
	# fc box
    bx = FreeCAD.ActiveDocument.addObject('Part::Feature','Box')
    bx.Shape = Part.makeBox( 20, 20, 20 )
    # box A
    fall_motionstate = DefaultMotionState( Transform(Quaternion(0,0,0,1), Vector3(x, y,z )) )
    fall_mass = 10
    local_inertia = Vector3(0,0,0)
    fall.calculateLocalInertia(fall_mass, local_inertia)
    # 1 is the mass, the last vector is the Inertia
    fall_rigidbody_info = RigidBodyConstructionInfo(fall_mass, fall_motionstate, fall, local_inertia)
    fall_rigidbody = RigidBody(fall_rigidbody_info)
    world.addRigidBody(fall_rigidbody)
    motion_obj.append( (bx, fall_motionstate, fall_rigidbody) )



trans = Transform()
for i in xrange(2):
    for s in xrange(20):
        for v in xrange(2):
            createBox( i*20, 200 + s*20, v*20 )


for i in range(0, 600):
    world.stepSimulation(1/2.0, 4 )
    for obj in motion_obj:
        obj[1].getWorldTransform(trans)
        base = FreeCAD.Vector(trans.getOrigin().getX(),trans.getOrigin().getY(),trans.getOrigin().getZ() )
        rot = trans.getRotation()
        rotation = FreeCAD.Rotation( rot.getX(), rot.getY(), rot.getZ(), rot.getW() )
        obj[0].Placement = FreeCAD.Placement(base, rotation)

    FreeCAD.Gui.updateGui()


for obj in motion_obj:
    FreeCAD.ActiveDocument.removeObject(obj[0].Name)
    world.removeRigidBody(obj[2])


world.removeRigidBody(ground_rigidbody)

