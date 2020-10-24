from hpp.corbaserver.manipulation import Robot, loadServerPlugin, createContext, newProblem, ProblemSolver, ConstraintGraph, Rule, Constraints, CorbaClient
from hpp.gepetto.manipulation import ViewerFactory
import sys, argparse

# parse arguments
defaultContext = "corbaserver"
p = argparse.ArgumentParser (description=
                             'Initialize demo of Pyrene manipulating a box')
p.add_argument ('--context', type=str, metavar='context',
                default=defaultContext,
                help="identifier of ProblemSolver instance")
p.add_argument ('--ros-param', type=str, metavar='ros_param',
                help="The name of the ROS param containing the URDF.")
args = p.parse_args ()
if args.context != defaultContext:
    createContext (args.context)
isSimulation = args.context == "simulation"

Robot.urdfFilename = "package://tiago_data/robots/tiago.urdf"
Robot.srdfFilename = "package://tiago_data/srdf/tiago.srdf"

class Box:
    urdfFilename = "package://agimus_demos/urdf/box_with_qr.urdf"
    srdfFilename = "package://agimus_demos/srdf/box_with_qr.srdf"
    rootJointType = "anchor"

## Reduce joint range for security
def shrinkJointRange (robot, ratio):
    for j in robot.jointNames:
        if j[:6] != "tiago/": continue
        tj = j[6:]
        if tj.startswith("torso") or tj.startswith("arm") or tj.startswith("head"):
            bounds = robot.getJointBounds (j)
            if len (bounds) == 2:
                width = bounds [1] - bounds [0]
                mean = .5 * (bounds [1] + bounds [0])
                m = mean - .5 * ratio * width
                M = mean + .5 * ratio * width
                robot.setJointBounds (j, [m, M])

print("context=" + args.context)
loadServerPlugin (args.context, "manipulation-corba.so")
client = CorbaClient(context=args.context)
client.manipulation.problem.selectProblem (args.context)

robot = Robot("robot", "tiago", rootJointType="planar", client=client)
robot.setJointBounds('tiago/root_joint', [-2, 2, -2, 2])
#robot.insertRobotSRDFModel("tiago", "tiago_data", "schunk", "_gripper")
ps = ProblemSolver(robot)
vf = ViewerFactory(ps)
vf.loadRobotModel (Box, "box")
robot.insertRobotSRDFModel("box", "gerard_bauzil", "")
robot.setJointBounds('box/root_joint', [-2, 2, -2, 2, 0, 2])

ps.selectPathValidation("Graph-Dichotomy", 0)
ps.selectPathProjector("Progressive", 0.2)
ps.addPathOptimizer("EnforceTransitionSemantic")
ps.addPathOptimizer("SimpleTimeParameterization")
if isSimulation:
    ps.setMaxIterProjection (1)

ps.setParameter("SimpleTimeParameterization/safety", 0.25)
ps.setParameter("SimpleTimeParameterization/order", 2)
ps.setParameter("SimpleTimeParameterization/maxAcceleration", 1.0)
ps.setParameter("ManipulationPlanner/extendStep", 0.7)

vf.loadObjectModel (Box, "box")

shrinkJointRange(robot, 0.95)

q0 = robot.getCurrentConfig()
q0[:4] = [0, -0.9, 0, 1]
q0[robot.rankInConfiguration['tiago/torso_lift_joint']] = 0.15
q0[robot.rankInConfiguration['tiago/arm_1_joint']] = 0.10
q0[robot.rankInConfiguration['tiago/arm_2_joint']] = -1.47
q0[robot.rankInConfiguration['tiago/arm_3_joint']] = -0.16
q0[robot.rankInConfiguration['tiago/arm_4_joint']] = 1.87
q0[robot.rankInConfiguration['tiago/arm_5_joint']] = -1.57
q0[robot.rankInConfiguration['tiago/arm_6_joint']] = 0.01
q0[robot.rankInConfiguration['tiago/arm_7_joint']] = 0.00
q0[robot.rankInConfiguration['tiago/gripper_left_finger_joint']] = 0
q0[robot.rankInConfiguration['tiago/gripper_right_finger_joint']] = 0


def lockJoint(jname, q, cname=None):
    if cname is None:
        cname = jname
    s = robot.rankInConfiguration[jname]
    e = s+robot.getJointConfigSize(jname)
    ps.createLockedJoint(cname, jname, q[s:e])
    ps.setConstantRightHandSide(cname, True)
    return cname

ljs = list()
ljs.append(lockJoint("tiago/root_joint", q0))

for n in robot.jointNames:
    if n.startswith('tiago/hand_'):
        ljs.append(lockJoint(n, q0))

ps.createPositionConstraint("gaze", "tiago/xtion_rgb_optical_frame", "box/tag_joint",
        (0,0,0), (0,0,0), (True,True,False))

from hpp.corbaserver.manipulation import ConstraintGraphFactory
graph = ConstraintGraph(robot, 'graph')
factory = ConstraintGraphFactory(graph)
factory.setGrippers([ "tiago/gripper", "driller/drill_tip", ])
factory.setObjects([ "driller", "skin", ],
        [ [ "driller/handle", ], [ "skin/hole", ], ],
        [ [ ], [ ], ])

factory.setRules([
    # Tiago always hold the gripper.
    Rule([ "tiago/gripper", ], [ "driller/handle", ], True), Rule([ "tiago/gripper", ], [ ".*", ], False),
    # Allow to associate drill_tip with skin/hole only.
    Rule([ "driller/drill_tip", ], [ "driller/handle", ], False), Rule([ "driller/drill_tip", ], [ ".*", ], True), ])
factory.generate()

graph.addConstraints(graph=True, constraints=Constraints(numConstraints=ljs))
for n in [ 'driller/drill_tip > skin/hole | 0-0_pregrasp', 'tiago/gripper grasps driller/handle : driller/drill_tip grasps skin/hole' ]:
    graph.addConstraints(node=n, constraints=Constraints(numConstraints=["gaze"]))
graph.initialize()

# Constraint in this state are explicit so ps.setMaxIterProjection(1) should not
# make it fail.
res, q1, err = graph.applyNodeConstraints('tiago/gripper grasps driller/handle', q0)
q1valid, msg = robot.isConfigValid(q1)
if not q1valid:
    print(msg)
assert res

ps.setInitialConfig(q1)

if not isSimulation:
    qrand = q1
    for i in range(100):
        q2valid, q2, err = graph.generateTargetConfig('driller/drill_tip > skin/hole | 0-0', q1, qrand)
        if q2valid:
            q2valid, msg = robot.isConfigValid(q2)
        if q2valid:
            break
        qrand = robot.shootRandomConfig()
    assert q2valid

    if not isSimulation:
        ps.addGoalConfig(q2)
        ps.solve()

    try:
        v = vf.createViewer()
        v (q1)
    except:
        pass
