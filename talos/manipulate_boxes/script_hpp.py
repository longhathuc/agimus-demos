# Copyright 2018, 2019, 2020 CNRS - Airbus SAS
# Author: Florent Lamiraux, Joseph Mirabel, Alexis Nicolin
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:

# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import argparse, numpy as np
from hpp import Quaternion, Transform
from hpp.corbaserver.manipulation import Constraints, ProblemSolver, newProblem
from hpp.corbaserver.manipulation.robot import CorbaClient
from hpp.corbaserver import loadServerPlugin, createContext

from common_hpp import *

# parse arguments
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

print("context=" + args.context)
loadServerPlugin (args.context, "manipulation-corba.so")

isSimulation = args.context == "simulation"

footPlacement = not isSimulation
comConstraint = not isSimulation
constantWaistYaw = not isSimulation
fixedArmWhenGrasping = not isSimulation

client = CorbaClient(context=args.context)
newProblem(client=client.manipulation, name=args.context)

robot, ps, vf, table, objects = makeRobotProblemAndViewerFactory(client, rolling_table=True,
        rosParam=args.ros_param)
if isSimulation:
    ps.setMaxIterProjection (1)


q_neutral = robot.getCurrentConfig()

# Set robot to neutral configuration before building constraint graph
robot.setCurrentConfig(q_neutral)
ps.setDefaultLineSearchType ("ErrorNormBased")
ps.setMaxIterPathPlanning (40)

# create locked joint for table

com_constraint, foot_placement, foot_placement_complement = \
    createQuasiStaticEquilibriumConstraint (ps, init_conf)
gaze_constraints = createGazeConstraints (ps)
gaze_cost = createGazeCost (ps)
waist_constraint = createWaistYawConstraint (ps)
left_arm_lock  = createLeftArmLockedJoints (ps)
right_arm_lock = createRightArmLockedJoints (ps)
left_gripper_lock, right_gripper_lock = \
    createGripperLockedJoints (ps, init_conf)
table_lock = createTableLockedJoint (ps, table, init_conf)

graph, factory = makeGraph(ps, table, objects)

# Add other locked joints in the edges.
for edgename, edgeid in graph.edges.items():
    graph.addConstraints(
        edge=edgename, constraints=Constraints(numConstraints=table_lock)
    )
# Add gaze and and COM constraints to each node of the graph
if comConstraint:
    for nodename, nodeid in graph.nodes.items():
        graph.addConstraints(
            node=nodename, constraints=Constraints(numConstraints=\
                com_constraint
            )
        )

# Add locked joints and foot placement constraints in the graph,
# add foot placement complement in each edge.
if footPlacement:
    for edgename, edgeid in graph.edges.items():
        graph.addConstraints(
            edge=edgename,
            constraints=Constraints(numConstraints=foot_placement_complement),
        )

if constantWaistYaw:
    for edgename, edgeid in graph.edges.items():
        graph.addConstraints(
            edge=edgename, constraints=Constraints(
                numConstraints=waist_constraint
            )
        )

graph.addConstraints(
    graph=True,
    constraints=Constraints(
        numConstraints=foot_placement + left_gripper_lock + right_gripper_lock,
    ),
)

# On the real robot, the initial configuration as measured by sensors is very
# likely not in any state of the graph. State "starting_state" and transition
# "starting_motion" are aimed at coping with this issue.
graph.createNode("starting_state")
graph.createEdge("starting_state", "free", "starting_motion", isInNode="starting_state")
graph.createEdge(
    "starting_state", "starting_state", "loop_ss", isInNode="starting_state", weight=0
)
graph.createEdge(
    "free",
    "starting_state",
    "go_to_starting_state",
    isInNode="starting_state",
    weight=0,
)
graph.addConstraints(
    node="starting_state", constraints=Constraints(numConstraints=["place_box"])
)
graph.addConstraints(
    edge="loop_ss",
    constraints=Constraints(
        numConstraints=["place_box/complement"] + table_lock
    ),
)
graph.addConstraints(
    edge="starting_motion",
    constraints=Constraints(
        numConstraints=["place_box/complement"] + table_lock
    ),
)
graph.addConstraints(
    edge="go_to_starting_state",
    constraints=Constraints(
        numConstraints=["place_box/complement"] + table_lock
    ),
)
# Add appropriate gaze constraint for each node
prefixLeft = 'talos/left_gripper > box/handle'
prefixRight = 'talos/right_gripper > box/handle'
l = len(prefixLeft)
r = len(prefixRight)

for n in graph.nodes.keys():
    if n[:l] == prefixLeft:
        graph.addConstraints(
            node=n,
            constraints=Constraints(numConstraints=["look_left_hand"]))
    elif n[:r] == prefixRight:
        graph.addConstraints(
            node=n,
            constraints=Constraints(numConstraints=["look_right_hand"]))

# Add a transition to look at the box
graph.createNode('look_at_box')
graph.createEdge('free', 'look_at_box', 'look_to_box', isInNode='free',
                 weight=0)
graph.addConstraints(edge='look_to_box',
    constraints=Constraints(numConstraints=table_lock+['place_box/complement']))
if comConstraint:
    graph.addConstraints(node='look_at_box',
                         constraints=Constraints(numConstraints=com_constraint))

if footPlacement:
    graph.addConstraints(
        edge='look_to_box',
        constraints=Constraints(numConstraints=foot_placement_complement))

if constantWaistYaw:
    graph.addConstraints(edge='look_to_box',
        constraints=Constraints(numConstraints=waist_constraint))

createGazeConstraint(ps)
graph.addConstraints(node='look_at_box', constraints=
                     Constraints(numConstraints=["look_at_box"]))

# Loop transitions
e_l_l1 = "Loop | 0-0"
e_l_r1 = "Loop | 1-0"
e_l_l2 = "Loop | 0-1"
e_l_r2 = "Loop | 1-1"
e_l_l3 = "Loop | 0-2"
e_l_r3 = "Loop | 1-2"
e_l_l4 = "Loop | 0-3"
e_l_r4 = "Loop | 1-3"
# Transitions from state 'free'
e_l1 = "talos/left_gripper > box/handle1 | f"
e_r1 = "talos/right_gripper > box/handle1 | f"
e_l2 = "talos/left_gripper > box/handle2 | f"
e_r2 = "talos/right_gripper > box/handle2 | f"
e_l3 = "talos/left_gripper > box/handle3 | f"
e_r3 = "talos/right_gripper > box/handle3 | f"
e_l4 = "talos/left_gripper > box/handle4 | f"
e_r4 = "talos/right_gripper > box/handle4 | f"
# Transitions from one grasp to two grasps
e_l1_r2 = "talos/right_gripper > box/handle2 | 0-0"
e_l1_r4 = "talos/right_gripper > box/handle4 | 0-0"
e_r1_l2 = "talos/left_gripper > box/handle2 | 1-0"
e_r1_l4 = "talos/left_gripper > box/handle4 | 1-0"
e_l2_r1 = "talos/right_gripper > box/handle1 | 0-3"
e_l2_r3 = "talos/right_gripper > box/handle3 | 0-3"
e_r2_l1 = "talos/left_gripper > box/handle3 | 1-3"
e_r2_l3 = "talos/left_gripper > box/handle3 | 1-3"
e_r3_l4 = "talos/left_gripper > box/handle4 | 1-2"
e_r3_l2 = "talos/left_gripper > box/handle2 | 1-2"
e_l3_r4 = "talos/right_gripper > box/handle4 | 0-2"
e_l3_r2 = "talos/right_gripper > box/handle2 | 0-2"
e_l4_r1 = "talos/right_gripper > box/handle1 | 0-3"
e_l4_r3 = "talos/right_gripper > box/handle3 | 0-3"
e_r4_l1 = "talos/left_gripper > box/handle1 | 1-3"
e_r4_l3 = "talos/left_gripper > box/handle3 | 1-3"
# Transition from 'free' to first waypoint
e_l1_app = e_l1 + "_01"
e_r1_app = e_r1 + "_01"
e_l2_app = e_l2 + "_01"
e_r2_app = e_r2 + "_01"
e_l3_app = e_l3 + "_01"
e_r3_app = e_r3 + "_01"
e_l4_app = e_l4 + "_01"
e_r4_app = e_r4 + "_01"

if fixedArmWhenGrasping:
    leftArmConstraint = Constraints(numConstraints=left_arm_lock)
    rightArmConstraint = Constraints(numConstraints=right_arm_lock)

    graph.addConstraints(edge=e_l1_app, constraints=rightArmConstraint)
    graph.addConstraints(edge=e_r1_app, constraints=leftArmConstraint)
    graph.addConstraints(edge=e_l2_app, constraints=rightArmConstraint)
    graph.addConstraints(edge=e_r2_app, constraints=leftArmConstraint)
    graph.addConstraints(edge=e_l3_app, constraints=rightArmConstraint)
    graph.addConstraints(edge=e_r3_app, constraints=leftArmConstraint)
    graph.addConstraints(edge=e_l4_app, constraints=rightArmConstraint)
    graph.addConstraints(edge=e_r4_app, constraints=leftArmConstraint)
    graph.initialize()


ps.setRandomSeed(123)
ps.selectPathProjector("Progressive", 0.2)
graph.setWeight ('Loop | f', 1)

graph.initialize()
for edge in [ e_l1_r2, e_l1_r4, e_r1_l2, e_r1_l4, e_l2_r1, e_l2_r3, e_r2_l1, e_r2_l3, e_r3_l4, e_r3_l2, e_l3_r4, e_l3_r2, e_l4_r1, e_l4_r3, e_r4_l1, e_r4_l3,]:
    addCostToComponent(graph, gaze_cost, edge=edge+"_01")

q_init = init_conf[::]

setGaussianShooter (ps, table, objects, q_init, 0.1)

# Set Optimization parameters
ps.setParameter("SimpleTimeParameterization/safety", 0.25)
ps.setParameter("SimpleTimeParameterization/order", 2)
ps.setParameter("SimpleTimeParameterization/maxAcceleration", 1.0)
ps.setParameter("ManipulationPlanner/extendStep", 0.7)
ps.setMaxIterPathPlanning(50)

if isSimulation:
    ps.setInitialConfig(q_init)
if args.context == defaultContext:
    # Define problem
    res, q_init, err = graph.generateTargetConfig("Loop | f", q_init, q_init)
    if not res:
        raise RuntimeError("Failed to project initial configuration")

    q_goal = q_init[::]
    rank = robot.rankInConfiguration["box/root_joint"]
    q_goal[rank + 3 : rank + 7] = (
        Quaternion([0, 1, 0, 0]) * Quaternion(q_init[rank + 3 : rank + 7])).\
        toTuple()
    # res, q_goal, err = graph.applyNodeConstraints ('free', q_goal)
    res, q_proj, err = graph.generateTargetConfig("Loop | f", q_goal, q_goal)
    if not res:
        raise RuntimeError("Failed to project goal configuration")
    assert q_init[-7:] == q_goal[-7:]

    solver = Solver(
        ps,
        graph,
        q_init,
        q_goal,
        e_l_l1,
        e_l_r1,
        e_l_l2,
        e_l_r2,
        e_l_l3,
        e_l_r3,
        e_l_l4,
        e_l_r4,
        e_l1,
        e_r1,
        e_l2,
        e_r2,
        e_l3,
        e_r3,
        e_l4,
        e_r4,
        e_l1_r2,
        e_l1_r4,
        e_r1_l2,
        e_r1_l4,
        e_l2_r1,
        e_l2_r3,
        e_r2_l1,
        e_r2_l3,
        e_r3_l4,
        e_r3_l2,
        e_l3_r4,
        e_l3_r2,
        e_l4_r1,
        e_l4_r3,
        e_r4_l1,
        e_r4_l3,
    )

    qBoxVisible, pathId = solver.makeBoxVisibleFrom(q_init, True, True)

# From an estimated configuration with position of objects
# solver.solveFromEstimatedConfiguration (init_conf)

## Solving with ManipulationRRT and random shortcut takes approximately 2 minutes
# ps.setMaxIterPathPlanning(1000)
# ps.clearPathOptimizers ()
# ps.addPathOptimizer ("RandomShortcut")
# ps.addPathOptimizer ("SimpleTimeParameterization")
# ps.setInitialConfig (q_init)
# ps.addGoalConfig (q_goal)
# time = ps.solve ()
