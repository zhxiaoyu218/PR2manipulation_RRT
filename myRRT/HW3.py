#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW3 for EECS 598 Motion Planning
import time
import openravepy

#### YOUR IMPORTS GO HERE ####
handles = [];
    
#### END OF YOUR IMPORTS ####

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);        
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

def CovToFloat(path):
    path = path.split('\n')
    for line in xrange(len(path)):
      path[line] = path[line].split(',')
      for i in xrange(len(path[line])):
          path[line][i]=float(path[line][i])
    return path
    
def drawPath(path,robot,color,size):   
    if type(path) is str: path = CovToFloat(path)
    for i in path:
        robot.SetActiveDOFValues(i)
        handles.append(env.plot3(points=robot.GetLinks()[49].GetTransform()[0:3,3],pointsize=size,colors=color,drawstyle=1))


if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)

    env.Reset()        
    # load a scene from ProjectRoom environment XML file
    env.Load('hw3.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    ### INITIALIZE YOUR PLUGIN HERE ###
    RaveInitialize();
    RaveLoadPlugin('build/myRRT')
    myRRT = RaveCreateModule(env,'myRRT')
    ### END INITIALIZING YOUR PLUGIN ###
   

    # tuck in the PR2's arms for driving
    tuckarms(env,robot);
  
    #set start config
    jointnames =['l_shoulder_pan_joint','l_shoulder_lift_joint','l_elbow_flex_joint','l_upper_arm_roll_joint','l_forearm_roll_joint','l_wrist_flex_joint','l_wrist_roll_joint']
    robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])      
    startconfig = [-0.15,0.075,-1.008,0,0,-0.11,0]
    robot.SetActiveDOFValues(startconfig);
    robot.GetController().SetDesired(robot.GetDOFValues());

    handles = [];
    # lmodel = databases.linkstatistics.LinkStatisticsModel(robot)
    # if not lmodel.load():
    #     lmodel.autogenerate()
    # lmodel.setRobotWeights()
    # lmodel.setRobotResolutions(xyzdelta=0.01)
    
    with env:
        goalconfig = [0.449,-0.201,-0.151,0,0,-0.11,0]
        ### YOUR CODE HERE ###
        ###call your plugin to plan, draw, and execute a path from the current configuration of the left arm to the goalconfig
        goalbias = 0.06;
        step = 0.05;
        biflag = 0;
        MaxIteration = 200;

        myRRT.SendCommand('SetStrGoal str %f,%f,%f,%f,%f,%f,%f; goal %f,%f,%f,%f,%f,%f,%f;'%tuple(startconfig+goalconfig) );
        myRRT.SendCommand('SetPara %f,%f,%f,%f'%tuple([goalbias,step,biflag, MaxIteration]) );
       
        RRTtime = time.time();
        PathString = myRRT.SendCommand("FindPath");
        RRTtime = time.time() - RRTtime;

        drawPath(PathString,robot,[1,0,0],0.03);
        print 'find path time     ',  RRTtime;
        SmoothTime = time.time();
        SmoothedPathString = myRRT.SendCommand("PathSmooth");
        SmoothTime =time.time() - SmoothTime; 

        drawPath(SmoothedPathString,robot,[0,0,1],0.03);
        print 'smooth path time    ',  SmoothTime;
        print 'path length is ', len( CovToFloat(PathString) );
        print 'smoothed path length is ', len(CovToFloat(SmoothedPathString));
        # raw_input("Press enter to draw traj!!!!");

        # robot.SetActiveDOFValues(startconfig)
        path = CovToFloat(SmoothedPathString)
        # path.reverse();
        traj = RaveCreateTrajectory(env,'')
        traj.Init(robot.GetActiveConfigurationSpecification())

        for i in xrange(len(path)):
            traj.Insert(i,path[i])

        planningutils.RetimeActiveDOFTrajectory(traj,robot,hastimestamps=False,maxvelmult=1,maxaccelmult=1)
        # print 'trajectory time ',traj.GetDuration()
        
        

        robot.GetController().SetPath(traj)

    ### END OF YOUR CODE ###
    waitrobot(robot)

    raw_input("Press enter to exit...")

