from visual import *
import pykinect
from pykinect import nui
from pykinect.nui import JointId
import math
import numpy as np
from naoqi import ALProxy


class Skeleton:
    """Kinect skeleton represented as a VPython frame.
    """

    def __init__(self, f):
        """Create a skeleton in the given VPython frame f.
        """
        self.frame = f
        self.joints = [sphere(frame=f, radius=0.08, color=color.yellow)
                       for i in range(20)]
        #Head
        self.joints[3].radius = 0.125
        self.bones = [cylinder(frame=f, radius=0.05, color=color.yellow)
                      for bone in _bone_ids]

    def update(self):
        """Update the skeleton joint positions in the depth sensor frame.

        Return true if the most recent sensor frame contained a tracked
        skeleton.
        """
        updated = False
        for skeleton in _kinect.skeleton_engine.get_next_frame().SkeletonData:
            if skeleton.eTrackingState == nui.SkeletonTrackingState.TRACKED:

                # Move the joints.
                for joint, p in zip(self.joints, skeleton.SkeletonPositions):
                    joint.pos = (p.x, p.y, p.z)

                # Move the bones.
                for bone, bone_id in zip(self.bones, _bone_ids):
                    p1, p2 = [self.joints[id].pos for id in bone_id]
                    bone.pos = p1
                    bone.axis = p2 - p1
                updated = True
        return updated

def draw_sensor(f):
    """Draw 3D model of the Kinect sensor.

    Draw the sensor in the given (and returned) VPython frame f, with
    the depth sensor frame aligned with f.
    """
    box(frame=f, pos=(0, 0, 0), length=0.2794, height=0.0381, width=0.0635,
        color=color.blue)
    cylinder(frame=f, pos=(0, -0.05715, 0), axis=(0, 0.0127, 0), radius=0.0381,
             color=color.blue)
    cone(frame=f, pos=(0, -0.04445, 0), axis=(0, 0.01905, 0), radius=0.0381,
         color=color.blue)
    cylinder(frame=f, pos=(0, -0.05715, 0), axis=(0, 0.0381, 0), radius=0.0127,
             color=color.blue)
    cylinder(frame=f, pos=(-0.0635, 0, 0.03175), axis=(0, 0, 0.003),
             radius=0.00635, color=color.red)
    cylinder(frame=f, pos=(-0.0127, 0, 0.03175), axis=(0, 0, 0.003),
             radius=0.00635, color=color.red)
    cylinder(frame=f, pos=(0.0127, 0, 0.03175), axis=(0, 0, 0.003),
             radius=0.00635, color=color.red)
    text(frame=f, text='VAMK', pos=(0.06985, -0.00635, 0.03175),
         align='center', height=0.0127, depth=0.003)
    return f

def analyse_frame(skeleton,raised,motionProxy):
    if skeleton.frame.visible:
        # results
        angles = {}
        # Original Skeleton info
        ojoints = []
        for index in range(20):
            ojoints.append(np.array([skeleton.joints[index].x, skeleton.joints[index].y, skeleton.joints[index].z]))
       
       
        # New axis space
        x = (ojoints[12] - ojoints[16]) / (np.linalg.norm(ojoints[12] - ojoints[16]))
        z = np.cross((ojoints[12] - ojoints[16]), (ojoints[16] - ojoints[2])) / np.linalg.norm(np.cross((ojoints[12] - ojoints[16]), (ojoints[16] - ojoints[2])))
        y = np.cross(x,z)
        # Transfer matrix
        A = np.array([x,y,z])

        # Coordinate transform
        joints = []
        for ojoint in ojoints:
            # joints.append(A.dot(ojoint))
            joints.append(ojoint)

        # New joints
        # Head
        head =  joints[3]
        neck =  joints[2]
        # Right
        right_shoulder = joints[8]
        right_elbow =  joints[9]
        right_wrist =  joints[10]
        # Left 
        left_shoulder =joints[4]
        left_elbow =  joints[5]
        left_wrist =  joints[6]
        # Legs
        left_hip =  joints[12] 
        right_hip =  joints[16]
        left_knee =  joints[13]
        right_knee = joints[17] 

    ## Left arm
        # Bonds
        # 45
        uparmLeft = left_elbow - left_shoulder 
        # 56
        forearmLeft = left_wrist - left_elbow

        pi = math.pi
        ## Shoulder angle
        LShoulderRoll = -(pi/2 - math.acos(np.cross(y,z).dot(uparmLeft)/(np.linalg.norm(np.cross(y,z)) * np.linalg.norm(uparmLeft))))
        # around 40 degree different
        LShoulderPitch = math.atan(uparmLeft[2]/uparmLeft[0])
        # LShoulderPitch = LShoulderPitch - math.radians(40)
        # print math.degrees(LShoulderRoll)
        # print math.degrees(LShoulderPitch)

        ## Elbow angle
        b1=np.array([[math.cos(LShoulderRoll), -math.sin(LShoulderRoll), 0],
                    [math.sin(LShoulderRoll), math.cos(LShoulderRoll), 0],
                    [0, 0, 1]])
        b2=np.array([[math.cos(LShoulderPitch), 0, math.sin(LShoulderPitch)],
                    [0, 1, 0],
                    [-math.sin(LShoulderPitch), 0, math.cos(LShoulderPitch)]])
        b = b1.dot(b2).dot(z)

        a_left = np.cross(uparmLeft,forearmLeft)/np.linalg.norm(np.cross(uparmLeft,forearmLeft))

        # around +40 degree different
        LElbowYaw = (pi/2 - math.acos(b.dot(a_left)/(np.linalg.norm(b)*np.linalg.norm(a_left))))
        LElbowRoll = -math.acos(uparmLeft.dot(forearmLeft)/(np.linalg.norm(uparmLeft) * np.linalg.norm(forearmLeft)))
        # print math.degrees(LElbowYaw)
        # print math.degrees(LElbowRoll)

    ## Right arm
        # Bonds
        # 89
        uparmRight = right_elbow - right_shoulder 
        # 90
        forearmRight = right_wrist - right_elbow

        ## Shoulder angle
        RShoulderRoll = -(pi/2 - math.acos(np.cross(y,z).dot(uparmRight)/(np.linalg.norm(np.cross(y,z)) * np.linalg.norm(uparmRight))))
        # around 40 degree different
        RShoulderPitch = -math.atan(uparmRight[2]/uparmRight[0])
        # print RShoulderRoll
        # print math.degrees(RShoulderRoll)
        # print math.degrees(RShoulderPitch)

        ## Elbow angle
        b1=np.array([[math.cos(RShoulderRoll), -math.sin(RShoulderRoll), 0],
                    [math.sin(RShoulderRoll), math.cos(RShoulderRoll), 0],
                    [0, 0, 1]])
        b2=np.array([[math.cos(RShoulderPitch), 0, math.sin(RShoulderPitch)],
                    [0, 1, 0],
                    [-math.sin(RShoulderPitch), 0, math.cos(RShoulderPitch)]])
        b = b1.dot(b2).dot(z)

        a_right = np.cross(uparmRight,forearmRight)/np.linalg.norm(np.cross(uparmRight,forearmRight))

        # around +40 degree different
        RElbowYaw = (pi/2 - math.acos(b.dot(a_right)/(np.linalg.norm(b)*np.linalg.norm(a_right))))
        RElbowRoll = math.acos(uparmRight.dot(forearmRight)/(np.linalg.norm(uparmRight) * np.linalg.norm(forearmRight)))
        # print math.degrees(RElbowYaw)
        # print math.degrees(RElbowRoll)

    ## Head
        neckHead = head - neck
        HeadPitch = (pi/2 - math.acos(np.cross(x,y).dot(neckHead)/(np.linalg.norm(np.cross(x,y)) * np.linalg.norm(neckHead))))
        # print math.degrees HeadPitch)  

    ## Legs
        leg_Left = left_knee - left_hip
        leg_Right = right_knee - right_hip
        
        LHipPitch = -(pi/2 - math.acos(np.cross(x,y).dot(leg_Left)/(np.linalg.norm(np.cross(x,y)) * np.linalg.norm(leg_Left))))
        RHipPitch = -(pi/2 - math.acos(np.cross(x,y).dot(leg_Right)/(np.linalg.norm(np.cross(x,y)) * np.linalg.norm(leg_Right))))
        print "LPitch:", LHipPitch
        # print math.degrees(RHipPitch)  

    ## Filters
    # Left arm
        if LShoulderRoll > 1.3265:
            LShoulderRoll = 1.3265
        elif LShoulderRoll < -0.3142:
            LShoulderRoll = -0.3142
        # print "Filer:L",LShoulderRoll

        if LShoulderPitch > 2.0857:
            LShoulderPitch = 2.0857
        elif LShoulderPitch < -2.0857:
            LShoulderPitch = -2.0857

        if LElbowYaw > 2.0857:
            LElbowYaw = 2.0857
        elif LElbowYaw < -2.0857:
            LElbowYaw = -2.0857

        if LElbowRoll > -0.0349:
            LElbowRoll = -0.0349
        elif LElbowRoll <  -1.5446:
            LElbowRoll = -1.5446

    # Right arm
        if RShoulderRoll > 0.3142:
            RShoulderRoll = 0.3
        elif RShoulderRoll < -1.3265:
            RShoulderRoll = -1.3
        # print "Filer:R",RShoulderRoll

        if RShoulderPitch > 2.0857:
            RShoulderPitch = 2.0857
        elif RShoulderPitch < -2.0857:
            RShoulderPitch = -2.0857

        if RElbowYaw > 2.0857:
            RElbowYaw = 2.0857
        elif RElbowYaw < -2.0857:
            RElbowYaw = -2.0857

        if RElbowRoll > 1.5446:
            RElbowRoll = 1.5446
        elif RElbowRoll < 0.0349:
            RElbowRoll = 0.0349

    # Hip
        if LHipPitch > 0.484090:
            LHipPitch = 0.484090
        elif LHipPitch < -1.535889:
            LHipPitch = -1.535889

        if RHipPitch > 0.484090:
            RHipPitch = 0.484090
        elif RHipPitch < -1.535889:
            RHipPitch = -1.535889

    # Head
        if HeadPitch > 0.5149:
            HeadPitch = 0.5149
        elif HeadPitch < -0.6720:
            HeadPitch = -0.6720

    # Update Nao
        names  = ["HeadPitch","LShoulderPitch","LShoulderRoll","LElbowYaw","LElbowRoll","RShoulderRoll","RShoulderPitch","RElbowYaw","RElbowRoll"]
        angles  =[HeadPitch,LShoulderPitch,LShoulderRoll,LElbowYaw,LElbowRoll,RShoulderRoll,RShoulderPitch,RElbowYaw,RElbowRoll] #,LHipPitch,RHipPitch
        fractionMaxSpeed  = 0.6
        motionProxy.setAngles(names, angles, fractionMaxSpeed)

        # Crouch
        LegNames  = ["LHipPitch","LKneePitch","LAnklePitch","RHipPitch","RKneePitch","RAnklePitch"]
        timelist = [0.7 for x in range(6)]
        global crouchState
        if(LHipPitch > 0.0):
            if(crouchState != 1):
                crouchState = 1
                LegAngles  = [0.13, -0.09, 0.09, 0.13, -0.09, 0.09]
                motionProxy.angleInterpolation(LegNames, LegAngles, timelist, True)
        elif(LHipPitch > -0.15):
            if(crouchState != 2):
                crouchState = 2
                LegAngles  = [-0.15, 0.7, -0.352,-0.15, 0.7, -0.352]
                motionProxy.angleInterpolation(LegNames, LegAngles, timelist, True)
        elif(LHipPitch <= -0.25):
            if(crouchState != 3):
                crouchState = 3
                # LegAngles  = [-0.67, 1.65, -0.724,-0.67, 1.65, -0.724]
                LegAngles  = [-0.05, 1, -0.724,-0.05, 1, -0.724]
                motionProxy.angleInterpolation(LegNames, LegAngles, timelist, True)

        # if right_wrist.y > right_shoulder.y and not raised:
        #     raised = True
        #     print('Recognized right hand wave.')
        # elif right_wrist.y < spine_mid.y and raised:
        #     raised = False

# A bone is a cylinder connecting two joints, each specified by an id.
_bone_ids = [[0, 1], [1, 2], [2, 3], [7, 6], [6, 5], [5, 4], [4, 2],
             [2, 8], [8, 9], [9, 10], [10, 11], [15, 14], [14, 13], [13, 12],
             [12, 0], [0, 16], [16, 17], [17, 18], [18, 19]]

# Initialize and level the Kinect sensor.
_kinect = nui.Runtime()
_kinect.skeleton_engine.enabled = True
_kinect.camera.elevation_angle = 15

# Initialize crouch state
crouchState = 1

def naoInit(postureProxy, motionProxy):
    motionProxy.setStiffnesses("Body", 1.0)
    motionProxy.setStiffnesses("Head", 1.0)
    motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])
    postureProxy.goToPosture("Stand", 0.5)
    # postureProxy.goToPosture("StandInit", 0.5)

    # motionProxy.wbEnable(True)

    # # Example showing how to Constraint Balance Motion.
    # isEnable   = True
    # supportLeg = "Legs"
    # motionProxy.wbEnableBalanceConstraint(isEnable, supportLeg)

def naoRest(postureProxy, motionProxy):
    postureProxy.goToPosture("Crouch", 0.2)
    motionProxy.setStiffnesses("Body", 0.0)
    motionProxy.setStiffnesses("Head", 0.0)


if __name__ == '__main__':
    # Initialize nao proxy
    # robotIp = "127.0.0.1"
    # robotIp = "192.168.1.106"
    robotIp = "192.168.1.115"
    motionProxy = ALProxy("ALMotion", robotIp, 9559)
    postureProxy = ALProxy("ALRobotPosture", robotIp, 9559)

    draw_sensor(frame())
    skeleton = Skeleton(frame(visible=False))
    count = 0
    raised = False
    naoInit(postureProxy, motionProxy)

    # Loop
    while True:
        count+=1
        if count > 700:
            _kinect.close()
            naoRest(postureProxy, motionProxy)
            exit()
        # print count

        rate(30)
        skeleton.frame.visible = skeleton.update()
        analyse_frame(skeleton,raised,motionProxy)
        
