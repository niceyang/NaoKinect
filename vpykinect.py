from visual import *
import pykinect
from pykinect import nui
from pykinect.nui import JointId
import math
import numpy as np


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


def analyse_frame(skeleton,raised):
    if skeleton.frame.visible:
        
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
        shoulderRoll_Left = -(pi/2 - math.acos(np.cross(y,z).dot(uparmLeft)/(np.linalg.norm(np.cross(y,z)) * np.linalg.norm(uparmLeft))))
        # around 40 degree different
        shoulderPich_Left = math.atan(uparmLeft[2]/uparmLeft[0])
        # shoulderPich_Left = shoulderPich_Left - math.radians(40)
        # print math.degrees(shoulderRoll_Left)
        # print math.degrees(shoulderPich_Left)

        ## Elbow angle
        b1=np.array([[math.cos(shoulderRoll_Left), -math.sin(shoulderRoll_Left), 0],
                    [math.sin(shoulderRoll_Left), math.cos(shoulderRoll_Left), 0],
                    [0, 0, 1]])
        b2=np.array([[math.cos(shoulderPich_Left), 0, math.sin(shoulderPich_Left)],
                    [0, 1, 0],
                    [-math.sin(shoulderPich_Left), 0, math.cos(shoulderPich_Left)]])
        b = b1.dot(b2).dot(z)

        a_left = np.cross(uparmLeft,forearmLeft)/np.linalg.norm(np.cross(uparmLeft,forearmLeft))

        # around +40 degree different
        elbowYaw_Left = (pi/2 - math.acos(b.dot(a_left)/(np.linalg.norm(b)*np.linalg.norm(a_left))))
        elbowRoll_Left = -math.acos(uparmLeft.dot(forearmLeft)/(np.linalg.norm(uparmLeft) * np.linalg.norm(forearmLeft)))
        # print math.degrees(elbowYaw_Left)
        # print math.degrees(elbowRoll_Left)

    ## Right arm
        # Bonds
        # 89
        uparmRight = right_elbow - right_shoulder 
        # 90
        forearmRight = right_wrist - right_elbow

        ## Shoulder angle
        shoulderRoll_Right = -(pi/2 - math.acos(np.cross(y,z).dot(uparmRight)/(np.linalg.norm(np.cross(y,z)) * np.linalg.norm(uparmRight))))
        # around 40 degree different
        shoulderPich_Right = -math.atan(uparmRight[2]/uparmRight[0])
        # print math.degrees(shoulderRoll_Right)
        # print math.degrees(shoulderPich_Right)

        ## Elbow angle
        b1=np.array([[math.cos(shoulderRoll_Right), -math.sin(shoulderRoll_Right), 0],
                    [math.sin(shoulderRoll_Right), math.cos(shoulderRoll_Right), 0],
                    [0, 0, 1]])
        b2=np.array([[math.cos(shoulderPich_Right), 0, math.sin(shoulderPich_Right)],
                    [0, 1, 0],
                    [-math.sin(shoulderPich_Right), 0, math.cos(shoulderPich_Right)]])
        b = b1.dot(b2).dot(z)

        a_right = np.cross(uparmRight,forearmRight)/np.linalg.norm(np.cross(uparmRight,forearmRight))

        # around +40 degree different
        elbowYaw_Right = (pi/2 - math.acos(b.dot(a_right)/(np.linalg.norm(b)*np.linalg.norm(a_right))))
        elbowRoll_Right = math.acos(uparmRight.dot(forearmRight)/(np.linalg.norm(uparmRight) * np.linalg.norm(forearmRight)))
        # print math.degrees(elbowYaw_Right)
        # print math.degrees(elbowRoll_Right)

    ## Head
        neckHead = head - neck
        neckAngle = (pi/2 - math.acos(np.cross(x,y).dot(neckHead)/(np.linalg.norm(np.cross(x,y)) * np.linalg.norm(neckHead))))
        # print math.degrees(neckAngle)  

    ## Legs
        leg_Left = left_knee - left_hip
        leg_Right = right_knee - right_hip
        
        legRoll_Left = -(pi/2 - math.acos(np.cross(x,y).dot(leg_Left)/(np.linalg.norm(np.cross(x,y)) * np.linalg.norm(leg_Left))))
        legRoll_Right = -(pi/2 - math.acos(np.cross(x,y).dot(leg_Right)/(np.linalg.norm(np.cross(x,y)) * np.linalg.norm(leg_Right))))
        # print math.degrees(legRoll_Left)  
        # print math.degrees(legRoll_Right)  



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

if __name__ == '__main__':
    draw_sensor(frame())
    skeleton = Skeleton(frame(visible=False))
    count = 0
    raised = False
    while True:
        count+=1
        if count > 500:
            _kinect.close()
            exit()
        # print count

        rate(30)
        skeleton.frame.visible = skeleton.update()
        analyse_frame(skeleton,raised)
        
