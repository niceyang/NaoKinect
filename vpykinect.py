from visual import *
import pykinect
from pykinect import nui
from pykinect.nui import JointId
import math

class Vector(object):
    """Vector functions for medium value for angle calc between joints"""
    def __init__(self,x,y,z):
        super(Vector, self).__init__()
        self.x =x
        self.y =y
        self.z =z


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
        self.joints[8].radius = 0.125
        self.joints[9].radius = 0.125
        self.joints[10  ].radius = 0.125
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

def line_angle(x1,y1,x2,y2):
    up = math.fabs(x1*y1+x2*y2)/1.0
    down = math.sqrt(x1*x1+y1*y1)*math.sqrt(x2*x2+y2*y2)
    value = up/down
    plus = 0
    minus = 0
    while value>1:
        value = value-1
        plus+=1 
    while value<-1:
        value = value+1
        minus+=1
    result = math.degrees(math.acos(value))
    if plus>0:
        return result+90*plus
    if minus>0:
        return (result+90*minus)*-1
    return result
    # return up/down

def calcVector(jointSart,jointEnd):
    vector = Vector(jointEnd.x - jointSart.x,jointEnd.y - jointSart.y,jointEnd.z - jointSart.z)
    return vector


def analyse_frame(skeleton,raised):
    if skeleton.frame.visible:
        
        # Skeleton info
        head = skeleton.joints[3]
        neck = skeleton.joints[2]
        spine_shoulder = skeleton.joints[2]
        spine_mid = skeleton.joints[1]

        right_shoulder = skeleton.joints[8]
        right_elbow = skeleton.joints[9]
        right_wrist = skeleton.joints[10]
        left_shoulder = skeleton.joints[4]
        left_elbow = skeleton.joints[5]
        left_wrist = skeleton.joints[6]

        left_hip = skeleton.joints[12]
        right_hip = skeleton.joints[16]
        left_knee = skeleton.joints[13]
        right_knee = skeleton.joints[17]

        # Vector Info
        spine_upV = calcVector(spine_mid,spine_shoulder)
        spine_downV = calcVector(spine_shoulder,spine_mid)
        headV = calcVector(spine_shoulder,head)

        elbow_leftV = calcVector(left_shoulder,left_elbow)
        wrist_leftV = calcVector(left_elbow,left_wrist)
        leg_leftV = calcVector(left_hip,left_knee)

        elbow_rightV = calcVector(right_shoulder,right_elbow)
        wrist_rightV = calcVector(right_elbow,right_wrist)
        leg_rightV = calcVector(right_hip,right_knee)


        # Angle Info
        # HeadYew = line_angle(head.x,head.y,neck.x,neck.y)
        HeadPitch = line_angle(spine_upV.y,spine_upV.z,headV.y,headV.z)

        LshoulderRoll = line_angle(spine_downV.y,spine_downV.z,elbow_leftV.y,elbow_leftV.z)
        LshoulderYaw = line_angle(spine_downV.x,spine_downV.y,elbow_leftV.x,elbow_leftV.y)

        RshoulderRoll = line_angle(spine_downV.y,spine_downV.z,elbow_rightV.y,elbow_rightV.z)
        RshoulderYaw = line_angle(spine_downV.x,spine_downV.y,elbow_rightV.x,elbow_rightV.y)

        LelbowRoll = line_angle(wrist_leftV.y,wrist_leftV.z,elbow_leftV.y,elbow_leftV.z)
        LelbowYaw = line_angle(wrist_leftV.x,wrist_leftV.y,elbow_leftV.x,elbow_leftV.y)
    
        RelbowRoll = line_angle(wrist_rightV.y,wrist_rightV.z,elbow_rightV.y,elbow_rightV.z)
        RelbowYaw = line_angle(wrist_rightV.x,wrist_rightV.y,elbow_rightV.x,elbow_rightV.y)

        print '---start---'
        # print 'HeadPitch:%s' % HeadPitch
        # print 'LshoulderRoll:%s' % LshoulderRoll
        # print 'LshoulderYaw:%s' % LshoulderYaw
        # print 'RshoulderRoll:%s' % RshoulderRoll
        # print 'RshoulderYaw:%s' % RshoulderYaw
        print 'LelbowRoll:%s' % LelbowRoll
        print 'LelbowYaw:%s' % LelbowYaw
        # print 'RelbowRoll:%s' % RelbowRoll
        # print 'RelbowYaw:%s' % RelbowYaw
        print '---end---'

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
        if count > 30000:
            _kinect.close()
            exit()
        # print count

        rate(30)
        skeleton.frame.visible = skeleton.update()
        analyse_frame(skeleton,raised)
        
