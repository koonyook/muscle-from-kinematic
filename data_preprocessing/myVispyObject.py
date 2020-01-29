import numpy as np
from vispy import scene
from vispy.visuals import transforms
from util import *
from myMath import *

class VispyMan():

    def __init__(self,parentScene,jointList:list,jointCenterDict:dict,markerDict:dict,parentDict:dict):
        self.scene=parentScene

        self.jointList=jointList
        self.jointCenterDict=jointCenterDict
        self.markerDict=markerDict
        self.p=parentDict

        self.n=len(jointList)

        #add object placeholder
        self.jMarker=scene.visuals.Markers(parent=self.scene,size=3,pos=np.zeros([len(jointCenterDict),3]),face_color='red')   #joint center
        self.aMarker=scene.visuals.Markers(parent=self.scene,size=3,pos=np.zeros([len(markerDict),3]),face_color='green')
        self.vMarker=scene.visuals.Markers(parent=self.scene,size=3,pos=np.zeros([len(markerDict),3]),face_color='blue')

    def update(self,pose,frame): #pose can be none or dict of jointName:rotMat, frame map from marker to position (mocap)
        am=[]
        for e in frame:
            am.append(frame[e])
        self.aMarker.set_data(pos=np.stack(am),face_color=(1,0,0,0.3))
        
        if(pose is None):
            return
        
        #update object with pose
        #belongDict
        parent=self.p
        markerDict=self.markerDict
        jointCenterDict=self.jointCenterDict

        worldA_frame={'Pelvis':pose['Pelvis']}
        for j in self.jointList[1:]:
            if j in pose:
                currentJointPose=pose[j]
            else:
                #print('pose not exist:',j)  #only FootThumb and FootPinky
                currentJointPose=np.eye(3)
            
            worldA_frame[j]=worldA_frame[parent[j]] @ combineRotMatAndTranslation(currentJointPose,jointCenterDict[j]-jointCenterDict[parent[j]])

        jm=[]   #joint center
        for j in worldA_frame:
            jm.append(worldA_frame[j][:3,3])
        self.jMarker.set_data(pos=np.stack(jm),face_color='green')

        vm=[]   #virtual marker
        for m in markerDict:
            worldA_m = (worldA_frame[belongDict[m]] @ homo(markerDict[m]-jointCenterDict[belongDict[m]]))[:3]
            vm.append(worldA_m)
            #if m=='LHEAD':
            #    print(m)
            #    print(belongDict[m])
            #    print(worldA_frame[belongDict[m]])  #this looks wrong
            #    print(worldA_m) #this point is lower than the head joint, this should not be possible
            #    print(markerDict[m]-jointCenterDict[belongDict[m]]) #looks correct
            #    exit()
        self.vMarker.set_data(pos=np.stack(vm),face_color='blue')
