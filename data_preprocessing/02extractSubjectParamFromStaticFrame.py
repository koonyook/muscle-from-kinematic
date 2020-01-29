#extract scaling parameters from a static frames
#inputs
#a frame of markers from mocap 
# #base bone landmark position in template from markerPosition.pkl
#base joint position in template from human.xml

#output
#all scaling parameters 

import numpy as np
from xml.dom.minidom import parse
import pickle
from util import *
from myMath import *
import refM

dataFolder='template/'

####
markerDict=pickle.load(open(dataFolder+"markerPosition.pkl",'rb'))    

####
jointCenterDict={}
parentDict={}
jointList=[]
human = parse(dataFolder+'human.xml')
for aNode in human.getElementsByTagName('Node'):
    name=aNode.getAttribute('name')
    parent=aNode.getAttribute('parent')

    parentDict[name]=parent
    jointList.append(name)
    #body=aNode.getElementsByTagName('Body')[0]
    joint=aNode.getElementsByTagName('Joint')[0]
    
    jointT=getTransformationXML(joint)
    jointCenterDict[name]=jointT[0:3,3]

####
from TSVreader import MarkerRecord
folder='data/'
staticFile='SNFAT_static.tsv'   #T-pose in frame 1
staticT=1   #frame number from staticFile that will be used

sta=MarkerRecord(folder+staticFile) #static record
d=sta.record[staticT]   #map from marker name to position

##############################################

#calculate 3 relations on the template that will not change
worldH_pelvisH=translationHomo(jointCenterDict['Pelvis'])
worldH_torsoH=translationHomo(jointCenterDict['Torso'])
#worldH_headH=translationHomo(jointCenterDict['Head'])

worldH_pelvisM=refM.getPelvisM(markerDict)

#must check first if static use T7 or T8, and I must put this in Subject Param
if('T7' in d):
    T7orT8='T7'
else:
    T7orT8='T8'

worldH_torsoM=refM.getThoraxM(markerDict,T7orT8)
#worldH_headM=refM.getHeadM(markerDict)

pelvisM_pelvisH = inverseHomo(worldH_pelvisM) @ worldH_pelvisH
torsoM_torsoH = inverseHomo(worldH_torsoM) @ worldH_torsoH
#headM_headH = inverseHomo(worldH_headM) @ worldH_headH

scale={}    #map from joint name to 3D scaling (the answer from this script)

####################### lower body ###################################
#start from pelvis downward

#Pelvis: scale on X-axis
actual_ASIS_distance = length(d['LASIS']-d['RASIS'])
template_ASIS_distance = length(markerDict['LASIS']-markerDict['RASIS'])
scale['Pelvis'] = np.array([actual_ASIS_distance/template_ASIS_distance,1,1])    #scale on x axis

pelvisH_hipL_scaled=(jointCenterDict['FemurL']-jointCenterDict['Pelvis'])*scale['Pelvis']
pelvisH_hipR_scaled=(jointCenterDict['FemurR']-jointCenterDict['Pelvis'])*scale['Pelvis']

#put scaled pelvis to the world frame to find actual hip location
worldA_pelvisM=refM.getPelvisM(d)   #worldA is from the actual mocap reference frame
worldA_hipL=worldA_pelvisM @ pelvisM_pelvisH @ homo(pelvisH_hipL_scaled)
worldA_hipR=worldA_pelvisM @ pelvisM_pelvisH @ homo(pelvisH_hipR_scaled)

#Femur
template_hipL_to_kneeMarker_distance = length(jointCenterDict['FemurL']-(markerDict['LFME']+markerDict['LFLE'])/2)
actual_hipL_to_kneeMarker_distance = length(worldA_hipL[:3]-(d['LFME']+d['LFLE'])/2)
scale['FemurL']=np.array([1,actual_hipL_to_kneeMarker_distance/template_hipL_to_kneeMarker_distance,1]) #scale on y axis

template_hipR_to_kneeMarker_distance = length(jointCenterDict['FemurR']-(markerDict['RFME']+markerDict['RFLE'])/2)
actual_hipR_to_kneeMarker_distance = length(worldA_hipR[:3]-(d['RFME']+d['RFLE'])/2)
scale['FemurR']=np.array([1,actual_hipR_to_kneeMarker_distance/template_hipR_to_kneeMarker_distance,1]) #scale on y axis

#Where is knee location in worldA
worldA_femurLM = refM.getFemurM(worldA_hipL,d['LFME'],d['LFLE'])
worldA_femurRM = refM.getFemurM(worldA_hipR,d['RFME'],d['RFLE'])

femurLH_femurLM = refM.getFemurM(np.zeros(3),(markerDict['LFME']-jointCenterDict['FemurL'])*scale['FemurL'],(markerDict['LFLE']-jointCenterDict['FemurL'])*scale['FemurL'])
femurRH_femurRM = refM.getFemurM(np.zeros(3),(markerDict['RFME']-jointCenterDict['FemurR'])*scale['FemurR'],(markerDict['RFLE']-jointCenterDict['FemurR'])*scale['FemurR'])

femurLH_kneeL_scaled=(jointCenterDict['TibiaL']-jointCenterDict['FemurL'])*scale['FemurL']
femurRH_kneeR_scaled=(jointCenterDict['TibiaR']-jointCenterDict['FemurR'])*scale['FemurR']

worldA_kneeL=worldA_femurLM @ inverseHomo(femurLH_femurLM) @ homo(femurLH_kneeL_scaled)
worldA_kneeR=worldA_femurRM @ inverseHomo(femurRH_femurRM) @ homo(femurRH_kneeR_scaled)

#Tibia
template_kneeL_to_ankleMarker_distance=length(jointCenterDict['TibiaL']-(markerDict['LFAL']+markerDict['LTAM'])/2)
actual_kneeL_to_ankleMarker_distance=length(worldA_kneeL[:3]-(d['LFAL']+d['LTAM'])/2)
scale['TibiaL']=np.array([1,actual_kneeL_to_ankleMarker_distance/template_kneeL_to_ankleMarker_distance,1]) #scale on y axis

template_kneeR_to_ankleMarker_distance=length(jointCenterDict['TibiaR']-(markerDict['RFAL']+markerDict['RTAM'])/2)
actual_kneeR_to_ankleMarker_distance=length(worldA_kneeR[:3]-(d['RFAL']+d['RTAM'])/2)
scale['TibiaR']=np.array([1,actual_kneeR_to_ankleMarker_distance/template_kneeR_to_ankleMarker_distance,1]) #scale on y axis

################### upper body ######################
#start from torso upward

#put torso to the world frame to find actual clavical joint location
worldA_torsoM=refM.getThoraxM(d,T7orT8)   #worldA is from the actual mocap reference frame

torsoH_shoulderLPoint=(jointCenterDict['ShoulderL']-jointCenterDict['Torso'])
torsoH_shoulderRPoint=(jointCenterDict['ShoulderR']-jointCenterDict['Torso'])

worldA_shoulderLPoint=worldA_torsoM @ torsoM_torsoH @ homo(torsoH_shoulderLPoint)
worldA_shoulderRPoint=worldA_torsoM @ torsoM_torsoH @ homo(torsoH_shoulderRPoint)

template_clavicleL_distance=length(jointCenterDict['ShoulderL']-markerDict['LACR'])
actual_clavicleL_distance=length(worldA_shoulderLPoint[:3]-d['LACR'])
scale['ShoulderL']=np.array([actual_clavicleL_distance/template_clavicleL_distance,1,1])

template_clavicleR_distance=length(jointCenterDict['ShoulderR']-markerDict['RACR'])
actual_clavicleR_distance=length(worldA_shoulderRPoint[:3]-d['RACR'])
scale['ShoulderR']=np.array([actual_clavicleR_distance/template_clavicleR_distance,1,1])

#where is ArmLPoint (shoulder) in worldA? this is complicated but hope it works 
#left side
torsoH_shoulderLFrameH = translationHomo(jointCenterDict['ShoulderL']-jointCenterDict['Torso'])
worldA_shoulderLFrameH = worldA_torsoM @ torsoM_torsoH @ torsoH_shoulderLFrameH #H means before rotate
shoulderLFrameH_LACRH = (markerDict['LACR']-jointCenterDict['ShoulderL'])*scale['ShoulderL'] 
worldA_LACRH=worldA_shoulderLFrameH @ homo(shoulderLFrameH_LACRH)          #before rotate
worldA_LACR=d['LACR']   #after rotate

worldA_directionBefore = normalize(worldA_LACRH - worldA_shoulderLPoint)
worldA_directionAfter  = normalize(worldA_LACR  - worldA_shoulderLPoint[:3])
oldFrame_newFrame=rotationHomo(getSmallestRotMatBetweenTwoDirections(worldA_directionBefore,worldA_directionAfter))  #tested
worldA_shoulderLFrame = worldA_shoulderLFrameH @ oldFrame_newFrame  #the shoulderFrame is rotated 

shoulderLFrame_ArmLPoint_scale = (jointCenterDict['ArmL']-jointCenterDict['ShoulderL']) * scale['ShoulderL']    #OK
worldA_ArmLPoint=worldA_shoulderLFrame @ homo(shoulderLFrame_ArmLPoint_scale)

#right side
torsoH_shoulderRFrameH = translationHomo(jointCenterDict['ShoulderR']-jointCenterDict['Torso'])
worldA_shoulderRFrameH = worldA_torsoM @ torsoM_torsoH @ torsoH_shoulderRFrameH #H means before rotate
shoulderRFrameH_RACRH = (markerDict['RACR']-jointCenterDict['ShoulderR'])*scale['ShoulderR'] 
worldA_RACRH=worldA_shoulderRFrameH @ homo(shoulderRFrameH_RACRH)          #before rotate
worldA_RACR=d['RACR']   #after rotate

worldA_directionBefore = normalize(worldA_RACRH - worldA_shoulderRPoint)
worldA_directionAfter  = normalize(worldA_RACR -  worldA_shoulderRPoint[:3])
oldFrame_newFrame=rotationHomo(getSmallestRotMatBetweenTwoDirections(worldA_directionBefore,worldA_directionAfter))  #tested
worldA_shoulderRFrame = worldA_shoulderRFrameH @ oldFrame_newFrame  #the shoulderFrame is rotated 

shoulderRFrame_ArmRPoint_scale = (jointCenterDict['ArmR']-jointCenterDict['ShoulderR']) * scale['ShoulderR']    #OK
worldA_ArmRPoint=worldA_shoulderRFrame @ homo(shoulderRFrame_ArmRPoint_scale)

#ArmL scale
#left side
template_armL_distance=length(jointCenterDict['ArmL']-(markerDict['LHLE']+markerDict['LHME'])/2)
actual_armL_distance=length(worldA_ArmLPoint[:3]-(d['LHLE']+d['LHME'])/2)
scale['ArmL']=np.array([actual_armL_distance/template_armL_distance,1,1])
#right side
template_armR_distance=length(jointCenterDict['ArmR']-(markerDict['RHLE']+markerDict['RHME'])/2)
actual_armR_distance=length(worldA_ArmRPoint[:3]-(d['RHLE']+d['RHME'])/2)
scale['ArmR']=np.array([actual_armR_distance/template_armR_distance,1,1])


#Where is ForearmLPoint (elbow)?
#left side
worldA_armLM = refM.getArmM(worldA_ArmLPoint,d['LHME'],d['LHLE'])
armLH_armLM = refM.getArmM(np.zeros(3),(markerDict['LHME']-jointCenterDict['ArmL'])*scale['ArmL'],(markerDict['LHLE']-jointCenterDict['ArmL'])*scale['ArmL'])
armLH_elbowL_scaled=(jointCenterDict['ForeArmL']-jointCenterDict['ArmL'])*scale['ArmL']
worldA_elbowL=worldA_armLM @ inverseHomo(armLH_armLM) @ homo(armLH_elbowL_scaled)
#right side
worldA_armRM = refM.getArmM(worldA_ArmRPoint,d['RHME'],d['RHLE'])
armRH_armRM = refM.getArmM(np.zeros(3),(markerDict['RHME']-jointCenterDict['ArmR'])*scale['ArmR'],(markerDict['RHLE']-jointCenterDict['ArmR'])*scale['ArmR'])
armRH_elbowR_scaled=(jointCenterDict['ForeArmR']-jointCenterDict['ArmR'])*scale['ArmR']
worldA_elbowR=worldA_armRM @ inverseHomo(armRH_armRM) @ homo(armRH_elbowR_scaled)

#Forearm scale
#left side
template_forearmL_distance=length(jointCenterDict['ForeArmL']-(markerDict['LRSP']+markerDict['LUSP'])/2)
actual_forearmL_distance=length(worldA_elbowL[:3]-(d['LRSP']+d['LUSP'])/2)
scale['ForeArmL']=np.array([actual_forearmL_distance/template_forearmL_distance,1,1])
#right side
template_forearmR_distance=length(jointCenterDict['ForeArmR']-(markerDict['RRSP']+markerDict['RUSP'])/2)
actual_forearmR_distance=length(worldA_elbowR[:3]-(d['RRSP']+d['RUSP'])/2)
scale['ForeArmR']=np.array([actual_forearmR_distance/template_forearmR_distance,1,1])

################## neck ###############
#torsoH_neckJointH=(jointCenterDict['Neck']-jointCenterDict['Torso'])
#worldA_neckJointH=worldA_torsoM @ torsoM_torsoH @ homo(torsoH_neckJointH)

#worldA_headM=refM.getHeadM(d)
#worldA_headH=worldA_headM @ headM_headH
#headH_headJointH = homo(np.zeros(3))
#worldA_headJointH=worldA_headH @ headH_headJointH
#
#actual_neck_distance=length(worldA_headJointH-worldA_neckJointH)
#template_neck_distance=length(jointCenterDict['Head']-jointCenterDict['Neck'])
#scale['Neck']=np.array([1,actual_neck_distance/template_neck_distance,1])

#neck v2    #trust static frame that subject face forward (not up or down)
#cannot trust RHEAD and LHEAD marker because it is pretty random
#this cause large random rotation in headM frame
torsoH_neckJointH=(jointCenterDict['Neck']-jointCenterDict['Torso'])
worldA_neckJointH=worldA_torsoM @ torsoM_torsoH @ homo(torsoH_neckJointH)

worldA_headM=refM.getHeadM(d)   #z-axis will point slightly upward
zAxis=worldA_headM[:3,2]
zAxisMod=normalize(zAxis*np.array([1,1,0]))
angle=getAngleBetweenTwoDirection(zAxis,zAxisMod)
if(zAxis[2]<0): #unlikely because HEAD is higher than TMP in static frame
    angle*=-1
headH_headM_orientation_personal=rotVecToRotMat(np.array([-angle,0,0]))

headH_headM_personal = combineRotMatAndTranslation(
    headH_headM_orientation_personal, #rotation that make 
    (markerDict['LTEMP']+markerDict['RTEMP'])/2 - jointCenterDict['Head']   #offset
)   #special, must be kept

worldA_headH=worldA_headM @ inverseHomo(headH_headM_personal)

actual_neck_distance=length(worldA_headH[:3,3]-worldA_neckJointH[:3])
template_neck_distance=length(jointCenterDict['Head']-jointCenterDict['Neck'])
scale['Neck']=np.array([1,actual_neck_distance/template_neck_distance,1])



################ spline ################
pelvisH_splineJoint=homo(jointCenterDict['Spine']-jointCenterDict['Pelvis'])
worldA_splineJoint = worldA_pelvisM @ pelvisM_pelvisH @ homo(pelvisH_splineJoint)
worldA_torsoH = worldA_torsoM @ torsoM_torsoH

actual_spine_distance = length(worldA_torsoH[:3,3]-worldA_splineJoint[:3])
template_spine_distance = length(jointCenterDict['Torso']-jointCenterDict['Spine'])
scale['Spine']=np.array([1,actual_spine_distance/template_spine_distance,1])

#########################################
#add piece with no scaling (for future automation)
noScaleList=[
    'TalusR','FootThumbR','FootPinkyR',
    'TalusL','FootThumbL','FootPinkyL',
    'Torso','Head',
    'HandR',
    'HandL'
]
for key in noScaleList:
    scale[key]=np.ones(3)

for e in scale:
    print(e,scale[e])

pickle.dump(scale,open('personalAdjustment.pkl','wb'))

#########################
### this section calculate scaling&shifting matrix for each segment
# afterScale_beforeScale operator in worldH frame
# the key formula is newP = (hP - hC)*scale + newC
# rearrange to newP = scale*hP + (newC-scale*hC) when hP is input point
# hP is old position in human.xml, muscle.xml, obj files
# hC is jointCenterDict[joint], this is constant according to human.xml

# newP will generate newC for the next joint
scaleMatDict={}
newC={}
newC['Pelvis']=np.zeros(3) #jointCenterDict['Pelvis']  //keep the root at origin for easier code
#print(jointList)
for joint in jointList:
    if(joint not in newC):
        newC[joint]=(scaleMatDict[parentDict[joint]] @ homo(jointCenterDict[joint]))[:3]

    s=np.diag(scale[joint])
    scaleMatDict[joint]=combineRotMatAndTranslation(s,newC[joint]-(s@jointCenterDict[joint]))

    #print(joint)
    #print(scaleMatDict[joint])
#print(newC)

#shift whole thing vertically to make foot on the ground
'''
targetHeight=jointCenterDict['TalusL'][1]
if(newC['TalusL'][1]<newC['TalusR'][1]):
    yShift=targetHeight-newC['TalusL'][1]
else:
    yShift=targetHeight-newC['TalusR'][1]

for joint in jointList:
    scaleMatDict[joint][1,3]+=yShift
'''

personalDict={
    'scaleMatDict':scaleMatDict,#this matrix apply to all 3D position but not the box orientation
    'headH_headM_personal':headH_headM_personal,
    'T7orT8':T7orT8,
}

pickle.dump(personalDict,open('output/personalDict.pkl','wb'))

#pickle.dump(scaleMatDict,open('personalAdjustmentMatrix.pkl','wb'))

print('done')