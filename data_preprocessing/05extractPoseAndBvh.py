import numpy as np
from xml.dom.minidom import parse
import pickle
import sys

from util import *
from myMath import *
import refM
from TSVreader import MarkerRecord
from bvhTemplate import saveToBvh
#read template and scale everything like 04displayScaledModel.py
#do not take from results of 05generatePersonalXmlObj.py

mocapFile='data/SNFAT_walking.tsv'
firstFrameToExtract=534
lastFrameToExtract=1250

#mocapFile='data/SNFAT_turning.tsv'
#firstFrameToExtract=1
#lastFrameToExtract=968

personalDict=pickle.load(open('output\personalDict.pkl','rb'))
scaleMatDict=personalDict['scaleMatDict']
headH_headM_personal=personalDict['headH_headM_personal']
T7orT8=personalDict['T7orT8']

dataFolder='template/'

markerDict=pickle.load(open(dataFolder+"markerPosition.pkl",'rb'))    
for marker in markerDict:
	segment=belongDict[marker]
	markerDict[marker]=(scaleMatDict[segment] @ homo(markerDict[marker]))[:3]

##########################################################

jointCenterDict={}
parentDict={}
jointList=[]
human = parse(dataFolder+'human.xml')
for aNode in human.getElementsByTagName('Node'):
	segment=aNode.getAttribute('name')
	parent=aNode.getAttribute('parent')

	parentDict[segment]=parent
	jointList.append(segment)
	#body=aNode.getElementsByTagName('Body')[0]
	joint=aNode.getElementsByTagName('Joint')[0]
	
	jointT=getTransformationXML(joint)
	jointCenterDict[segment]=(scaleMatDict[segment] @jointT[:,3])[:3]


###########################################################

#calculate relations on the template that will not change
worldH_pelvisH=translationHomo(jointCenterDict['Pelvis'])
worldH_pelvisM=refM.getPelvisM(markerDict)
pelvisM_pelvisH = inverseHomo(worldH_pelvisM) @ worldH_pelvisH

worldH_torsoH=translationHomo(jointCenterDict['Torso'])
worldH_torsoM=refM.getThoraxM(markerDict,T7orT8)
torsoM_torsoH = inverseHomo(worldH_torsoM) @ worldH_torsoH

#worldH_headH=translationHomo(jointCenterDict['Head'])
#worldH_headM=refM.getHeadM(markerDict)
#headM_headH = inverseHomo(worldH_headM) @ worldH_headH

femurLH_femurLM = refM.getFemurM(np.zeros(3),markerDict['LFME']-jointCenterDict['FemurL'],markerDict['LFLE']-jointCenterDict['FemurL'])
femurRH_femurRM = refM.getFemurM(np.zeros(3),markerDict['RFME']-jointCenterDict['FemurR'],markerDict['RFLE']-jointCenterDict['FemurR'])

tibiaLH_tibiaLM = refM.getTibiaM(np.zeros(3),markerDict['LFAL']-jointCenterDict['TibiaL'],markerDict['LTAM']-jointCenterDict['TibiaL'])
tibiaRH_tibiaRM = refM.getTibiaM(np.zeros(3),markerDict['RFAL']-jointCenterDict['TibiaR'],markerDict['RTAM']-jointCenterDict['TibiaR'])

armLH_armLM = refM.getArmM(np.zeros(3),markerDict['LHME']-jointCenterDict['ArmL'],markerDict['LHLE']-jointCenterDict['ArmL'])
armRH_armRM = refM.getArmM(np.zeros(3),markerDict['RHME']-jointCenterDict['ArmR'],markerDict['RHLE']-jointCenterDict['ArmR'])

foreArmLH_foreArmLM = refM.getForeArmM(np.zeros(3),markerDict['LRSP']-jointCenterDict['ForeArmL'],markerDict['LUSP']-jointCenterDict['ForeArmL'])
foreArmRH_foreArmRM = refM.getForeArmM(np.zeros(3),markerDict['RRSP']-jointCenterDict['ForeArmR'],markerDict['RUSP']-jointCenterDict['ForeArmR'])

#some need only orientation relationship
talusLH_talusLM_orientation = refM.getTalusM_orientation(markerDict['LFCC'],markerDict['LFMT1'],markerDict['LFMT5'])
talusRH_talusRM_orientation = refM.getTalusM_orientation(markerDict['RFCC'],markerDict['RFMT1'],markerDict['RFMT5'])

##################################


###################################
#now, everything is scaled. jointCenterDict, markerDict
#time to load tsv

# H is frame from human.xml (already scaled) 
# M is frame from marker (already scaled)

sta=MarkerRecord(mocapFile,rotateToBvhFrame=True) #static record
allAns=[]
successFrame=[]
skipCount=0

for i,d in zip(sta.frameNo,sta.record):
	
	if(i<firstFrameToExtract or i>lastFrameToExtract):
		continue

	print('frameNo:',i)

	#d is a dictionary that map from marker name to position

	#detect markerSet
	if allKeyExist(d,[		#some marker is not used and can be removed
		#pelvis
		'LPSIS','RPSIS','LASIS','RASIS',	'LICR','RICR',
		#thorax
		'STER','XPRO','C7','T4',T7orT8,'T10',
		#head
		'RTEMP','RHEAD','LHEAD','LTEMP',
		#arm
		'RACR','LACR',
		'RUA1','RUA2','RUA3','RUA4',
		'LUA1','LUA2','LUA3','LUA4',
		'RHLE','RHME',
		'LHLE','LHME',
		
		'LFA1','LFA2','LFA3','LRSP','LUSP',
		'RFA1','RFA2','RFA3','RRSP','RUSP',
	]):
		if allKeyExist(d,[		#some marker is not used and can be removed
			#leg & foot
			'LTH1','LTH2','LTH3','LTH4','LFLE','LFME',
			'LSK1','LSK2','LSK3','LSK4','LTAM','LFAL',
			'LFCC','LFMT1','LFMT2','LFMT5',
			'RTH1','RTH2','RTH3','RTH4','RFLE','RFME',
			'RSK1','RSK2','RSK3','RSK4','RTAM','RFAL',
			'RFCC','RFMT1','RFMT2','RFMT5'
		]):
			markerSet='full'
		else:
			markerSet='invalid' #'upper'
			successFrame.append(None)
			skipCount+=1
			continue
	else:
		markerSet='invalid'
		#poseSequence.append(None)
		successFrame.append(None)
		skipCount+=1
		continue

	############# lower body #################
	#pelvis
	worldA_pelvisM=refM.getPelvisM(d)   #worldA is from the actual mocap reference frame
	worldA_pelvisH=worldA_pelvisM @ pelvisM_pelvisH     #ans at the root

	#femur
	pelvisH_hipL=jointCenterDict['FemurL']-jointCenterDict['Pelvis']
	pelvisH_hipR=jointCenterDict['FemurR']-jointCenterDict['Pelvis']

	worldA_hipL=worldA_pelvisH @ homo(pelvisH_hipL)
	worldA_hipR=worldA_pelvisH @ homo(pelvisH_hipR)

	worldA_femurLM = refM.getFemurM(worldA_hipL,d['LFME'],d['LFLE'])
	worldA_femurRM = refM.getFemurM(worldA_hipR,d['RFME'],d['RFLE'])

	worldA_femurLH = worldA_femurLM @ inverseHomo(femurLH_femurLM)
	worldA_femurRH = worldA_femurRM @ inverseHomo(femurRH_femurRM)
	#print(worldA_femurLH.shape)
	pelvisH_femurLH = inverseHomo(worldA_pelvisH) @ worldA_femurLH  #ans
	pelvisH_femurRH = inverseHomo(worldA_pelvisH) @ worldA_femurRH  #ans

	#tibia
	femurLH_kneeL=jointCenterDict['TibiaL']-jointCenterDict['FemurL']
	femurRH_kneeR=jointCenterDict['TibiaR']-jointCenterDict['FemurR']

	worldA_kneeL=worldA_femurLH @ homo(femurLH_kneeL)
	worldA_kneeR=worldA_femurRH @ homo(femurRH_kneeR)

	worldA_tibiaLM = refM.getTibiaM(worldA_kneeL,d['LFAL'],d['LTAM'])
	worldA_tibiaRM = refM.getTibiaM(worldA_kneeR,d['RFAL'],d['RTAM'])

	worldA_tibiaLH = worldA_tibiaLM @ inverseHomo(tibiaLH_tibiaLM)
	worldA_tibiaRH = worldA_tibiaRM @ inverseHomo(tibiaRH_tibiaRM)

	femurLH_tibiaLH = inverseHomo(worldA_femurLH) @ worldA_tibiaLH  #ans
	femurRH_tibiaRH = inverseHomo(worldA_femurRH) @ worldA_tibiaRH  #ans

	#talus (preserve segment orientation, so relative orientation is enough to produce answer)
	worldA_talusLM_orientation = refM.getTalusM_orientation(d['LFCC'],d['LFMT1'],d['LFMT5'])
	worldA_talusRM_orientation = refM.getTalusM_orientation(d['RFCC'],d['RFMT1'],d['RFMT5'])

	tibiaLH_talusLH_orientation = np.transpose(worldA_tibiaLH[:3,:3]) @ worldA_talusLM_orientation @ np.transpose(talusLH_talusLM_orientation) #ans
	tibiaRH_talusRH_orientation = np.transpose(worldA_tibiaRH[:3,:3]) @ worldA_talusRM_orientation @ np.transpose(talusRH_talusRM_orientation) #ans

	############# upper body #################
	#start from torso upward (it will be connected to lower body later)
	worldA_torsoM=refM.getThoraxM(d,T7orT8)

	worldA_torsoH=worldA_torsoM @ torsoM_torsoH

	torsoH_shoulderLPoint=(jointCenterDict['ShoulderL']-jointCenterDict['Torso'])
	torsoH_shoulderRPoint=(jointCenterDict['ShoulderR']-jointCenterDict['Torso'])

	worldA_shoulderLPoint= worldA_torsoH @ homo(torsoH_shoulderLPoint)
	worldA_shoulderRPoint= worldA_torsoH @ homo(torsoH_shoulderRPoint)

	#shoulder segment
	#left side
	#torsoH_shoulderLFrameH = translationHomo(jointCenterDict['ShoulderL']-jointCenterDict['Torso'])
	#worldA_shoulderLFrameH = worldA_torsoH @ torsoH_shoulderLFrameH #H means before rotate
	#shoulderLFrameH_LACRH = markerDict['LACR']-jointCenterDict['ShoulderL'] 
	#worldA_LACRH=worldA_shoulderLFrameH @ homo(shoulderLFrameH_LACRH)          #before rotate
	#worldA_LACR=d['LACR']   #after rotate
	#oldFrame_newFrame=getSmallestRotMatFromThreePoints(worldA_shoulderLPoint,worldA_LACRH,worldA_LACR)
	#worldA_shoulderLFrame = worldA_shoulderLFrameH @ oldFrame_newFrame  #the shoulderFrame is rotated 
	#torsoH_shoulderLFrame = inverseHomo(worldA_torsoH) @ worldA_shoulderLFrame  #ans

	#right side
	#torsoH_shoulderRFrameH = translationHomo(jointCenterDict['ShoulderR']-jointCenterDict['Torso'])
	#worldA_shoulderRFrameH = worldA_torsoH @ torsoH_shoulderRFrameH #H means before rotate
	#shoulderRFrameH_RACRH = markerDict['RACR']-jointCenterDict['ShoulderR'] 
	#worldA_RACRH=worldA_shoulderRFrameH @ homo(shoulderRFrameH_RACRH)          #before rotate
	#worldA_RACR=d['RACR']   #after rotate
	#oldFrame_newFrame=getSmallestRotMatFromThreePoints(worldA_shoulderRPoint,worldA_RACRH,worldA_RACR)
	#worldA_shoulderRFrame = worldA_shoulderRFrameH @ oldFrame_newFrame  #the shoulderFrame is rotated 
	#torsoH_shoulderRFrame = inverseHomo(worldA_torsoH) @ worldA_shoulderRFrame  #ans

	#shoulder segment
	#this version is more compact and less confused by stepping back one joint (rotation happen in torsoH frame is easier)
	#left side
	torsoH_LACRH = markerDict['LACR']-jointCenterDict['Torso'] #point before rotate
	torsoH_LACR  = inverseHomo(worldA_torsoH) @ homo(d['LACR']) #point after rotate #worldA_LACR=d['LACR']
	torsoH_shoulderLFrame_orientation=getSmallestRotMatFromThreePoints(torsoH_shoulderLPoint,torsoH_LACRH,torsoH_LACR)  #ans #torsoH_shoulderLPoint as center of rotation
	torsoH_shoulderLFrame=combineRotMatAndTranslation(torsoH_shoulderLFrame_orientation,jointCenterDict['ShoulderL']-jointCenterDict['Torso'])
	worldA_shoulderLFrame=worldA_torsoH @ torsoH_shoulderLFrame

	#right side
	torsoH_RACRH = markerDict['RACR']-jointCenterDict['Torso'] #point before rotate
	torsoH_RACR  = inverseHomo(worldA_torsoH) @ homo(d['RACR']) #point after rotate #worldA_RACR=d['RACR']
	torsoH_shoulderRFrame_orientation=getSmallestRotMatFromThreePoints(torsoH_shoulderRPoint,torsoH_RACRH,torsoH_RACR)  #ans #torsoH_shoulderRPoint as center of rotation
	torsoH_shoulderRFrame=combineRotMatAndTranslation(torsoH_shoulderRFrame_orientation,jointCenterDict['ShoulderR']-jointCenterDict['Torso'])
	worldA_shoulderRFrame=worldA_torsoH @ torsoH_shoulderRFrame

	#arm segment  
	shoulderLFrame_ArmLPoint = jointCenterDict['ArmL']-jointCenterDict['ShoulderL']    #OK
	shoulderRFrame_ArmRPoint = jointCenterDict['ArmR']-jointCenterDict['ShoulderR']    #OK

	worldA_ArmLPoint=worldA_shoulderLFrame @ homo(shoulderLFrame_ArmLPoint)
	worldA_ArmRPoint=worldA_shoulderRFrame @ homo(shoulderRFrame_ArmRPoint)

	worldA_armLM = refM.getArmM(worldA_ArmLPoint,d['LHME'],d['LHLE'])
	worldA_armRM = refM.getArmM(worldA_ArmRPoint,d['RHME'],d['RHLE'])

	worldA_armLH = worldA_armLM @ inverseHomo(armLH_armLM)
	worldA_armRH = worldA_armRM @ inverseHomo(armRH_armRM)

	shoulderLFrame_armLH = inverseHomo(worldA_shoulderLFrame) @ worldA_armLH #ans
	shoulderRFrame_armRH = inverseHomo(worldA_shoulderRFrame) @ worldA_armRH #ans

	#forearm segment
	armLH_elbowL=jointCenterDict['ForeArmL']-jointCenterDict['ArmL']
	armRH_elbowR=jointCenterDict['ForeArmR']-jointCenterDict['ArmR']

	worldA_elbowL= worldA_armLH @ homo(armLH_elbowL)
	worldA_elbowR= worldA_armRH @ homo(armRH_elbowR)

	worldA_foreArmLM = refM.getForeArmM(worldA_elbowL,d['LRSP'],d['LUSP'])
	worldA_foreArmRM = refM.getForeArmM(worldA_elbowR,d['RRSP'],d['RUSP'])

	worldA_foreArmLH = worldA_foreArmLM @ inverseHomo(foreArmLH_foreArmLM)
	worldA_foreArmRH = worldA_foreArmRM @ inverseHomo(foreArmRH_foreArmRM)

	armLH_foreArmLH = inverseHomo(worldA_armLH) @ worldA_foreArmLH #ans
	armRH_foreArmRH = inverseHomo(worldA_armRH) @ worldA_foreArmRH #ans

	#hand (conditioned on marker availability)
	if('LHMC2' in d):
		#try to match direction from wrist to HMC2 (similar to shoulder with ACR)
		#left hand
		forearmLH_handLPoint=(jointCenterDict['HandL']-jointCenterDict['ForeArmL'])
		forearmLH_LHMC2H = markerDict['LHMC2']-jointCenterDict['ForeArmL'] #point before rotate
		forearmLH_LHMC2  = inverseHomo(worldA_foreArmLH) @ homo(d['LHMC2']) #point after rotate #worldA_LHMC2=d['LHMC2']
		forearmLH_handLH_orientation=getSmallestRotMatFromThreePoints(forearmLH_handLPoint,forearmLH_LHMC2H,forearmLH_LHMC2)  #ans #forearmLH_handLPoint as center of rotation
		
		#right hand
		forearmRH_handRPoint=(jointCenterDict['HandR']-jointCenterDict['ForeArmR'])
		forearmRH_RHMC2H = markerDict['RHMC2']-jointCenterDict['ForeArmR'] #point before rotate
		forearmRH_RHMC2  = inverseHomo(worldA_foreArmRH) @ homo(d['RHMC2']) #point after rotate #worldA_RHMC2=d['RHMC2']
		forearmRH_handRH_orientation=getSmallestRotMatFromThreePoints(forearmRH_handRPoint,forearmRH_RHMC2H,forearmRH_RHMC2)  #ans #forearmLH_handLPoint as center of rotation

	else:
		forearmLH_handLH_orientation=np.eye(3)   #ans
		forearmRH_handRH_orientation=np.eye(3)   #ans

	#neck (use pointing strategy and bending similar to shoulder)
	worldA_headM = refM.getHeadM(d)
	worldA_headH = worldA_headM @ inverseHomo(headH_headM_personal) #headM_headH
	torsoH_headPointH = jointCenterDict['Head']-jointCenterDict['Torso']    #point before rotate
	torsoH_headPoint  = inverseHomo(worldA_torsoH) @ worldA_headH[:,3]      #point after rotate (observed)
	torsoH_neckPointH = jointCenterDict['Neck']-jointCenterDict['Torso']    #center of rotation
	torsoH_neck_orientation=getSmallestRotMatFromThreePoints(torsoH_neckPointH,torsoH_headPointH,torsoH_headPoint)  #ans
	torsoH_neck=combineRotMatAndTranslation(torsoH_neck_orientation,jointCenterDict['Neck']-jointCenterDict['Torso'])   #might not be used
	worldA_neck=worldA_torsoH @ torsoH_neck                                                                             #might not be used
	
	#head (preserve orientation in worldA frame)
	neck_headH_orientation = np.transpose(worldA_neck[:3,:3]) @ worldA_headH[:3,:3]     #ans   

	#spine connection can choose to keep upperbody constant or lower body constant
	focusLowerBody=True
	if(focusLowerBody):
		#twist sharing between 2 joints are complicated, so do the same thing as neck
		pelvisH_spinePoint  = jointCenterDict['Spine']-jointCenterDict['Pelvis']
		pelvisH_torsoPointH = jointCenterDict['Torso']-jointCenterDict['Pelvis']    #point before rotate
		pelvisH_torsoPoint  = inverseHomo(worldA_pelvisH) @ worldA_torsoH[:,3]
		pelvisH_spine_orientation=getSmallestRotMatFromThreePoints(pelvisH_spinePoint,pelvisH_torsoPointH,pelvisH_torsoPoint)   #ans
		pelvisH_spine=combineRotMatAndTranslation(pelvisH_spine_orientation,pelvisH_spinePoint)
		worldA_spine=worldA_pelvisH @ pelvisH_spine

		#torso
		spine_torsoH_orientation = np.transpose(worldA_spine[:3,:3]) @ worldA_torsoH[:3,:3] #ans
		#spine_torsoH = combineRotMatAndTranslation(spine_torsoH_orientation,jointCenterDict['Torso']-jointCenterDict['Spine']) #might not be used

		#check if this method preserve the torso orientation
		#worldA_torsoH_afterMod = worldA_pelvisH[:3,:3] @ pelvisH_spine_orientation[:3,:3] @ spine_torsoH_orientation
		#print('before & after mod')
		#print(worldA_torsoH[:3,:3])
		#print('-----')
		#print(worldA_torsoH_afterMod)	#exactly the same
		#exit()
	else:
		pass    

	#all the answers are calculated in form of rotation matrix, I must convert them to Euler angle used in bhv (ZXY)
	ans=[]
	ans+=list(worldA_pelvisH[:3,3]*100)    #root translation (XYZ) in the unit of cm
	ans+=eulerZXY(worldA_pelvisH[:3,:3])    #in unit of rad
	ans+=eulerZXY(pelvisH_spine[:3,:3])
	ans+=eulerZXY(spine_torsoH_orientation)

	ans+=eulerZXY(torsoH_shoulderLFrame[:3,:3])
	ans+=eulerZXY(shoulderLFrame_armLH[:3,:3])
	ans+=eulerZXY(armLH_foreArmLH[:3,:3])
	ans+=eulerZXY(forearmLH_handLH_orientation[:3,:3])
	ans+=[0,0,0]
	ans+=[0,0,0]
	ans+=[0,0,0]
	ans+=eulerZXY(torsoH_shoulderRFrame[:3,:3])
	ans+=eulerZXY(shoulderRFrame_armRH[:3,:3])
	ans+=eulerZXY(armRH_foreArmRH[:3,:3])
	ans+=eulerZXY(forearmRH_handRH_orientation[:3,:3])
	ans+=[0,0,0]
	ans+=[0,0,0]
	ans+=[0,0,0]
	ans+=eulerZXY(torsoH_neck[:3,:3])
	ans+=eulerZXY(neck_headH_orientation)
	ans+=[0,0,0]
	ans+=[0,0,0]
	ans+=[0,0,0]
	ans+=eulerZXY(pelvisH_femurRH[:3,:3])
	ans+=eulerZXY(femurRH_tibiaRH[:3,:3])
	ans+=eulerZXY(tibiaRH_talusRH_orientation)
	ans+=[0,0,0]
	ans+=[0,0,0]
	ans+=[0,0,0]
	ans+=[0,0,0]
	ans+=eulerZXY(pelvisH_femurLH[:3,:3])
	ans+=eulerZXY(femurLH_tibiaLH[:3,:3])
	ans+=eulerZXY(tibiaLH_talusLH_orientation)
	ans+=[0,0,0]
	ans+=[0,0,0]
	ans+=[0,0,0]
	ans+=[0,0,0]
	ans+=[0,0,0]

	allAns.append(ans)

	successFrame.append({
		'Pelvis':worldA_pelvisH,
		'Spine':pelvisH_spine[:3,:3],
		'Torso':spine_torsoH_orientation,
		
		'ShoulderL':torsoH_shoulderLFrame[:3,:3],
		'ArmL':shoulderLFrame_armLH[:3,:3],
		'ForeArmL':armLH_foreArmLH[:3,:3],
		'HandL':forearmLH_handLH_orientation,

		'ShoulderR':torsoH_shoulderRFrame[:3,:3],
		'ArmR':shoulderRFrame_armRH[:3,:3],
		'ForeArmR':armRH_foreArmRH[:3,:3],
		'HandR':forearmRH_handRH_orientation,

		'Neck':torsoH_neck[:3,:3],
		'Head':neck_headH_orientation,
		
		'FemurR':pelvisH_femurRH[:3,:3],
		'TibiaR':femurRH_tibiaRH[:3,:3],
		'TalusR':tibiaRH_talusRH_orientation,
		
		'FemurL':pelvisH_femurLH[:3,:3],
		'TibiaL':femurLH_tibiaLH[:3,:3],
		'TalusL':tibiaLH_talusLH_orientation
	})

outputFileName=mocapFile.split('/')[-1].replace('.tsv','.bvh')
saveToBvh("output/"+outputFileName,jointCenterDict,sta.frequency,allAns)
print('done export to test.bvh')
print('skip count:',skipCount)

#################################################
#################################################
#visualize the result to see if the result match the marker
exit()

#things to display
#1. joint center position + linkage
#2. actual marker position
#3. virtual marker position
from vispy import scene
from vispy.scene import SceneCanvas
from vispy.visuals import transforms

from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import *
from PyQt5.QtWidgets import QMainWindow, QWidget, QLabel, QGridLayout, QPushButton, QCheckBox, QSlider

from myVispyObject import VispyMan

appQt = QtWidgets.QApplication(sys.argv)

win = QMainWindow()
win.resize(700, 500)
win.setWindowTitle('Component Tester')

canvas=SceneCanvas()
canvas.create_native()
canvas.native.setParent(win)
canvas.unfreeze()
view=canvas.central_widget.add_view()   #add view to the canvas
view.camera='turntable'
view.camera.up='+y'
canvas.freeze()
#=================================

originAxis=scene.visuals.XYZAxis(parent=view.scene)    #axis length=1
man=VispyMan(view.scene,jointList,jointCenterDict,markerDict,parentDict)

#=================================
rightPanel=QWidget()

gbox = QtWidgets.QGridLayout()


testButton=QPushButton()
testButton.setText("test")
gbox.addWidget(testButton,0,1)

timer=QTimer()

firstFrameIndex=firstFrameToExtract-sta.frameNo[0]

currentFrameIndex=0
def test():
	global currentFrameIndex
	currentFrameIndex=(currentFrameIndex+1)%len(successFrame)
	
	while(successFrame[currentFrameIndex] is None):
		man.update(None,sta.record[currentFrameIndex])
		currentFrameIndex=(currentFrameIndex+1)%len(successFrame)
	man.update(successFrame[currentFrameIndex],sta.record[currentFrameIndex+firstFrameIndex])

	#while(display[currentFrameIndex] is None or phaseRecord[currentFrameIndex]<0):
	#	w.update(None,None)
	#	currentFrameIndex=(currentFrameIndex+1)%len(display)
	#	
	#w.update(display[currentFrameIndex],phaseRecord[currentFrameIndex])
	
	timer.start(15)

	#sk.set_data(connect=np.array([[0,1],[0,2],[0,3]],dtype=int))
	#while True:
	#	for aFrame,phase in zip(display,phaseRecord):
	#		#vp.rate(1/dt)
	#		w.update(aFrame,phase)

timer.timeout.connect(test)

def toggle():
	if(timer.isActive()):
		timer.stop()
	else:
		timer.start(15)

testButton.clicked.connect(toggle)


rightPanel.setLayout(gbox)

splitter=QtWidgets.QSplitter(QtCore.Qt.Horizontal)
splitter.addWidget(canvas.native)      #add canvas to splitter
splitter.addWidget(rightPanel)

win.setCentralWidget(splitter)      #add splitter to main window
#========================

#========================
win.show()
appQt.exec_()