import sys

from vispy import scene
from vispy.scene import SceneCanvas
from vispy.visuals import transforms

from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import *
from PyQt5.QtWidgets import QMainWindow, QWidget, QLabel, QGridLayout, QPushButton, QCheckBox, QSlider

#from MyWidget import *

from vispy import io
#from vispy.visuals.filters import Alpha
from vispy.visuals.transforms import STTransform, MatrixTransform, ChainTransform

import numpy as np
from xml.dom.minidom import parse
import pickle

import random
from bvh import Bvh
import trimesh  #

from util import *
from myMath import *
#scaleMatDict=pickle.load(open('personalAdjustmentMatrix.pkl','rb'))
personalDict=pickle.load(open('output/personalDict.pkl','rb'))
scaleMatDict=personalDict['scaleMatDict']
headH_headM_personal=personalDict['headH_headM_personal']

######################################################
dataFolder='template/'
objPath=dataFolder+'OBJ/'
#bvhPath=dataFolder+'motion/'

######################################################
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

#transparent surface must come last (order is important)
originAxis=scene.visuals.XYZAxis(parent=view.scene)    #axis length=1
print(originAxis.pos)

################ marker set ####################
#markerDict={}
#f=open(objPath+'markerVertex.txt','r')
#markerCount=0
#for line in f.readlines():
#    markerName,objName,vertexIndex=line.strip().split(' ')
#    objName=objName+'.obj'
#    vertexIndex=int(vertexIndex)
#    if objName not in markerDict:
#        markerDict[objName]=[]
#
#    markerDict[objName].append((vertexIndex,markerName,))
#    markerCount+=1
#
#f.close()
#print(markerDict.keys())



######################################################
human = parse(dataFolder+"human.xml")
nodes=human.getElementsByTagName('Node')
humanJointPositionList=[]
boxCenterList=[]
for aNode in nodes:
    name=aNode.getAttribute('name')
    parent=aNode.getAttribute('parent')
    body=aNode.getElementsByTagName('Body')[0]
    joint=aNode.getElementsByTagName('Joint')[0]
    #print(name,parent,body,joint)   #ok

    mass=body.getAttribute('mass')
    size=floatList(body.getAttribute('size'))   #*
    size*=np.diag(scaleMatDict[name])[:3]
    #print(size)
    contact=body.getAttribute('contact')    #Off or On
    obj=body.getAttribute('obj') 
    
    bodyT=getTransformationXML(body)       #*
    jointT=getTransformationXML(joint)     #*

    #boxCenterList.append(bodyT[0:3,3])
    #humanJointPositionList.append(jointT[0:3,3])
    modBoxCenter=(scaleMatDict[name]@bodyT[:,3:4])[0:3,0]
    bodyT[:3,3]=modBoxCenter
    #bodyT rotation should be adjusted a little bit too
    #but keeping orientation the same should not be that bad
    boxCenterList.append(modBoxCenter)
    humanJointPositionList.append((scaleMatDict[name]@jointT[:,3:4])[0:3,0])


    #print(bodyT)    #ok
    #print(jointT)   #ok

    verts, faces, normals, nothin = io.read_mesh(objPath+obj)   #verts preserve the number of vertex but the order is not preserved
    verts*=0.01

    vertexMod = scaleMatDict[name] @ np.vstack([verts.transpose(),np.ones([1,verts.shape[0]])]) #(4,n)
    verts = vertexMod[:3,:].transpose()

    objMesh = scene.visuals.Mesh(parent=view.scene,vertices=verts, shading='flat', faces=faces,color=(0.8, 0.8, 0.8,0.2)) #'flat' is much faster than 'smooth', None removes lighting
    objMesh.set_gl_state('translucent', cull_face=False,depth_test=False)
    
    aBox=scene.visuals.Box(parent=view.scene,width=size[0],depth=size[1],height=size[2],color=(0.8,0.1,0.1,0.2))
    aBox.transform=MatrixTransform(bodyT.transpose())   #transpose to match openGL format
    aBox.set_gl_state('translucent', cull_face=False,depth_test=False)



    #if(obj in markerDict):
    #    v=trimesh.load(objPath+obj, process=False).vertices #use this library because it preserve the order of vertex 
    #    v*=0.01
    #    for tup in markerDict[obj]:
    #        vertexIndex,markerName=tup  #markerDict[obj][i]
    #        markerPosition.append(v[vertexIndex,:])
    #        print(obj,markerName,vertexIndex,v[vertexIndex,:])
    #else:
    #    print(obj)

humanJointPositionList=np.stack(humanJointPositionList)
humanJointMarker=scene.visuals.Markers(parent=view.scene,size=8,pos=humanJointPositionList,face_color='green')
humanJointMarker.set_gl_state('translucent', cull_face=False,depth_test=False)

boxCenterList=np.stack(boxCenterList)
boxCenterMarker=scene.visuals.Markers(parent=view.scene,size=8,pos=boxCenterList,face_color='red')
boxCenterMarker.set_gl_state('translucent', cull_face=False,depth_test=False)

###################### marker position ######################
'''
markerPosition=[] #np.zeros([markerCount,3])   #global frame

finalMarkerDict=pickle.load(open(dataFolder+"markerPosition.pkl",'rb'))     #processed from 01 genTemplateMarkerPosition
for markerName in finalMarkerDict:
    markerPosition.append(finalMarkerDict[markerName])

markerPosition=np.stack(markerPosition)
markerMarker=scene.visuals.Markers(parent=view.scene,size=8,pos=markerPosition,face_color='orange')
markerMarker.set_gl_state('translucent', cull_face=False,depth_test=False)
'''
###################################################################
muscles = parse(dataFolder+"muscle284.xml").getElementsByTagName('Unit')
wp=[]
muscleColor=[]
indexPair=[]
random.seed(a=0)
for e in muscles:
    name=e.getAttribute('name')
    f0=e.getAttribute('f0')
    lm=e.getAttribute('lm')
    lt=e.getAttribute('lt')
    pen_angle=e.getAttribute('pen_angle')
    lmax=e.getAttribute('lmax')
    #print(name)
   
    #Random a bright color
    while True:
        cr=random.random()
        cg=random.random()
        cb=random.random()
        if(max([cr,cg,cb])>0.3):
            break

    for i,w in enumerate(e.getElementsByTagName('Waypoint')):
        belongTo=w.getAttribute('body')
        p=np.array(floatList(w.getAttribute('p')))  #*
        #print(belongTo,p)
        pMod=scaleMatDict[belongTo] @ homo(p)
        wp.append(pMod[:3])
        muscleColor.append([cr,cg,cb,1.0])
        if(i>0):
            indexPair.append([len(wp)-2,len(wp)-1])

    #aLine=scene.visuals.Line(parent=view.scene,pos=wp,width=1,connect='strip',color='yellow')  # method='agg'
    #aLine.set_gl_state('translucent', cull_face=False,depth_test=True)
wp=np.stack(wp)
muscleColor=np.stack(muscleColor)
allLine=scene.visuals.Line(parent=view.scene,pos=wp,color=muscleColor,width=1,connect=np.array(indexPair))
allLine.set_gl_state('translucent', cull_face=False,depth_test=True)

'''
################# BVH ####################################
bvhMultiplier=0.01
with open(bvhPath+'SNFAT_walking.bvh') as f:
    mocap = Bvh(f.read())

rootName='Character1_Hips'
rootOffset=np.array([float(e) for e in next(mocap.root.filter('ROOT'))['OFFSET']])*bvhMultiplier
rootOffset=humanJointPositionList[0,:] #np.zeros(3)  #hack
#turn everything I need to dict of dict
s={
    '':{
        'parent':'',
        'relOffset':np.zeros(3), #relative offset from parent joint
        'absOffset':np.zeros(3)  #absolute offset in rest pose (T-pose)
    },
    rootName:{
        'parent':'',
        'relOffset':rootOffset,
        'absOffset':rootOffset,
}}

bvhJointList=mocap.get_joints_names()
print(bvhJointList)
for p in bvhJointList:
    s[p]['children']=[str(e).split(' ',1)[1] for e in mocap.joint_direct_children(p)]
    for c in s[p]['children']:
        relativeOffset=np.array(mocap.joint_offset(c))*bvhMultiplier
        s[c]={
            'parent':p,
            'relOffset':relativeOffset,
            'absOffset':s[p]['absOffset']+relativeOffset,
        }

bvhJointPosition=np.zeros([len(bvhJointList),3])
for i,p in enumerate(bvhJointList):
    bvhJointPosition[i,:]=s[p]['absOffset']
    #print(p,s[p]['absOffset'])
    if('RIGMESH' in p):
        bvhJointPosition[i,:]=0

#print(bvhJointPosition)

bvhMarker=scene.visuals.Markers(parent=view.scene,size=8,pos=bvhJointPosition,face_color='blue')
bvhMarker.set_gl_state('translucent', cull_face=False,depth_test=False)
'''



#=================================
rightPanel=QWidget()

gbox = QtWidgets.QGridLayout()



testButton=QPushButton()
testButton.setText("test")
gbox.addWidget(testButton,0,1)

def test():
    #sk.set_data(connect=np.array([[0,1],[0,2],[0,3]],dtype=int))
    print('click')
testButton.clicked.connect(test)


rightPanel.setLayout(gbox)

splitter=QtWidgets.QSplitter(QtCore.Qt.Horizontal)
splitter.addWidget(canvas.native)      #add canvas to splitter
splitter.addWidget(rightPanel)

win.setCentralWidget(splitter)      #add splitter to main window
#========================

#========================
win.show()
appQt.exec_()