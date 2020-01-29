#with the same input as 04displayScaledModel.py

#output files to generate
#1.human.xml
#2.muscle.xml
#3.all the bone.obj

#for xml, find library that can modify and save
#for obj, find library that can modify and save (without changing order)

#have to also think about mass distribution too

import numpy as np
from xml.dom.minidom import parse
import pickle

import random
import os,sys
import shutil
from bvh import Bvh
import trimesh  #

from util import *
from myMath import *

srcDataFolder='template/'
srcObjPath=srcDataFolder+'OBJ/'

dstDataFolder='output/'    #for XML
dstObjPath=dstDataFolder+'OBJ/'    #for OBJ

os.makedirs(dstDataFolder,exist_ok=True)
os.makedirs(dstObjPath,exist_ok=True)

shutil.copy(srcObjPath+'material.mtl', dstObjPath)

#scaleMatDict=pickle.load(open('personalAdjustmentMatrix.pkl','rb'))
personalDict=pickle.load(open('output/personalDict.pkl','rb'))
scaleMatDict=personalDict['scaleMatDict']
headH_headM_personal=personalDict['headH_headM_personal']

targetMass=60 #FAT=SN052=71.2    #MJ=60

############## human.xml + obj ###############
templateMass=61.4   #confirmed   #kg
massMultiplier=targetMass/templateMass

human = parse(srcDataFolder+"human.xml")
nodes=human.getElementsByTagName('Node')
#sm=0
for aNode in nodes:
    name=aNode.getAttribute('name')
    parent=aNode.getAttribute('parent')

    body=aNode.getElementsByTagName('Body')[0]
    joint=aNode.getElementsByTagName('Joint')[0]

    #sm=sm+float(body.getAttribute('mass'))

    mass=massMultiplier*float(body.getAttribute('mass'))
    body.setAttribute('mass',str(mass))

    size=floatList(body.getAttribute('size'))   #*
    size*=np.diag(scaleMatDict[name])[:3]
    body.setAttribute('size','%f %f %f'%tuple(size))

    contact=body.getAttribute('contact')    #Off or On
    
    #bodyT=getTransformationXML(body)       #*
    #jointT=getTransformationXML(joint)     #*
    bodyTransformation = body.getElementsByTagName('Transformation')[0]
    bodyTranslation=np.array(floatList(bodyTransformation.getAttribute('translation')))
    bodyTranslation = (scaleMatDict[name] @ homo(bodyTranslation))[:3]
    bodyTransformation.setAttribute('translation','%f %f %f'%tuple(bodyTranslation))
    
    jointTransformation = joint.getElementsByTagName('Transformation')[0]
    jointTranslation=np.array(floatList(jointTransformation.getAttribute('translation')))
    jointTranslation=(scaleMatDict[name] @ homo(jointTranslation))[:3]
    jointTransformation.setAttribute('translation','%f %f %f'%tuple(jointTranslation))

    obj=body.getAttribute('obj') 
    transformVertexObj(srcObjPath+obj,dstObjPath+obj,scaleMatDict[name])
#print(sm)
with open(dstDataFolder+"human.xml", "w") as f:
    human.writexml(f)

################ muscle284.xml ######################

muscleXML=parse(srcDataFolder+"muscle284.xml")
muscles = muscleXML.getElementsByTagName('Unit')

for e in muscles:
    name=e.getAttribute('name')
    #f0=e.getAttribute('f0')
    #lm=e.getAttribute('lm')
    #lt=e.getAttribute('lt')
    #pen_angle=e.getAttribute('pen_angle')
    #lmax=e.getAttribute('lmax')

    for i,w in enumerate(e.getElementsByTagName('Waypoint')):
        belongTo=w.getAttribute('body')
        p=np.array(floatList(w.getAttribute('p')))  #*
        #print(belongTo,p)
        pMod=(scaleMatDict[belongTo] @ homo(p))[:3]
        w.setAttribute('p','%f %f %f'%tuple(pMod))

with open(dstDataFolder+"muscle284.xml", "w") as f:
    muscleXML.writexml(f)

print('done')