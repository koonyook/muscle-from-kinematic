import numpy as np
import pickle
import trimesh

dataFolder='template/'
objPath=dataFolder+'OBJ/'

################ marker set ####################
markerDict={}
f=open(objPath+'markerVertex.txt','r')
markerCount=0
for line in f.readlines():
    markerName,objName,vertexIndex=line.strip().split(' ')
    objName=objName+'.obj'
    vertexIndex=int(vertexIndex)
    if objName not in markerDict:
        markerDict[objName]=[]

    markerDict[objName].append((vertexIndex,markerName,))
    markerCount+=1

f.close()

markerPosition=[] #np.zeros([markerCount,3])   #global frame

finalDict={}    #map from marker name to a desired position

for obj in markerDict:
    v=trimesh.load(objPath+obj, process=False).vertices #use this library because it preserve the order of vertex 
    v*=0.01
    for tup in markerDict[obj]:
        vertexIndex,markerName=tup  #markerDict[obj][i]
        vertexPosition=v[vertexIndex,:]

        finalDict[markerName]=vertexPosition
        
        print(obj,markerName,vertexIndex,vertexPosition)

    if obj=='R_Hand.obj' or obj=='L_Hand.obj':
        finalDict[obj[0]+'HMC2']=(v[2710,:]+v[2494,:])/2    #for LHMC2 and

for markerName in finalDict:
    if(markerName in ['STER','XPRO','T4','T8','T10','C7','T7']):
        finalDict[markerName][0]=0     #because XPRO is not centered

    if(markerName.startswith('L')): #copy from R
        targetName='R'+markerName[1:]
        finalDict[markerName]=np.copy(finalDict[targetName])
        finalDict[markerName][0]*=-1

for markerName in finalDict:
    print(markerName,finalDict[markerName])

pickle.dump(finalDict,open(dataFolder+"markerPosition.pkl",'wb'))