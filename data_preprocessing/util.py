import numpy as np

belongDict={
    'C7':'Torso',   #intentionally move from neck to torso
    'T7':'Torso',
    'STER':'Torso',
    'XPRO':'Torso',
    'T4':'Torso',
    'T8':'Torso',
    'T10':'Torso',
    'RPSIS':'Pelvis',
    'LPSIS':'Pelvis',
    'RASIS':'Pelvis',
    'LASIS':'Pelvis',
    'RTEMP':'Head',
    'LTEMP':'Head',
    'RHEAD':'Head',
    'LHEAD':'Head',
    'RACR':'ShoulderR',
    'LACR':'ShoulderL',
    'RHLE':'ArmR',
    'RHME':'ArmR',
    'LHLE':'ArmL',
    'LHME':'ArmL',
    'RUSP':'ForeArmR',
    'RRSP':'ForeArmR',
    'LUSP':'ForeArmL',
    'LRSP':'ForeArmL',
    'RCAP':'HandR',
    'RHMC2':'HandR',    #
    'LCAP':'HandL',
    'LHMC2':'HandL',    #
    'RFME'  :'FemurR',
    'RFLE'  :'FemurR',
    'LFME'  :'FemurL',
    'LFLE'  :'FemurL',
    'RFCC'  :'TalusR',
    'LFCC'  :'TalusL',
    'RTAM'  :'TibiaR',
    'RFAL'  :'TibiaR',
    'LTAM'  :'TibiaL',
    'LFAL'  :'TibiaL',
    'RFMT1' :'TalusR',
    'RFMT2' :'TalusR',
    'RFMT5' :'TalusR',
    'LFMT1' :'TalusL',
    'LFMT2' :'TalusL',
    'LFMT5' :'TalusL'
}

def floatList(s):
    l=s.strip().split(' ')
    ans=[]
    for a in l:
        ans.append(float(a))
    return ans

def getTransformationXML(e):   #input element that contain "Transformation"
    t=e.getElementsByTagName('Transformation')[0]
    linear=floatList(t.getAttribute('linear'))
    tran=floatList(t.getAttribute('translation'))
    
    r=np.array(linear).reshape([3,3])
    t=np.array(tran).reshape([3,1])
    #print(r)
    #print(t)
    return np.vstack([np.hstack([r,t]),np.array([[0,0,0,1]])])

def transformVertexObj(srcObj,dstObj,transformMat):
    fr=open(srcObj,'r')
    fw=open(dstObj,'w')

    line=fr.readline()
    while(line):
        if(line.startswith("v ")):
            v=[]
            for a in line.strip().split(' ')[1:4]:
                v.append(float(a)*0.01) #scale from cm to meter
            v.append(1)
            v = (transformMat @ np.array(v))[:3]
            v*=100  #scale from meter to cm
            fw.write('v %f %f %f\n'%(v[0],v[1],v[2]))
        else:
            fw.write(line)  #line already contain newline

        line=fr.readline()

    fr.close()
    fw.close()