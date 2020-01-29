import numpy as np
from myMath import *

#def getHeadM(d:dict):    
#    try:
#        origin=(d['LTEMP']+d['RTEMP']+d['LHEAD']+d['RHEAD'])/4
#        xAxis=normalize(d['LTEMP']-d['RTEMP'])
#        zTmp=normalize((d['LHEAD']+d['RHEAD'])/2 - (d['LTEMP']+d['RTEMP'])/2)
#        yAxis=crossAxis(zTmp,xAxis)
#        zAxis=crossAxis(xAxis,yAxis)
#        return getTransformation(xAxis,yAxis,zAxis,origin) 
#    except:
#        return None

def getHeadM(d:dict):    
    try:
        origin=(d['LTEMP']+d['RTEMP'])/2
        xAxis=normalize(d['LTEMP']-d['RTEMP'])
        zTmp=normalize((d['LHEAD']+d['RHEAD'])/2 - (d['LTEMP']+d['RTEMP'])/2)
        yAxis=crossAxis(zTmp,xAxis)
        zAxis=crossAxis(xAxis,yAxis)
        return getTransformation(xAxis,yAxis,zAxis,origin) 
    except:
        return None

def getThoraxM(d:dict,T7orT8):
    try:
        origin=(d['STER']+d['C7'])/2
        tmp=(d['XPRO']+d[T7orT8])/2
        yAxis=normalize(origin-tmp)
        zTmp=normalize(d['STER']-d['T4'])
        xAxis=crossAxis(yAxis,zTmp)
        zAxis=crossAxis(xAxis,yAxis)
        return getTransformation(xAxis,yAxis,zAxis,origin) 
    except:
        for marker in ['STER','C7','XPRO',T7orT8,'T4']:
            if marker not in d:
                print(marker,'not found.')

        return None

def getPelvisM(d:dict):
    #follow coda pelvis
    try:
        origin=(d['LASIS']+d['RASIS'])/2
        xAxis=normalize(d['RASIS']-d['LASIS'])
        zAxis=crossAxis(xAxis,normalize(origin-(d['LPSIS']+d['RPSIS'])/2)) 
        yAxis=crossAxis(zAxis,xAxis)
        return getTransformation(xAxis,yAxis,zAxis,origin) 
    except:
        return None

def getFemurM(hip,FME,FLE): #hip is preclaculated, FME inside, FLE outside, must be the same frame
    mid=(FME+FLE)/2
    xAxis=normalize(mid-hip[:3])
    zAxis=crossAxis(xAxis,normalize(FLE-FME))
    yAxis=crossAxis(zAxis,xAxis)
    return getTransformation(xAxis,yAxis,zAxis,hip) 

def getTibiaM(knee,FAL,TAM):
    mid=(FAL+TAM)/2
    xAxis=normalize(mid-knee[:3])
    zAxis=crossAxis(xAxis,normalize(TAM-FAL))
    yAxis=crossAxis(zAxis,xAxis)
    return getTransformation(xAxis,yAxis,zAxis,knee) 

def getTalusM_orientation(FCC,FMT1,FMT5):
    xAxis=normalize(FMT1-FCC)
    zAxis=crossAxis(xAxis,normalize(FMT5-FMT1))
    yAxis=crossAxis(zAxis,xAxis)
    return combine3axis(xAxis,yAxis,zAxis)
    #return getTransformation(xAxis,yAxis,zAxis,knee)

def getArmM(armPoint,HME,HLE):   #armPoint  is precalculated, must be the same frame
    mid=(HME+HLE)/2
    xAxis=normalize(mid-armPoint[:3])
    zAxis=crossAxis(xAxis,normalize(HLE-HME))
    yAxis=crossAxis(zAxis,xAxis)
    return getTransformation(xAxis,yAxis,zAxis,armPoint)

def getForeArmM(elbow,RSP,USP):
    mid=(RSP+USP)/2
    xAxis=normalize(mid-elbow[:3])
    zAxis=crossAxis(xAxis,normalize(USP-RSP))
    yAxis=crossAxis(zAxis,xAxis)
    return getTransformation(xAxis,yAxis,zAxis,elbow)