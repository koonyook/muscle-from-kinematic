import numpy as np
import cv2
import math
import euler

def getSmallestRotMatBetweenTwoDirections(w_directionBefore,w_directionAfter):  #rotate frame (to keep the point constant)
    w_rotVec=getSmallestRotVecBetweenTwoDirections(w_directionBefore,w_directionAfter)
    return rotVecToRotMat(w_rotVec) #oldFrame_newFrame

def getSmallestRotVecBetweenTwoDirections(w_directionBefore,w_directionAfter):
    #print(w_directionBefore)
    #print(w_directionAfter)
    #print(math.acos(np.dot(w_directionBefore,w_directionAfter)))
    axis=crossAxis(w_directionBefore,w_directionAfter)
    w_rotVec=axis*math.acos(np.dot(w_directionBefore,w_directionAfter))
    return w_rotVec

def getAngleBetweenTwoDirection(w_directionBefore,w_directionAfter):
    return math.acos(np.dot(w_directionBefore,w_directionAfter))

def getSmallestRotMatFromThreePoints(C,P1,P2):   #C is center of rotation
    C=C[:3]
    P1=P1[:3]
    P2=P2[:3]
    worldA_directionBefore = normalize(P1 - C)
    worldA_directionAfter  = normalize(P2  - C)
    oldFrame_newFrame=rotationHomo(getSmallestRotMatBetweenTwoDirections(worldA_directionBefore,worldA_directionAfter))  #tested
    return oldFrame_newFrame

def combineRotVecAndTranslation(rotVec,translation):
    return combineRotMatAndTranslation(rotVecToRotMat(rotVec), translation)

def getTransformation(xAxis,yAxis,zAxis,origin):
    #t=np.zeros([3,4])
    t=np.eye(4)
    t[:3,0]=xAxis
    t[:3,1]=yAxis
    t[:3,2]=zAxis
    t[:3,3]=origin[:3]

    return t #3x4 transformation matrix (X,Y,Z,Origin)  #change to 4x4 on 11/11/19

def rotMatToRotVec(rotMat):
    rotVec,_=cv2.Rodrigues(rotMat)
    return np.ravel(rotVec)

def rotVecToRotMat(rotVec):
    rotMat,_=cv2.Rodrigues(rotVec)
    return rotMat

def getRotVecBetweenFrames(w_a,w_b):
    a_b = w_a[:3,:3].transpose() @ w_b[:3,:3]
    return rotMatToRotVec(a_b)

def allKeyExist(d,listOfKeys):
    return all(name in d for name in listOfKeys)

def length(v):
    return np.linalg.norm(v)

def normalize(v):
    if v.shape[0]==3:
        return v/np.linalg.norm(v)
    if v.shape[0]==4:
        return normalize(v[:3])

def crossAxis(v1,v2):
    return normalize(np.cross(v1,v2))

def homo(v):
    if(len(v.shape)==1 and v.shape[0]==3):
        return np.concatenate([v,np.array([1])],axis=0)
    elif(len(v.shape)==2 and v.shape[0]==3 and v.shape[1]==4):
        return np.concatenate([v,np.array([[0,0,0,1]])])
    elif(len(v.shape)==2 and v.shape[0]==3 and v.shape[1]==3):
        ans=np.identity(4)
        ans[:3,:3]=v
        return ans
    else:
        return v

def inverseHomo(T):
    ans=np.zeros([4,4])
    ans[3,3]=1
    ans[:3,:3]=T[:3,:3].transpose()
    ans[:3,3]= - ans[:3,:3] @ T[:3,3]
    return ans

def combine3axis(A_xAxisB,A_yAxisB,A_ZaxisB):
    A_Bframe=np.stack([A_xAxisB,A_yAxisB,A_ZaxisB],axis=-1)
    return A_Bframe

def combineRotMatAndTranslation(rotMat,tran):
    ans=homo(rotMat)
    ans[:3,3]=tran
    return ans

def translationHomo(tran):
    return combineRotMatAndTranslation(np.eye(3),tran)

def rotationHomo(rotMat):
    return combineRotMatAndTranslation(rotMat,np.zeros(3))

def doubleCrossAxis():
    return 0

def segmentR_segmentM_orientation():
    return 0

def rx(a):
    return np.array([
        [1,0,0],
        [0,np.cos(a),-np.sin(a)],
        [0,np.sin(a),np.cos(a)]
    ])

def ry(a):
    return np.array([
        [np.cos(a),0,np.sin(a)],
        [0,1,0],
        [-np.sin(a),0,np.cos(a)]
    ])

def rz(a):
    return np.array([
        [np.cos(a),-np.sin(a),0],
        [np.sin(a),np.cos(a),0],
        [0,0,1]
    ])


def eulerZXY(rotMat):   #use this version
    R=rotMat[:3,:3]
    y,x,z=euler.R_to_euler(R,'yxz','static')    #,second_sol=True)  
    
    #I feel that the quality could be bad sometimes
    # I should revert it back to rotMat can check orientation error in degree
    R2 = rz(z)@rx(x)@ry(y)    #it should be very close to R

    Rdiff = R2.transpose() @ R
    degDiff=length(rotMatToRotVec(Rdiff))*180/np.pi
    
    if(degDiff>0.1):
        print("*******",degDiff)    #tested, no issue

    a=wrap(z)*180/np.pi
    b=wrap(x)*180/np.pi
    c=wrap(y)*180/np.pi
    return [a,b,c]      #[Z,X,Y]

'''
def eulerZXY(rotMat):
    R=rotMat[:3,:3]
    z,x,y=euler.R_to_euler(R,'zxy','static')    #,second_sol=True)  
    
    #I feel that the quality could be bad sometimes
    # I should revert it back to rotMat can check orientation error in degree
    R2 = ry(y)@rx(x)@rz(z)    #it should be very close to R

    Rdiff = R2.transpose() @ R
    degDiff=length(rotMatToRotVec(Rdiff))*180/np.pi
    
    if(degDiff>0.1):
        print("*******",degDiff)

    a=wrap(z)*180/np.pi
    b=wrap(x)*180/np.pi
    c=wrap(y)*180/np.pi
    return [a,b,c]      #[Z,X,Y]
'''
'''
def eulerZXY(rotMat):
    #not smooth
    #z,x,y=euler.R_to_euler_zxy(rotMat[:3,:3])  
    #return [z*180/np.pi,x*180/np.pi,y*180/np.pi]
    
    #smooth but wrong direction
    #a,b,c=euler.R_to_euler(rotMat[:3,:3].transpose(),'yxz')  
    #return [a*180/np.pi,b*180/np.pi,c*180/np.pi] 
    
    #weird angle still come out (look the same as the simplest one)
    #a,b,c=euler.R_to_euler(rotMat[:3,:3].transpose(),'yxz')  
    #return [-c*180/np.pi,-b*180/np.pi,-a*180/np.pi]     #[Z,X,Y]

    #not smooth, looks like the first one (zxy original)
    #y,x,z=euler.R_to_euler(rotMat[:3,:3].transpose(),'yxz')  
    #return [-z*180/np.pi,-x*180/np.pi,-y*180/np.pi]

    #y,x,z=euler.R_to_euler(rotMat[:3,:3].transpose(),'yxz')  
    #return [x*180/np.pi,y*180/np.pi,z*180/np.pi]

    #simplest one that should work 
    a,b,c=euler.R_to_euler(rotMat[:3,:3],'zxy','static')    #,second_sol=True)  
    a=wrap(a)*180/np.pi
    b=wrap(b)*180/np.pi
    c=wrap(c)*180/np.pi
    return [a,b,c]      #[Z,X,Y]

    #a,b,c=euler.R_to_euler(rotMat[:3,:3].transpose(),'zxy','rotating')  #static or rotating
    #a*=180/np.pi
    #b*=180/np.pi
    #c*=180/np.pi
    #return [a,b,c]      #[Z,X,Y]
'''

def wrap(a):    #just to make number stay in a smaller range
    if(a>np.pi):
        return a-(2*np.pi)
    if(a<-np.pi):
        return a+(2*np.pi)
    return a