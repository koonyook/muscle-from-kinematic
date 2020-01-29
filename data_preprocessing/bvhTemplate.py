hierarchySectionTemplate='''
HIERARCHY
ROOT Character1_Hips
{
	OFFSET %f %f %f
	CHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation
	JOINT Character1_Spine
	{
		OFFSET %f %f %f
		CHANNELS 3 Zrotation Xrotation Yrotation
		JOINT Character1_Spine1
		{
			OFFSET %f %f %f
			CHANNELS 3 Zrotation Xrotation Yrotation
			JOINT Character1_LeftShoulder
			{
				OFFSET %f %f %f
				CHANNELS 3 Zrotation Xrotation Yrotation
				JOINT Character1_LeftArm
				{
					OFFSET %f %f %f
					CHANNELS 3 Zrotation Xrotation Yrotation
					JOINT Character1_LeftForeArm
					{
						OFFSET %f %f %f
						CHANNELS 3 Zrotation Xrotation Yrotation
						JOINT Character1_LeftHand
						{
							OFFSET %f %f %f
							CHANNELS 3 Zrotation Xrotation Yrotation
							End Site
							{
								OFFSET 0.269758 0.141128 0.89614
							}
						}
						JOINT L_Radius_RIGMESH
						{
							OFFSET 0 10 0
							CHANNELS 3 Zrotation Xrotation Yrotation
							End Site
							{
								OFFSET 0 0 0
							}
						}
					}
					JOINT L_Humerus_RIGMESH
					{
						OFFSET 0 10 0
						CHANNELS 3 Zrotation Xrotation Yrotation
						End Site
						{
							OFFSET 0 0 0
						}
					}
				}
				JOINT L_Shoulder_RIGMESH
				{
					OFFSET 0 0 10
					CHANNELS 3 Zrotation Xrotation Yrotation
					End Site
					{
						OFFSET 0 0 0
					}
				}
			}
			JOINT Character1_RightShoulder
			{
				OFFSET %f %f %f
				CHANNELS 3 Zrotation Xrotation Yrotation
				JOINT Character1_RightArm
				{
					OFFSET %f %f %f
					CHANNELS 3 Zrotation Xrotation Yrotation
					JOINT Character1_RightForeArm
					{
						OFFSET %f %f %f
						CHANNELS 3 Zrotation Xrotation Yrotation
						JOINT Character1_RightHand
						{
							OFFSET %f %f %f
							CHANNELS 3 Zrotation Xrotation Yrotation
							End Site
							{
								OFFSET -0.269758 0.141827 0.896322
							}
						}
						JOINT R_Radius_RIGMESH
						{
							OFFSET 0 10 0
							CHANNELS 3 Zrotation Xrotation Yrotation
							End Site
							{
								OFFSET 0 0 0
							}
						}
					}
					JOINT R_Humerus_RIGMESH
					{
						OFFSET 0 10 0
						CHANNELS 3 Zrotation Xrotation Yrotation
						End Site
						{
							OFFSET 0 0 0
						}
					}
				}
				JOINT R_Shoulder_RIGMESH
				{
					OFFSET 0 0 10
					CHANNELS 3 Zrotation Xrotation Yrotation
					End Site
					{
						OFFSET 0 0 0
					}
				}
			}
			JOINT Character1_Neck
			{
				OFFSET %f %f %f
				CHANNELS 3 Zrotation Xrotation Yrotation
				JOINT Character1_Head
				{
					OFFSET %f %f %f
					CHANNELS 3 Zrotation Xrotation Yrotation
					End Site
					{
						OFFSET 0 0 10
					}
				}
				JOINT Neck_RIGMESH
				{
					OFFSET 0 0 10
					CHANNELS 3 Zrotation Xrotation Yrotation
					End Site
					{
						OFFSET 0 0 0
					}
				}
			}
			JOINT Torso_RIGMESH
			{
				OFFSET 0 0 10
				CHANNELS 3 Zrotation Xrotation Yrotation
				End Site
				{
					OFFSET 0 0 0
				}
			}
		}
		JOINT Spine_RIGMESH
		{
			OFFSET 0 0 10
			CHANNELS 3 Zrotation Xrotation Yrotation
			End Site
			{
				OFFSET 0 0 0
			}
		}
	}
	JOINT Character1_RightUpLeg
	{
		OFFSET %f %f %f
		CHANNELS 3 Zrotation Xrotation Yrotation
		JOINT Character1_RightLeg
		{
			OFFSET %f %f %f
			CHANNELS 3 Zrotation Xrotation Yrotation
			JOINT Character1_RightFoot
			{
				OFFSET %f %f %f
				CHANNELS 3 Zrotation Xrotation Yrotation
				JOINT Character1_RightToeBase
				{
					OFFSET 1.87056 -6.69517 14.7145
					CHANNELS 3 Zrotation Xrotation Yrotation
					End Site
					{
						OFFSET -10 0 0
					}
				}
				JOINT R_Talus_RIGMESH
				{
					OFFSET -10 0 0
					CHANNELS 3 Zrotation Xrotation Yrotation
					End Site
					{
						OFFSET 0 0 0
					}
				}
			}
			JOINT R_Tibia_RIGMESH
			{
				OFFSET -10 0 0
				CHANNELS 3 Zrotation Xrotation Yrotation
				End Site
				{
					OFFSET 0 0 0
				}
			}
		}
		JOINT R_Femur_RIGMESH
		{
			OFFSET -10 0 0
			CHANNELS 3 Zrotation Xrotation Yrotation
			End Site
			{
				OFFSET 0 0 0
			}
		}
	}
	JOINT Character1_LeftUpLeg
	{
		OFFSET %f %f %f
		CHANNELS 3 Zrotation Xrotation Yrotation
		JOINT Character1_LeftLeg
		{
			OFFSET %f %f %f
			CHANNELS 3 Zrotation Xrotation Yrotation
			JOINT Character1_LeftFoot
			{
				OFFSET %f %f %f
				CHANNELS 3 Zrotation Xrotation Yrotation
				JOINT Character1_LeftToeBase
				{
					OFFSET -1.87056 -6.69517 14.7145
					CHANNELS 3 Zrotation Xrotation Yrotation
					End Site
					{
						OFFSET 10 0 0
					}
				}
				JOINT L_Talus_RIGMESH
				{
					OFFSET 10 0 0
					CHANNELS 3 Zrotation Xrotation Yrotation
					End Site
					{
						OFFSET 0 0 0
					}
				}
			}
			JOINT L_Tibia_RIGMESH
			{
				OFFSET 10 0 0
				CHANNELS 3 Zrotation Xrotation Yrotation
				End Site
				{
					OFFSET 0 0 0
				}
			}
		}
		JOINT L_Femur_RIGMESH
		{
			OFFSET 10 0 0
			CHANNELS 3 Zrotation Xrotation Yrotation
			End Site
			{
				OFFSET 0 0 0
			}
		}
	}
	JOINT Pelvis_RIGMESH
	{
		OFFSET 0 0 10
		CHANNELS 3 Zrotation Xrotation Yrotation
		End Site
		{
			OFFSET 0 0 0
		}
	}
}
'''

def generateHierarchySection(jointCenterDict):
		j=jointCenterDict
		offset=[]
		offset+=[0,0,0]	#list(j['Pelvis'])	#zeros are correct choice	
		offset+=list(j['Spine']-j['Pelvis'])
		offset+=list(j['Torso']-j['Spine'])
		
		offset+=list(j['ShoulderL']-j['Torso'])
		offset+=list(j['ArmL']-j['ShoulderL'])
		offset+=list(j['ForeArmL']-j['ArmL'])
		offset+=list(j['HandL']-j['ForeArmL'])
		
		offset+=list(j['ShoulderR']-j['Torso'])
		offset+=list(j['ArmR']-j['ShoulderR'])
		offset+=list(j['ForeArmR']-j['ArmR'])
		offset+=list(j['HandR']-j['ForeArmR'])

		offset+=list(j['Neck']-j['Torso'])
		offset+=list(j['Head']-j['Neck'])

		offset+=list(j['FemurR']-j['Pelvis'])
		offset+=list(j['TibiaR']-j['FemurR'])
		offset+=list(j['TalusR']-j['TibiaR'])

		offset+=list(j['FemurL']-j['Pelvis'])
		offset+=list(j['TibiaL']-j['FemurL'])
		offset+=list(j['TalusL']-j['TibiaL'])

		

		for i in range(len(offset)):
			offset[i]*=100

		return hierarchySectionTemplate%tuple(offset)

motionSectionHeaderTemplate='''
MOTION
Frames:	%d
Frame Time:	%.8f
'''

def generateMotionSection(fps,data): #data is list of list
		s=motionSectionHeaderTemplate%(len(data),1/fps)
		for aFrame in data:
				for aNum in aFrame:
						s+=str(aNum)+' '
				s+='\n'
		return s
		
def saveToBvh(filePath,jointCenterDict,fps,data):
		s=generateHierarchySection(jointCenterDict)+generateMotionSection(fps,data)
		f=open(filePath,'w')
		f.write(s)
		f.close()