import numpy as np
class MarkerRecord:
	
	def __init__(self,filepath,rotateToBvhFrame=False):
		self.markerNameList = []
		self.record = []
		self.frameNo = []
		self.timestamp = []	 #in second 

		if(rotateToBvhFrame):
			rotator=np.array([
				[ 0,-1, 0],
				[ 0, 0, 1],
				[-1, 0, 0]
			])	#this is bvhFrame_mocapFrame which cause vertical axis to be Y-axis (instead of Z-axis in mocap frame)
		else:
			rotator=np.eye(3)

		counter=0

		f=open(filepath,'r')
		while(True):
			line=f.readline()
			if(line==''):
				f.close()
				break
			else:
				s=line.strip().split('\t')
				if(s[0]=="NO_OF_FRAMES"):
					pass
				elif(s[0]=="NO_OF_CAMERAS"):
					pass
				elif(s[0]=="NO_OF_MARKERS"):
					pass
				elif(s[0]=="FREQUENCY"):
					self.frequency=float(s[1])
				elif(s[0]=="NO_OF_ANALOG"):
					pass
				elif(s[0]=="ANALOG_FREQUENCY"):
					pass
				elif(s[0]=="DESCRIPTION"):
					self.description=s[1]
				elif(s[0]=="TIME_STAMP"):		#timestamp in seconds from when the computer was started.
					pass
				elif(s[0]=="DATA_INCLUDED"):
					pass
				elif(s[0]=="EVENT"):
					pass
				elif(s[0]=="MARKER_NAMES"):	#read all marker's name
					self.markerNameList=s[1:]
					#print(self.markerNameList)
				elif(s[0]=="Frame" and s[1]=="Time"):		#header above each column
					pass
				else:
					if(len(s)>len(self.markerNameList)*3):	#frame number and time in the first 2 columns
						self.frameNo.append(int(s[0]))
						self.timestamp.append(float(s[1]))
						numbers=s[2:]
					else:	#no frame number and timestamp info
						numbers=s
					
					row={}
					for i,markerName in enumerate(self.markerNameList):
						if(numbers[i*3+0]=="NULL" and numbers[i*3+1]=="NULL" and numbers[i*3+2]=="NULL"):
							continue	#just skip this marker (normally happen at the early or late frame)
						else:
							row[markerName]=rotator @ np.array([
								float(numbers[i*3+0])/1000,
								float(numbers[i*3+1])/1000,
								float(numbers[i*3+2])/1000
							])
					self.record.append(row)