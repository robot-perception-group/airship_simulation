#!/usr/bin/env python

import sys
import math


file1=open(sys.argv[1],'r')
file2=open(sys.argv[2],'r')
lines1=file1.readlines()
lines2=file2.readlines()

#remove headers
del lines1[0]
del lines2[0]

print("%Timestamp,X,Y,Z,2D,3D,VarX,VarY,VarZ,Var2D,Var3D")
left=0
right=0
while (left<len(lines1) and right<len(lines2)):
	fields1=lines1[left].split(',')
	fields2=lines2[right].split(',')
	t1=(int)(fields1[2])
	t2=(int)(fields2[2])
	if (t1<t2):
		left=left+1
	elif (t2<t1):
		right=right+1
	else:
		X=(float)(fields2[4]) - (float)(fields1[4])
		Y=(float)(fields2[5]) - (float)(fields1[5])
		Z=(float)(fields2[6]) - (float)(fields1[6])
		XY=math.sqrt(X*X+Y*Y)
		XYZ=math.sqrt(X*X+Y*Y+Z*Z)
		vX=(float)(fields2[11+0*6+0])
		vY=(float)(fields2[11+1*6+1])
		vZ=(float)(fields2[11+2*6+2])
		vXY=math.sqrt(vX*vX+vY*vY)
		vXYZ=math.sqrt(vX*vX+vY*vY+vZ*vZ)

		print("%i,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f"%(t1,X,Y,Z,XY,XYZ,vX,vY,vZ,vXY,vXYZ))
		left=left+1
		right=right+1


