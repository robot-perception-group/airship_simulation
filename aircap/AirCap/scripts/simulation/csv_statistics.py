#!/usr/bin/python3
import math
import sys
import statistics
#file1=open("copter44_cut.pose","r")
file1=sys.stdin

# 1496047391  1000000000

lines1=file1.readlines()


allvalues=[]
num=0
totals=[]
mins=[]
maxs=[]
alls=[]
first=1
columns=""

def castfloat(val):
        res=0.0
        try:
                res=(float)(val)
        except ValueError:
                res=0.0
        return res

for line in lines1:
    if line[:1]!='%':
        values=map(castfloat,line.split(","))
        num+=1
        if first:
                first=0
                for i, v in enumerate(values):
                        mins.append(v)
                        maxs.append(v)
                        totals.append(v)
                        alls.append([v])

        else:
                for i, v in enumerate(values):
                        if mins[i]>=v:
                                mins[i]=v
                        if maxs[i]<=v:
                                maxs[i]=v
                        totals[i]+=v
                        alls[i].append(v)
    else:
        columns=line.strip()

print ("%%Rows: %i " %(num))
print ("%%Statistics,%s"%(columns))
print ("Sum,%s"%(','.join(map(str,totals))))
print ("Min,%s"%(','.join(map(str,mins))))
print ("Max,%s"%(','.join(map(str,maxs))))
print ("Avg,%s"%(','.join(map(lambda x: (str)(x/num),totals))))
print ("Median,%s"%(','.join(map(lambda x: (str)(statistics.median(x)),alls))))
print ("Variance,%s"%(','.join(map(lambda x: (str)(statistics.variance(x)),alls))))

