def getTaxelPosition(taxelNumber,scale1,scale2):

	lvl1 = 1 + scale1
	lvl2 = 1 + scale1 + scale2
	
	if taxelNumber == 1:
		x = 3
		y = 0
	elif taxelNumber == 2:
		x = 2
		y = 0
	elif taxelNumber == 3:
		x = 2
		y = 1
	elif taxelNumber == 4:
		x = 3
		y = 1
	elif taxelNumber == 5:
		x = 3
		y = lvl1
	elif taxelNumber == 6:
		x = 2
		y = lvl1
	elif taxelNumber == 7:
		x = 1.5
		y = lvl2
	elif taxelNumber == 8:
		x = 1
		y = lvl1
	elif taxelNumber == 9:
		x = 0
		y = lvl1
	elif taxelNumber == 10:
		x = 0
		y = 1
	elif taxelNumber == 11:
		x = 1
		y = 1
	elif taxelNumber == 12:
		x = 1
		y = 0		
	return [1.0*x,1.0*y]

def getContactPosition(tactileArray):

	tactileSum = 0
	xTemp = 0
	yTemp = 0
	for i in range(12):
		taxPos = getTaxelPosition(i+1,1,1);
		tactileSum = tactileSum + tactileArray[i]
		xTemp = xTemp + tactileArray[i]*taxPos[0]
		yTemp = yTemp + tactileArray[i]*taxPos[1]
		
	x = xTemp/tactileSum
	y = yTemp/tactileSum
	
	return [x,y]		
		