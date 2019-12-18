import sys
sys.path.append("D:/01. School stuff/Animation")

import loadXMLUI
import pymel.core as pm
import pymel.core.datatypes as dt
reload(loadXMLUI)

## Define path to ui file
pathToFile = "D:/01. School stuff/Animation/animationTransferUI.ui"

# load and s
ui = loadXMLUI.loadUI(pathToFile)
cont = loadXMLUI.UIController(ui)

# =================================================================#
# ====================== VARIABLES ================================#
# =================================================================#
nrOfFrames = 0
nrOfJoints = 0

"""source """
SourceRootJointName = ""
sourceRootJoint = ""
SourceJointList = []


"""Target """
TargetRootJointName = ""
targetRootJoint = ""
TargetJointList = []

del SourceJointList[:]
del TargetJointList[:]

# =================================================================#
# ======================= ANIM FUNCTIONS ==========================#
# =================================================================#

""" ..... Fill list with joints ..... """ 
def getSkeletonInfo(node, jntList):
    for child in node.getChildren():

        if child.getChildren() > 0:
            jntList.append(child)

            getSkeletonInfo(child, jntList)

""" ..... Set Translation for root joint ..... """	
def SetTranslation(nrOfFrames, source, target): 
	for x in range(nrOfFrames):
		pm.copyKey(source, time=(x, x), attribute='translate', option="curve")
		pm.pasteKey(target, time=(x, x), attribute='translate')	
		
		
		
# =================================================================#
# ==================== TRANSFER ANIMATION =========================#
# =================================================================#

def TransferAnimation(jointCount, frameCount, sourceList, targetList):
	
	
	for jointIndex in range(jointCount):
		
		
		""" SOURCE - Get bindpose and orientations """ 
		sourceJointBindPose = sourceList[jointIndex].getRotation().asMatrix()
		sourceJoindBindPoseInversed = sourceJointBindPose.inverse()
		
		sourceJointOrientation = sourceList[jointIndex].getOrientation().asMatrix()
		sourceJointOrientationInversed = sourceJointOrientation.inverse().asMatrix()
		
		""" SOURCE - Get parent Matricies """ 
		sourceParentMatrix = dt.Matrix()
		sourceParentList = []
		sourceParentList = sourceList[jointIndex].getAllParents()
		parLenS = len(sourceParentList)
		
		if parLenS > 0:
			for prnt in sourceParentList:
				rot = prnt.getRotation().asMatrix()
				ori = prnt.getOrientation().asMatrix()
				sourceParentMatrix *= ori * rot
				
		elif jointIndex is jointCount - 1:
			rot = dt.Matrix()
			ori = dt.Matrix()
			sourceParentMatrix *= ori * rot
			
		sourceParentMatrixInverse = sourceParentMatrix.inverse().asMatrix()
		
		""" TARGET -  Get Bindpose and Orientations """
		targetJointBindPose = targetList[jointIndex].getRotation().asMatrix()
		
		targetJointOrientation = targetList[jointIndex].getOrientation().asMatrix()
		targetJointOrientationInverse = targetJointOrientation.inverse().asMatrix()
		
		""" TARGET - Get parent matricies """
		targetParentMatrix = dt.Matrix()
		targetParentList = []
		targetParentList = targetList[jointIndex].getAllParents()
		parLenT = len(targetParentList)
		
		if parLenT > 0:
			for tPrnt in targetParentList:
				rotT = tPrnt.getRotation().asMatrix()
				oriT = tPrnt.getOrientation().asMatrix()
				targetParentMatrix *= oriT * rotT
				
		elif jointIndex is jointCount - 1:
			rotT = dt.Matrix()
			oriT = dt.Matrix()
			targetParentMatrix *= oriT * rotT
			
		targetParentMatrixInverse = targetParentMatrix.inverse().asMatrix()
		
			
		for frame in range(frameCount):		
			""" SOURCE - Calculate K """
			
			if jointIndex is jointCount - 1:
				sourceJointFinalRot = pm.keyframe(sourceList[jointIndex], time=(frame, frame), query=True, eval=True)[7:10]		
			else:
				sourceJointFinalRot = pm.keyframe(sourceList[jointIndex], time=(frame, frame), query=True, eval=True)[6:9]
			
			sourceJointFinalRotEuler = dt.EulerRotation(sourceJointFinalRot).asMatrix()
			
			
			isolatedRotK = sourceJoindBindPoseInversed * sourceJointFinalRotEuler
			kPrim = sourceJointOrientationInversed * sourceParentMatrixInverse * isolatedRotK * sourceParentMatrix * sourceJointOrientation
			kBis = targetJointOrientation * targetParentMatrix * kPrim * targetParentMatrixInverse * targetJointOrientationInverse
			
		
			finalRotation = targetJointBindPose * kBis
			
			FinEuler = dt.EulerRotation(finalRotation)
			FinDegrees = dt.degrees(FinEuler)
			
			"""SET FINAL ROTATION . . . . . . . . . . . . . . . . . . """
			pm.setKeyframe(targetList[jointIndex], time=(frame, frame), attribute='rotateX', value=FinDegrees.x)
			pm.setKeyframe(targetList[jointIndex], time=(frame, frame), attribute='rotateY', value=FinDegrees.y)
			pm.setKeyframe(targetList[jointIndex], time=(frame, frame), attribute='rotateZ', value=FinDegrees.z)
			
		del targetParentList[:]
		del sourceParentList[:]
				
		

# =================================================================#
# ===================== BUTTON FUNCTIONS ==========================#
# =================================================================#

""" ..... Move joints UP ..... """ 
def TargetUp():
    currentRow = ui.TargetJointList.currentRow()
    currentItem = ui.TargetJointList.takeItem(currentRow)
    ui.TargetJointList.insertItem(currentRow - 1, currentItem)
    
    temp = TargetJointList[currentRow]
    TargetJointList[currentRow] = TargetJointList[currentRow - 1]
    TargetJointList[currentRow - 1] = temp
   
   
def SourceUp():    
    currentRow = ui.SourceJointList.currentRow()
    currentItem = ui.SourceJointList.takeItem(currentRow)
    ui.SourceJointList.insertItem(currentRow - 1, currentItem)
   
    temp = SourceJointList[currentRow]
    SourceJointList[currentRow] = SourceJointList[currentRow - 1]
    SourceJointList[currentRow - 1] = temp
      
""" ..... Move Joints Down ..... """     
def TargetDown():
	currentRow = ui.TargetJointList.currentRow()
	currentItem = ui.TargetJointList.takeItem(currentRow)
	ui.TargetJointList.insertItem(currentRow + 1, currentItem)
	
	temp = TargetJointList[currentRow]
	TargetJointList[currentRow] = TargetJointList[currentRow + 1]
	TargetJointList[currentRow + 1] = temp
	
def SourceDown():
	currentRow = ui.SourceJointList.currentRow()
	currentItem = ui.SourceJointList.takeItem(currentRow)
	ui.SourceJointList.insertItem(currentRow + 1, currentItem)	
	
	temp = SourceJointList[currentRow]
	SourceJointList[currentRow] = SourceJointList[currentRow + 1]
	SourceJointList[currentRow + 1] = temp
    
""" ..... Delete Joints ..... """ 
def DelSource():
    for SelectedItem in ui.SourceJointList.selectedItems():
    	ui.SourceJointList.takeItem(ui.SourceJointList.row(SelectedItem))
    	currentRow = ui.SourceJointList.currentRow()
    	SourceJointList.pop(currentRow)
    	   	
    	
def DelTarget():
    for SelectedItem in ui.TargetJointList.selectedItems():
    	ui.TargetJointList.takeItem(ui.TargetJointList.row(SelectedItem))
    	currentRow = ui.TargetJointList.currentRow()
    	TargetJointList.pop(currentRow)
    	
""" ..... Add Root Joint ..... """ 
def AddSourceRootJoint():
	SourceRootJointName = ui.SourceRootBox.text()
	SourceRootJointName = SourceRootJointName[1:]
	
	pm.select(SourceRootJointName)
	sourceRootJoint = pm.ls(sl=True)[0]
	
	SourceJointList.append(sourceRootJoint)
	getSkeletonInfo(sourceRootJoint, SourceJointList)
	
	for x in SourceJointList:
		ui.SourceJointList.addItem(str(x))

	
def AddTargetRootJoint():
	TargetRootJointName = ui.TargetRootBox.text()
	TargetRootJointName = TargetRootJointName[1:]
	
	pm.select(TargetRootJointName)
	targetRootJoint = pm.ls(sl=True, type='transform')[0]
	
	TargetJointList.append(targetRootJoint)
	getSkeletonInfo(targetRootJoint, TargetJointList)
	
	for x in TargetJointList:
		ui.TargetJointList.addItem(str(x))

""" ..... Transfer Animation ..... """ 	
def TransferAnimationButton():
	
	sourceRootJoint = SourceJointList[0]
	targetRootJoint = TargetJointList[0]
	
	nrOfTargetJoints = len(TargetJointList)
	nrOfSourceJoints = len(SourceJointList) 
	

	if nrOfSourceJoints == nrOfTargetJoints:
		sys.stdout.write("Source skeleton has equal or less nr of bones than target. Transfering animation") 
		SourceJointListRev = SourceJointList[::-1]
		TargetJointListRev = TargetJointList[::-1]
			
		nrOfFrames = pm.keyframe(sourceRootJoint, query=True, keyframeCount=True) / 10
		print "Frames: " +str(nrOfFrames)
		
		test = 50
		
		TransferAnimation(nrOfSourceJoints, nrOfFrames, SourceJointListRev, TargetJointListRev)
		SetTranslation(nrOfFrames, sourceRootJoint, targetRootJoint)
	else:
		sys.stdout.write("ERROR! Different ammount of joints!") 
		
		

# =================================================================#
# ===================== BUTTON CONNECTIONS ========================#
# =================================================================#

"""Buttons"""
ui.TUp.clicked.connect(TargetUp)
ui.SUp.clicked.connect(SourceUp)
ui.TDown.clicked.connect(TargetDown)
ui.SDown.clicked.connect(SourceDown)
ui.TDelete.clicked.connect(DelTarget)
ui.SDelete.clicked.connect(DelSource)

ui.TransferAnim.clicked.connect(TransferAnimationButton)

ui.SourceRootBox.returnPressed.connect(AddSourceRootJoint)
ui.TargetRootBox.returnPressed.connect(AddTargetRootJoint)

# =================================================================#
# =================================================================#
# =================================================================#





