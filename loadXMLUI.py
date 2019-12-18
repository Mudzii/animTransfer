from maya import OpenMayaUI as omui
import PySide2
from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2 import QtWidgets 
from PySide2.QtUiTools import *
from shiboken2 import wrapInstance

def getMayaWin():
	mayaWinPtr = omui.MQtUtil.mainWindow( )
	mayaWin = wrapInstance( long( mayaWinPtr ), QtWidgets.QMainWindow )


def loadUI( path ):
	loader = QUiLoader()
	uiFile = QFile( path )

	dirIconShapes = ""
	buff = None

	if uiFile.exists():
		dirIconShapes = path
		uiFile.open( QFile.ReadOnly )

		buff = QByteArray( uiFile.readAll() )
		uiFile.close()
	else:
		print "UI file missing! Exiting..."
		exit(-1)

	fixXML( path, buff )
	qbuff = QBuffer()
	qbuff.open( QBuffer.ReadOnly | QBuffer.WriteOnly )
	qbuff.write( buff )
	qbuff.seek( 0 )
	ui = loader.load( qbuff, parentWidget = getMayaWin() )
	ui.path = path

	return ui


def fixXML( path, qbyteArray ):
	# first replace forward slashes for backslashes
	if path[-1] != '/':
		path += '/'
	path = path.replace( "/", "\\" )

	# construct whole new path with <pixmap> at the begining
	tempArr = QByteArray( "<pixmap>" + path + "\\" )

	# search for the word <pixmap>
	lastPos = qbyteArray.indexOf( "<pixmap>", 0 )
	while lastPos != -1:
		qbyteArray.replace( lastPos, len( "<pixmap>" ), tempArr )
		lastPos = qbyteArray.indexOf( "<pixmap>", lastPos + 1 )
	return


class UIController:
	def __init__(self, ui):
		# Connect each signal to it's slot one by one
		#ui.myButton.clicked.connect(self.ButtonClicked)

		self.ui = ui
		ui.setWindowFlags(Qt.WindowStaysOnTopHint)
		ui.show()