#!/usr/bin/env python
# Extension of the QLineEdit class that captures the event when the tab key is pressed

from PyQt4.QtCore import * 
from PyQt4.QtGui import * 

class MyLineEditWithTabPressedEvent(QLineEdit):
    def __init__(self, *args):
        QLineEdit.__init__(self, *args)
        
    def event(self, event):
        if (event.type()==QEvent.KeyPress) and (event.key()==Qt.Key_Tab):
            self.emit(SIGNAL("tabPressed"))
            return True

        return QLineEdit.event(self, event)