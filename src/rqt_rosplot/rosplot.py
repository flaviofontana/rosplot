#!/usr/bin/env python
import os
import rospy
import argparse
from qt_gui.plugin import Plugin
from .rosplot_widget import RosplotWidget

class Rosplot(Plugin):
  """
  Subclass of Plugin to display Quad status
  """
  def __init__(self, context):
    # Init Plugin
    super(Rosplot, self).__init__(context)
    self._widget = RosplotWidget()
    
    if context.serial_number() > 1:
        self._widget.setWindowTitle('Rosplot' + (' (%d)' % context.serial_number()))
    else:
        self._widget.setWindowTitle('Rosplot')

    context.add_widget(self._widget)

