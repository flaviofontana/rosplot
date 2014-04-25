#!/usr/bin/env python
# Ros Plot Tool by fly.fontana@gmail.com
# this is a modified version of the rqt_plot tool written by Dorian Scholz

import os
import roslib
roslib.load_manifest('rqt_rosplot')
import rospy
import random

from PyQt4.QtGui import *
from PyQt4.QtCore import *
from PyQt4.uic import loadUi


#internal 
from plot_helper import * # check what this is for
from graph_widget import GraphWidget
from topic_completer import TopicCompleter
from ros_topic_tree import RosTopicTree

class RosplotWidget(QWidget):
    _redraw_interval = 40

    def __init__(self, arguments = None, initial_topics = None):
        super(RosplotWidget, self).__init__()
        
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../resource/plot_widget.ui')
        loadUi(ui_file, self)
        
        self.subscribe_topic_button.setIcon(QIcon.fromTheme('add'))
        self.remove_topic_button.setIcon(QIcon.fromTheme('remove'))
        self.clear_button.setText('Remove all Topics')

        self.subscribe_topic_button.setEnabled(False)

        self.zoom_button.setText('Zoom: On ')
        
        # Topic Tree and Topic Completer
        self.ros_topic_tree = RosTopicTree(self)        
        self._topic_completer = TopicCompleter(self)
        self._topic_completer.setModel(self.ros_topic_tree)     
        self.topic_edit.setCompleter(self._topic_completer)
        
        # the rosstuff
        self._start_time = rospy.get_time()
        self._rosdata = {}
        
        self._remove_topic_menu = QMenu()

        # init and start update timer for plot
        self._update_plot_timer = QTimer(self)
        self._update_plot_timer.timeout.connect(self.update_plot)
        self._update_plot_timer.start(self._redraw_interval)

        # graph window
        self.graph_widget = GraphWidget(self)
        self.graph_widget_layout.addWidget(self.graph_widget)
        QObject.connect(self.graph_widget, SIGNAL("legendChecked(QwtPlotItem *,bool)"), self.legend_clicked)                

        self.subscribed_topics_changed()
        self.show()

    def legend_clicked(self,item,val):
        topic_path = item.title().text()
        topic_path = str(topic_path)
        self.remove_topic( topic_path )
    
    @pyqtSlot(str)
    def on_topic_edit_textChanged(self, slot_name):
        if slot_name =="":
            print 'Updating topics'
            self.ros_topic_tree.update()
            return
        is_num = self.ros_topic_tree.isNummeric(slot_name)         
        all_children_num = self.ros_topic_tree.allChildrenNummeric(slot_name)     
        self.subscribe_topic_button.setEnabled(is_num or all_children_num)

    @pyqtSlot()
    def on_subscribe_topic_button_clicked(self):
        slot_name = str(self.topic_edit.text())
        if self.ros_topic_tree.isNummeric(slot_name):
            self.add_topic(slot_name)
        if self.ros_topic_tree.allChildrenNummeric(slot_name):
            if not slot_name[-1] == '/':
                slot_name += '/'
            for slot in self.ros_topic_tree.pathFromAllChildren(slot_name):
               self.add_topic(slot)

    @pyqtSlot()
    def on_clear_button_clicked(self):
        self.clean_up_subscribers()
        
    @pyqtSlot()
    def on_zoom_button_clicked(self):
        if self.graph_widget.zoom_enabled:
            self.graph_widget.zoom_enabled = False
            self.zoom_button.setText('Zoom: Off')
        else:
            self.graph_widget.zoom_enabled = True
            self.zoom_button.setText('Zoom: On ')
        
    def update_plot(self):
        if self.graph_widget is not None:
            for topic_name, rosdata in self._rosdata.items():
                try:
                    data_x, data_y = rosdata.next()
                    self.graph_widget.update_values(topic_name, data_x, data_y)
                except RosPlotException as e:
                    qWarning('PlotWidget.update_plot(): error in rosplot: %s' % e)
            self.graph_widget.redraw()

    def subscribed_topics_changed(self):
        self.update_remove_topic_menu()
        self.graph_widget.redraw()

    def update_remove_topic_menu(self):
        def make_remove_topic_function(x):
            return lambda: self.remove_topic(x)

        self._remove_topic_menu.clear()
        for topic_name in sorted(self._rosdata.keys()):
            action = QAction(topic_name, self._remove_topic_menu)
            action.triggered.connect(make_remove_topic_function(topic_name))
            self._remove_topic_menu.addAction(action)
        self.remove_topic_button.setMenu(self._remove_topic_menu)

    def add_topic(self, topic_name):
        if topic_name in self._rosdata:
            qWarning('PlotWidget.add_topic(): topic already subscribed: %s' % topic_name)
            return
        self._rosdata[topic_name] = ROSData(topic_name, self._start_time)
        data_x, data_y = self._rosdata[topic_name].next()
        self.graph_widget.add_curve(topic_name, topic_name, data_x, data_y)
        self.subscribed_topics_changed()

    def remove_topic(self, topic_name):
        self._rosdata[topic_name].close()
        del self._rosdata[topic_name]
        self.graph_widget.remove_curve(topic_name)
        self.subscribed_topics_changed()

    def clean_up_subscribers(self):
        for topic_name, rosdata in self._rosdata.items():
            rosdata.close()
            self.graph_widget.remove_curve(topic_name)
        self._rosdata = {}
        self.subscribed_topics_changed()



