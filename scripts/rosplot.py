#!/usr/bin/env python
# Ros Plot Tool by fly.fontana@gmail.com
# this is a modified version of the rqt_plot tool written by Dorian Scholz


import roslib
roslib.load_manifest('rosplot')
import rospy

import os
import sys

from PyQt4.QtGui import *
from PyQt4.QtCore import *
from PyQt4.uic import loadUi

pkg_path = roslib.packages.find_resource('rosplot', 'topic_tree.py')
sys.path.append(os.path.dirname(pkg_path[0]))

#internal 
from plot_widget import *
from topic_autocomplete import *
from topic_tree import *

def parse_args(argv):
    extended_topics = []
    if len(argv) >1:
        topics = argv[1:]      
        for t in topics:
            c_topics = []
            # check for shorthand '/foo/field1:field2:field3'
            if ':' in t:
                base = t[:t.find(':')]
                # the first prefix includes a field name, so save then strip it off
                c_topics.append(base)
                base = base[:base.rfind('/')]
                
                # compute the rest of the field names
                fields = t.split(':')[1:]
                c_topics.extend(["%s/%s"%(base, f) for f in fields if f])
                extended_topics.extend(c_topics)
            else:
                extended_topics.append(t)
    return extended_topics

def main():       
    app = QApplication(sys.argv)
    plot = PlotWidget()
    
    topics = parse_args(sys.argv)
    for t in topics:
        plot.add_topic(t)
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()