#!/usr/bin/python
# -*- coding: utf-8 -*-

from PyQt4.QtCore import QAbstractItemModel,QModelIndex, Qt, QVariant

import roslib
import rospy

from rostopic import get_topic_class 
from rostopic import get_topic_type
      
from tree_helper import *
roslib.load_manifest('rqt_rosplot')


class RosTopicTree(TreeModel):
    def __init__(self,parent=None):
        super(RosTopicTree, self).__init__(parent)
        self.update()
        
        
    def addSlot(self, slot_path, slot_type  ):
        current_index = QModelIndex()   # start at the root item
        subpaths = slot_path.split('/')[1:] # split the topic path in subpaths and drop the first zero item
        for subpath in subpaths[:-1]:
            current_index = self.addChild(subpath, '', current_index ) # add the subpath to the tree if it
        current_index = self.addChild(subpaths[-1], slot_type, current_index )
        
    def childId(self, name, item):
        for i in range(item.childCount()):
            if item.child(i).data(0) == name:
                return i
        return False
   
    def addChild(self,slot_name,slot_type, index):
        item = self.itemFromIndex(index)
        child_nr = self.childId(slot_name, item)
        if child_nr is False:
            item.appendChild(TreeItem([slot_name,slot_type]))
            return self.index(item.childCount()-1,0, index)
        else:
            return self.index(child_nr,0,index)         

    NumTypes = ['int8','uint8','int16','uint16','int32','uint32','int64','uint64','float32','float64' ]
    def typeFromPath(self, path):
        index = self.indexFromPath(path)
        
        if hasattr(index.internalPointer(), 'data'):
            return index.internalPointer().data(1)
        else:
            return None
    
    def isNummeric(self, path):          
        return self.typeFromPath(path) in self.NumTypes

    def allChildrenNummeric(self,path):
        if not path[-1] == '/':
            path += '/'
        index = self.indexFromPath(path)
        item = index.internalPointer()
        if item is None:
            return False
        if item.childCount() == 0:
            return False        
        for i in range(item.childCount()):
            child = item.child(i)            
            if not child.data(1) in self.NumTypes:
                return False
        return True

    def pathFromAllChildren(self,path):
        children_path = []
        if not path[-1] == '/':
            return children_path
        
        index = self.indexFromPath(path)
        item = index.internalPointer()
        if item is None:
            return children_path
        if item.childCount() == 0:
            return children_path          
        for i in range(item.childCount()):
            child = item.child(i)  
            children_path.append(path + child.data(0) )
        return children_path
               
    def indexFromPath(self, path):
        current_item = self.rootItem
        fields = [f for f in path.split('/') if f]
        for field in fields:
            child_nr = self.childId(field, current_item)
            if child_nr is False:
                return self.createIndex(0,0)
            else:
                current_item = current_item.child(child_nr)               
        return self.createIndex(0,0,current_item)  
        
    def pathFromIndex(self,index):
        fullpath = []
        self.separator = '/'
        while index.isValid():
            field_of_current_item = index.internalPointer().data(0)
            fullpath.append( field_of_current_item )
            index = index.parent()
        fullpath.reverse()
        fullpath_str = self.separator + self.separator.join(fullpath)
        return fullpath_str
   
    def update(self):
        self.reset()    
        self.rootItem = TreeItem(['name','type'])
        self.list = []    
        for topic_name, topic_type in rospy.get_published_topics():      
            message_class = roslib.message.get_message_class(topic_type)
            message_obj = message_class()
            self._inception(topic_name, message_obj, topic_type)
            
    def _slot_has_children(self, slot_class):
        return hasattr(slot_class, '__slots__') and hasattr(slot_class, '_slot_types')
      
    def _inception(self, slot_path, slot_class, slot_type):
        if self._slot_has_children(slot_class):
            for slot_name, slot_type in zip(slot_class.__slots__, slot_class._slot_types):
                child_slot_path = slot_path + '/' + slot_name
                child_slot_type = slot_type
                child_slot_class = getattr(slot_class, slot_name) #slot_class.'slot_name'
                self._inception(child_slot_path, child_slot_class, child_slot_type)
        else:
            self.list.append(slot_path)
            self.addSlot(slot_path,slot_type)                                                              # does not exist yet              
