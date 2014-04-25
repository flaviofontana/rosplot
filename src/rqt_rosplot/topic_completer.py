# -*- coding: utf-8 -*-

from PyQt4.QtGui import QCompleter

class TopicCompleter(QCompleter):
    def __init__(self, parent=None):
        super(TopicCompleter,self).__init__()
        self.setCompletionMode(QCompleter.PopupCompletion)
        self.separator = '/'

    def splitPath(self, path):
        if path[0] == "/":
            path = path[1:]
        path_list = path.split(self.separator)
        if ':' in path_list[-1]:
            fields = path_list[-1].split(':')
            path_list[-1] = fields[-1]         
        return path_list
        
    def pathFromIndex(self,index):       
        fullpath = []     
        while index.isValid():
            field_of_current_item = index.internalPointer().data(0)
            fullpath.append( field_of_current_item )
            index = index.parent()
        fullpath.append('')
        fullpath.reverse()
        
        fullpath_str = self.separator.join(fullpath)
        
        current_field = fullpath.pop()  
        fullpath.append('')
        base_path = self.separator.join(fullpath)
        return fullpath_str

