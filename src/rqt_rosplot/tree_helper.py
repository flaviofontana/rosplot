# -*- coding: utf-8 -*-

from PyQt4.QtCore import QAbstractItemModel,QModelIndex, Qt, QVariant

class TreeItem():
    def __init__(self, data, parent=None):
        self.parentitem = parent # TreeItem parent
        self.childItems = [] # QList<TreeItem*> childItems;
        self.itemData = data # QList<QVariant> itemData;
        
    def appendChild(self, child):
        self.childItems.append(child)
        child.parentitem = self
        
    def child(self, row):
        return self.childItems[row]
        
    def childCount(self):
        return len(self.childItems)
        
    def columnCount(self):
        return len(self.itemData)
        
    def data(self, column):
        return self.itemData[column]
        
    def parent(self):
        return self.parentitem
        
    def row(self):
        if(self.parentitem):
            return self.parentitem.childItems.index(self)
        else:
            return 0
            
class TreeModel(QAbstractItemModel):
    def __init__(self,parent=None):
        super(TreeModel, self).__init__(parent)
        
    # returns the data from a selected index
    # index consits of a parent(level), row, column
    def data(self, index, role):
       if not index.isValid() :
           return None
       if not (role == Qt.DisplayRole or role == 2 ):
           return None
       if index.internalPointer() is None:
           return None
       return index.internalPointer().data(index.column())
    
    # generates an index to a selected item
    # index consits of a parent(level), row, column
    def index(self, row, column, index = QModelIndex()):       
        item = self.itemFromIndex(index)
        if row < item.childCount():
            childItem = item.child(row)
            return self.createIndex(row,column, childItem)
        else:
            # this child does not exist
            return QModelIndex()
   
    # return parent item if available otherwiese the root item  
    def itemFromIndex(self,index):
        if not index.isValid(): 
            return self.rootItem
        else:
            return index.internalPointer()        
   
    # return an index to a parent of a selected item
    def parent(self, index):
        if not index.isValid():
            return QModelIndex()
        parentItem = index.internalPointer().parent()
        if (parentItem is None):          # the rootItem does not have a parent item
            return QModelIndex()     
        if (parentItem is self.rootItem): # do not return the rootItem, because that would screew the autocomplete
            return QModelIndex()
        return self.createIndex(parentItem.row(), 0, parentItem);
                
    def rowCount(self, index = QModelIndex()):
        if index.isValid():
            return index.internalPointer().childCount()
        else:
            return self.rootItem.childCount()
        
    def columnCount(self, index):
        if(index.isValid()):
            return index.internalPointer().columnCount()
        else:
            return self.rootItem.columnCount()

    def flags(self, index):
        if not index.isValid():
            return Qt.ItemFlags()
        return Qt.ItemIsEnabled | Qt.ItemIsSelectable
        
    def headerData(self, column, orientation, role):
        if (orientation == Qt.Horizontal and role == Qt.DisplayRole):
            return self.rootItem.data(column)
        return QVariant()