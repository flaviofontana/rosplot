

from PyQt4.QtGui import *
from PyQt4.QtCore import *
import sys
import rospy
import rospkg

from topic_completer import TopicCompleter
from ros_topic_tree import RosTopicTree

class RosTopicTreeTester(QWidget):
    def __init__(self):
        super(RosTopicTreeTester, self).__init__()
        self.initUI()
        
    def initUI(self):
        self.setGeometry(800, 300, 400, 400)
        self.setWindowTitle('QtGui.QLineEdit')

        start_time = rospy.Time.now()
        self.ros_topic_tree = RosTopicTree()        
        print( 'It took %.2f ms to pupulate the tree' % ((rospy.Time.now() - start_time).to_sec()*1000 ) ) 
 
        self.topic_tree_view= QTreeView(self)
        self.topic_tree_view.setModel(self.ros_topic_tree)
 
        self.topic_tree_completer = TopicCompleter(self)
        self.topic_tree_completer.setModel(self.ros_topic_tree)
        
        self.text_field = QLineEdit(self)
        self.text_field.move(0, 200)
        self.text_field.resize(300,20)
        self.text_field.setCompleter(self.topic_tree_completer)
        
        self.show()
        
def main():
    app = QApplication(sys.argv)
    rospy.init_node('RosTopicTreeTester')
    ros_topic_tree_tester = RosTopicTreeTester()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
