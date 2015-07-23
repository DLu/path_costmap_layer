#!/usr/bin/python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
TOPICS = ['/p1_001/path', '/p1_002/path']

def get_pose(x=0,y=0):
    p = PoseStamped()
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.orientation.w = 1.0
    return p

rospy.init_node('path_generator')
pubs = {}
for topic in TOPICS:
    pub = rospy.Publisher(topic, Path, queue_size=1)
    pubs[topic] = pub

c = 0
rate = rospy.Rate(0.2)
while not rospy.is_shutdown():
    path = Path()
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = '/map'
    
    path.poses.append( get_pose(1,1) )
    path.poses.append( get_pose(1,2) )
    path.poses.append( get_pose(1,3) )
    path.poses.append( get_pose(1,4) )
    path.poses.append( get_pose(0,4) )
    path.poses.append( get_pose(-1,4) )
    path.poses.append( get_pose(-3,0) )
    
    pubs[ TOPICS[c] ].publish( path )
    
    rate.sleep()
print 'done'    
