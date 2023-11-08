import rospy
from visualization_msgs.msg import Marker

def make_gap(gap_center, gap_width):
  rospy.init_node('rviz_marker_pavement')

  marker_pub = rospy.Publisher("/env", Marker, queue_size = 2)

  marker_1 = Marker()

  marker_1.header.frame_id = "world"
  marker_1.header.stamp = rospy.Time.now()

  # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
  marker_1.type = 1
  marker_1.id = 0

  center_gap = 1.
  size_x_pavement = 2
  size_y_pavement = 10
  depth_pavement = 10
  gap = 0.5
  pavement_sink = 0.05
  # Set the scale of the marker

  center_pavement_1 = center_gap-size_x_pavement/2
  center_pavement_2 = center_gap+size_x_pavement/2

  marker_1.scale.x = size_x_pavement
  marker_1.scale.y = size_y_pavement
  marker_1.scale.z = depth_pavement

  # Set the color
  marker_1.color.r = 0.0
  marker_1.color.g = 1.0
  marker_1.color.b = 0.0
  marker_1.color.a = 1.0

  # Set the pose of the marker
  marker_1.pose.position.x = center_pavement_1 - gap/2
  marker_1.pose.position.y = 0
  marker_1.pose.position.z = -depth_pavement/2 -pavement_sink
  marker_1.pose.orientation.x = 0.0
  marker_1.pose.orientation.y = 0.0
  marker_1.pose.orientation.z = 0.0
  marker_1.pose.orientation.w = 1.0


  marker_2 = Marker()

  marker_2.header.frame_id = "world"
  marker_2.header.stamp = rospy.Time.now()

  marker_2.type = 1
  marker_2.id = 1

  # Set t2e scale of the marker
  marker_2.scale.x = size_x_pavement
  marker_2.scale.y = size_y_pavement
  marker_2.scale.z = depth_pavement

  # Set t2e color
  marker_2.color.r = 0.0
  marker_2.color.g = 1.0
  marker_2.color.b = 0.0
  marker_2.color.a = 1.0

  # Set t2e pose of the marker
  marker_2.pose.position.x = center_pavement_2 + gap/2
  marker_2.pose.position.y = 0
  marker_2.pose.position.z = -depth_pavement/2-pavement_sink
  marker_2.pose.orientation.x = 0.0
  marker_2.pose.orientation.y = 0.0
  marker_2.pose.orientation.z = 0.0
  marker_2.pose.orientation.w = 1.0



  rate = rospy.Rate(1/0.01)
  while not rospy.is_shutdown():
    marker_pub.publish(marker_1)
    marker_pub.publish(marker_2)
    rate.sleep()

def make_box(pos, size=[1, 1, 1]):

  rospy.init_node('rviz_marker_box')

  marker_pub = rospy.Publisher("/env", Marker, queue_size = 2)

  marker = Marker()

  marker.header.frame_id = "world"
  marker.header.stamp = rospy.Time.now()

  # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
  marker.type = 1
  marker.id = 0

  # Set the scale of the marker
  marker.scale.x = size[0]
  marker.scale.y = size[1]
  marker.scale.z = size[2]

  # Set the color
  marker.color.r = 0.0
  marker.color.g = 0.0
  marker.color.b = 1.0
  marker.color.a = 1.0

  # Set the pose of the marker
  marker.pose.position.x = pos[0]
  marker.pose.position.y = pos[1]
  marker.pose.position.z = pos[2]
  marker.pose.orientation.x = 0.0
  marker.pose.orientation.y = 0.0
  marker.pose.orientation.z = 0.0
  marker.pose.orientation.w = 1.0


  rate = rospy.Rate(1/0.01)
  while not rospy.is_shutdown():
    marker_pub.publish(marker)
    rate.sleep()

if __name__ == '__main__':

  make_pavement()