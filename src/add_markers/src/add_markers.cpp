#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  struct MarkerData {
    float pos_x, pos_y, pos_z;
    float or_x, or_y, or_z, or_w; 
  };
  
  MarkerData marker_in_pick_up_zone;
  MarkerData marker_in_drop_off_zone;
  
  marker_in_pick_up_zone.pos_x = -3;
  marker_in_pick_up_zone.pos_y = 2;
  marker_in_pick_up_zone.pos_z = 0.1;
  marker_in_pick_up_zone.or_x = 0;
  marker_in_pick_up_zone.or_y = 0;
  marker_in_pick_up_zone.or_z = 0;
  marker_in_pick_up_zone.or_w = 1.0;
  
  marker_in_drop_off_zone.pos_x = -4;
  marker_in_drop_off_zone.pos_y = -4;
  marker_in_drop_off_zone.pos_z = 0.1;
  marker_in_drop_off_zone.or_x = 0;
  marker_in_drop_off_zone.or_y = 0;
  marker_in_drop_off_zone.or_z = 0;
  marker_in_drop_off_zone.or_w = 1.0;
  

  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "add_markers";
  marker.id = 0;
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  // Set marker duration:
  marker.lifetime = ros::Duration();
  
  while (ros::ok())
  {

    marker.header.stamp = ros::Time::now();

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = marker_in_pick_up_zone.pos_x;
    marker.pose.position.y = marker_in_pick_up_zone.pos_y;
    marker.pose.position.z = marker_in_pick_up_zone.pos_z;
    marker.pose.orientation.x = marker_in_pick_up_zone.or_x;
    marker.pose.orientation.y = marker_in_pick_up_zone.or_y;
    marker.pose.orientation.z = marker_in_pick_up_zone.or_z;
    marker.pose.orientation.w = marker_in_pick_up_zone.or_w;
    marker_pub.publish(marker);
	ros::Duration(5.0).sleep();
    
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);
    ros::Duration(5.0).sleep();
    
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = marker_in_drop_off_zone.pos_x;
    marker.pose.position.y = marker_in_drop_off_zone.pos_y;
    marker.pose.position.z = marker_in_drop_off_zone.pos_z;
    marker.pose.orientation.x = marker_in_drop_off_zone.or_x;
    marker.pose.orientation.y = marker_in_drop_off_zone.or_y;
    marker.pose.orientation.z = marker_in_drop_off_zone.or_z;
    marker.pose.orientation.w = marker_in_drop_off_zone.or_w;
    marker.action = visualization_msgs::Marker::ADD;
    marker.header.stamp = ros::Time::now();
    marker_pub.publish(marker);
    ros::Duration(5.0).sleep();
    
    marker.action = visualization_msgs::Marker::DELETE;
    marker.header.stamp = ros::Time::now();
    marker_pub.publish(marker);
    ros::Duration(5.0).sleep();
    
  }
}