#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

bool initialize_marker(visualization_msgs::Marker& marker_in);
bool set_marker_position_and_orientation(visualization_msgs::Marker& marker_in,
                                         const float pos_x, const float pos_y, const float pos_z,
                                         const float or_x, const float or_y, const float or_z, const float or_w);

bool set_marker_state(ros::Publisher& marker_pub, visualization_msgs::Marker& marker_in, uint8_t state_in, float duration_in);

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
  marker_in_pick_up_zone = {-3, 2, 0.1, 0, 0, 0, 1.0};
  marker_in_drop_off_zone = {-4, -4, 0.1, 0, 0, 0, 1.0};

  visualization_msgs::Marker marker;
  initialize_marker(marker);
  
  while (ros::ok())
  {
	// Show marker in pick-up zone
    set_marker_position_and_orientation(marker, marker_in_pick_up_zone.pos_x, marker_in_pick_up_zone.pos_y,
                                        marker_in_pick_up_zone.pos_z, marker_in_pick_up_zone.or_x, marker_in_pick_up_zone.or_y,
                                        marker_in_pick_up_zone.or_z, marker_in_pick_up_zone.or_w);
    set_marker_state(marker_pub, marker, visualization_msgs::Marker::ADD, 5.0);
    
    // Hide marker in pick-up zone (is carried now):
    set_marker_state(marker_pub, marker, visualization_msgs::Marker::DELETE, 5.0);
    
    // Show marker in drop-off zone
    set_marker_position_and_orientation(marker, marker_in_drop_off_zone.pos_x, marker_in_drop_off_zone.pos_y,
                                        marker_in_drop_off_zone.pos_z, marker_in_drop_off_zone.or_x, marker_in_drop_off_zone.or_y,
                                        marker_in_drop_off_zone.or_z, marker_in_drop_off_zone.or_w);
	set_marker_state(marker_pub, marker, visualization_msgs::Marker::ADD, 5.0);
    
    // Hide marker in drop-off zone:
	set_marker_state(marker_pub, marker, visualization_msgs::Marker::DELETE, 5.0);
  }
}

bool initialize_marker(visualization_msgs::Marker& marker_in) {
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker_in.header.frame_id = "map";
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker_in.ns = "add_markers";
  marker_in.id = 0;
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker_in.type = shape;
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker_in.scale.x = 0.1;
  marker_in.scale.y = 0.1;
  marker_in.scale.z = 0.1;
  // Set the color -- be sure to set alpha to something non-zero!
  marker_in.color.r = 1.0f;
  marker_in.color.g = 0.0f;
  marker_in.color.b = 0.0f;
  marker_in.color.a = 1.0;
  // Set marker duration:
  marker_in.lifetime = ros::Duration();
}

bool set_marker_position_and_orientation(visualization_msgs::Marker& marker_in,
                                         const float pos_x, const float pos_y, const float pos_z,
                                         const float or_x, const float or_y, const float or_z, const float or_w) {
	marker_in.pose.position.x = pos_x;
    marker_in.pose.position.y = pos_y;
    marker_in.pose.position.z = pos_z;
    marker_in.pose.orientation.x = or_x;
    marker_in.pose.orientation.y = or_y;
    marker_in.pose.orientation.z = or_z;
    marker_in.pose.orientation.w = or_w;
}

bool set_marker_state(ros::Publisher& marker_pub, visualization_msgs::Marker& marker_in, uint8_t state_in, float duration_in = 5.0) {
	marker_in.header.stamp = ros::Time::now();
    marker_in.action = state_in;
    marker_pub.publish(marker_in);
    ros::Duration(duration_in).sleep();
}