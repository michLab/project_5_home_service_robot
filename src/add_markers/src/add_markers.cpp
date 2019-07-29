#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <cmath>

#define DEBUG	false

class AddMarker {
  public:
    AddMarker();
    void init();
    void set_state(uint8_t state_in, float duration_in);
    void set_pick_up_loc_xyz(float x, float y, float z);
    void set_drop_off_loc_xyz(float x, float y, float z);
  	void set_marker_loc_xyz(float x, float y, float z);
  	bool is_at_pick_up_loc(float x, float y, float z);
  	bool is_at_drop_off_loc(float x, float y, float z);
  
private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  	float get_euclidean_distance(float x1, float y1, float z1, float x2, float y2, float z2);
    ros::NodeHandle n;
    ros::Publisher marker_pub;
    ros::Subscriber odom_sub;;
    visualization_msgs::Marker marker_msg;
    float pick_up_loc_xyz[3];
    float drop_off_loc_xyz[3];
  	bool is_carring_object;
};

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  AddMarker marker;
  marker.set_pick_up_loc_xyz(-3.0f, 2.0f, 0.1f);
  marker.set_drop_off_loc_xyz(-4.0f, -4.0f, 0.1f);
  // Display marker in pick-up location:
  marker.set_marker_loc_xyz(-3.0f, 2.0f, 0.1f);
  marker.set_state(visualization_msgs::Marker::ADD, 5.0);
  ROS_INFO("Init: show object in pick-up zone");

  ros::spin();
  
  return 0;
}

AddMarker::AddMarker() {
	init();
}

void AddMarker::init() {
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  odom_sub = n.subscribe("/odom", 10, &AddMarker::odomCallback, this);
  
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker_msg.header.frame_id = "map";
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker_msg.ns = "add_markers";
  marker_msg.id = 0;
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker_msg.type = shape;
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker_msg.scale.x = 0.2;
  marker_msg.scale.y = 0.2;
  marker_msg.scale.z = 0.2;
  // Set the color -- be sure to set alpha to something non-zero!
  marker_msg.color.r = 0.0f;
  marker_msg.color.g = 0.0f;
  marker_msg.color.b = 1.0f;
  marker_msg.color.a = 1.0;
  // Set marker duration:
  marker_msg.lifetime = ros::Duration();
  is_carring_object = false;
}

void AddMarker::set_pick_up_loc_xyz(float x, float y, float z) {
	pick_up_loc_xyz[0] = x;
  	pick_up_loc_xyz[1] = y;
  	pick_up_loc_xyz[2] = z;
}

void AddMarker::set_drop_off_loc_xyz(float x, float y, float z) {
  	drop_off_loc_xyz[0] = x;
  	drop_off_loc_xyz[1] = y;
  	drop_off_loc_xyz[2] = z;
}

void AddMarker::set_marker_loc_xyz(float x, float y, float z) {
	marker_msg.pose.position.x = x;
  	marker_msg.pose.position.y = y;
  	marker_msg.pose.position.z = z;
}

bool AddMarker::is_at_pick_up_loc(float x, float y, float z) {
  	//float dist = get_euclidean_distance(x, y, z, pick_up_loc_xyz[0], pick_up_loc_xyz[1], pick_up_loc_xyz[2]);
  	float dist = get_euclidean_distance(x, y, z, -2.0, 2.81f, pick_up_loc_xyz[2]);
  	if (DEBUG) ROS_INFO("Distance from pick-up zone = %f", dist);
    if ( dist < marker_msg.scale.x) {
      	return true;
    }
    else {
      	return false;
    }
}

bool AddMarker::is_at_drop_off_loc(float x, float y, float z) {
  	//float dist = get_euclidean_distance(x, y, z, drop_off_loc_xyz[0], drop_off_loc_xyz[1], drop_off_loc_xyz[2]);
  	float dist = get_euclidean_distance(x, y, z, 3.85, 2.0, drop_off_loc_xyz[2]);
  	if (DEBUG) ROS_INFO("Distance from drop-off zone = %f", dist);
    if (dist < marker_msg.scale.x) {
      	return true;
    }
    else {
      	return false;
    }
}

float AddMarker::get_euclidean_distance(float x1, float y1, float z1, float x2, float y2, float z2) {
  	float d = sqrt(pow(x1 - x2, 2.0f) + pow(y1 - y2, 2.0f)); 
	return d;
}

void AddMarker::set_state(uint8_t state_in, float duration_in = 5.0f) {
  	marker_msg.header.stamp = ros::Time::now();
    marker_msg.action = state_in;
    marker_pub.publish(marker_msg);
    //ros::Duration(duration_in).sleep();
}

void AddMarker::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
 	float robot_pos_x = msg->pose.pose.position.x;
  	float robot_pos_y = msg->pose.pose.position.y;
  	float robot_pos_z = msg->pose.pose.position.z;
  	if (DEBUG) ROS_INFO("Robot pose = %f, %f, %f", robot_pos_x, robot_pos_y, robot_pos_z);
  	
  	if (is_at_pick_up_loc(robot_pos_x, robot_pos_y, robot_pos_z)) {
    	set_state(visualization_msgs::Marker::DELETE);
      	is_carring_object = true;
      	ROS_INFO("Robot reaches pick-up zone, grab object");
    }
  	else if (is_at_drop_off_loc(robot_pos_x, robot_pos_y, robot_pos_z)) {
    	set_marker_loc_xyz(drop_off_loc_xyz[0], drop_off_loc_xyz[1], drop_off_loc_xyz[2]);
      	set_state(visualization_msgs::Marker::ADD);
      	is_carring_object = false;
      	ROS_INFO("Robot reaches drop-off zone, release object");
    }
  	else if (is_carring_object == false) {
        set_marker_loc_xyz(pick_up_loc_xyz[0], pick_up_loc_xyz[1], pick_up_loc_xyz[2]);
      	set_state(visualization_msgs::Marker::ADD);
      	ROS_INFO("Robot is travelling, show object in pick-up zone");
    }
}











