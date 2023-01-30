#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

bool object = false;
int main(int argc, char** argv){
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    
    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;

    sleep(10);
    visualization_msgs::Marker pickup;
    visualization_msgs::Marker dropoff;

    // Set the frame ID and timestamp. See the TF tutorials for infromation on these.
    pickup.header.frame_id = "/map";
    pickup.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker. This servers to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    pickup.ns = "add_markers";
    pickup.id = 0;

    // Set the marker type. Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    pickup.type = shape;

    // Set the marker action. Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    pickup.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker. This is a full 6DOF pose relative to the frame/time specified in the header
    pickup.pose.position.x = -6.9;
    pickup.pose.position.y = 5.73;
    pickup.pose.position.z = 0;
    pickup.pose.orientation.x = 0.0;
    pickup.pose.orientation.y = 0.0;
    pickup.pose.orientation.z = 0.0;
    pickup.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 0.5x0.5x0.5 here means 0.5m on a side
    pickup.scale.x = 0.5;
    pickup.scale.y = 0.5;
    pickup.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    pickup.color.r = 0.0f;
    pickup.color.g = 0.0f;
    pickup.color.b = 1.0f;
    pickup.color.a = 1.0;

    pickup.lifetime = ros::Duration();
    if (object == false){
      marker_pub.publish(pickup);
      object = true;
    }
    sleep(10);
    
    if (object == true){
      pickup.action = visualization_msgs::Marker::DELETE;
      marker_pub.publish(pickup);
    }
    
    // Set the frame ID and timestamp. See the TF tutorials for infromation on these.
    dropoff.header.frame_id = "/map";
    dropoff.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker. This servers to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    dropoff.ns = "add_markers";
    dropoff.id = 1;

    // Set the marker type. Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    dropoff.type = shape;

    // Set the marker action. Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    dropoff.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker. This is a full 6DOF pose relative to the frame/time specified in the header
    dropoff.pose.position.x = -2.57;
    dropoff.pose.position.y = 2.89;
    dropoff.pose.position.z = 0;
    dropoff.pose.orientation.x = 0.0;
    dropoff.pose.orientation.y = 0.0;
    dropoff.pose.orientation.z = 0.0;
    dropoff.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 0.5x0.5x0.5 here means 0.5m on a side
    dropoff.scale.x = 0.5;
    dropoff.scale.y = 0.5;
    dropoff.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    dropoff.color.r = 0.0f;
    dropoff.color.g = 0.0f;
    dropoff.color.b = 1.0f;
    dropoff.color.a = 1.0;

    dropoff.lifetime = ros::Duration();

    marker_pub.publish(dropoff);

    ros::spin();
    return 0;
}
