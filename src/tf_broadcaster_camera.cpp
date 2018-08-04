#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main( int argc, char* argv[] )
{
    ros::init( argc, argv, "camera_broadcaster" );
    ros::NodeHandle nh;

    tf::TransformBroadcaster br;
    tf::Transform transform;

    ros::Rate rate( 10.0 );

    while ( nh.ok() )
    {
        transform.setOrigin( tf::Vector3( 0.05, 0.0, -0.15 ) );
        transform.setRotation( tf::Quaternion( 0, 0, 0, 1 ) );
        br.sendTransform( tf::StampedTransform( transform, ros::Time::now(), "raspicam", "odom" ) );

        rate.sleep();
    }

    return 0;
}
