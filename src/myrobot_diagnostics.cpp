/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehoon Lim (Darby) */

#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/CameraInfo.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <turtlebot3_msgs/SensorState.h>
#include <turtlebot3_msgs/VersionInfo.h>

#define SOFTWARE_VERSION "1.0.0"
#define FIRMWARE_VERSION "1.2.0"
#define HARDWARE_VERSION "1.0.0"

ros::Publisher tb3_diagnostics_pub;
diagnostic_msgs::DiagnosticArray tb3_diagnostics;

diagnostic_msgs::DiagnosticStatus imu_state;
diagnostic_msgs::DiagnosticStatus motor_state;
diagnostic_msgs::DiagnosticStatus LDS_state;
diagnostic_msgs::DiagnosticStatus battery_state;
diagnostic_msgs::DiagnosticStatus button_state;
diagnostic_msgs::DiagnosticStatus temperatur_state;
diagnostic_msgs::DiagnosticStatus camera_state;

void setDiagnosisMsg( diagnostic_msgs::DiagnosticStatus* diag, uint8_t level, std::string name, std::string message, std::string hardware_id )
{
    diag->level = level;
    diag->name = name;
    diag->message = message;
    diag->hardware_id = hardware_id;
}

void setIMUDiagnosis( uint8_t level, std::string message )
{
    setDiagnosisMsg( &imu_state, level, "IMU Sensor", message, "MPU9250" );
}

void setMotorDiagnosis( uint8_t level, std::string message )
{
    setDiagnosisMsg( &motor_state, level, "Actuator", message, "DYNAMIXEL X" );
}

void setBatteryDiagnosis( uint8_t level, std::string message )
{
    setDiagnosisMsg( &battery_state, level, "Power System", message, "Battery" );
}

void setLDSDiagnosis( uint8_t level, std::string message )
{
    setDiagnosisMsg( &LDS_state, level, "Lidar Sensor", message, "HLS-LFCD-LDS" );
}

void setButtonDiagnosis( uint8_t level, std::string message )
{
    setDiagnosisMsg( &button_state, level, "Analog Button", message, "OpenCR Button" );
}

void setTemperatureDiagnosis( uint8_t level, std::string message )
{
    setDiagnosisMsg( &temperatur_state, level, "RaspberryPi Temperature", message, "RaspberryPi" );
}

void setCameraDiagnosis( uint8_t level, std::string message )
{
    setDiagnosisMsg( &camera_state, level, "RaspberryPi Camera", message, "RaspberryPi Camera" );
}

void imuMsgCallback( const sensor_msgs::Imu::ConstPtr& msg )
{
    setIMUDiagnosis( diagnostic_msgs::DiagnosticStatus::OK, "Good Condition" );
}

void LDSMsgCallback( const sensor_msgs::LaserScan::ConstPtr& msg )
{
    setLDSDiagnosis( diagnostic_msgs::DiagnosticStatus::OK, "Good Condition" );
}

void sensorStateMsgCallback( const turtlebot3_msgs::SensorState::ConstPtr& msg )
{
    if ( msg->battery > 11.0 )
        setBatteryDiagnosis( diagnostic_msgs::DiagnosticStatus::OK, "Good Condition" );
    else
        setBatteryDiagnosis( diagnostic_msgs::DiagnosticStatus::WARN, "Charge!!! Charge!!!" );

    if ( msg->button == turtlebot3_msgs::SensorState::BUTTON0 )
        setButtonDiagnosis( diagnostic_msgs::DiagnosticStatus::OK, "BUTTON 0 IS PUSHED" );
    else if ( msg->button == turtlebot3_msgs::SensorState::BUTTON1 )
        setButtonDiagnosis( diagnostic_msgs::DiagnosticStatus::OK, "BUTTON 1 IS PUSHED" );
    else
        setButtonDiagnosis( diagnostic_msgs::DiagnosticStatus::OK, "Pushed Nothing" );

    if ( msg->torque == true )
        setMotorDiagnosis( diagnostic_msgs::DiagnosticStatus::OK, "Torque ON" );
    else
        setMotorDiagnosis( diagnostic_msgs::DiagnosticStatus::WARN, "Torque OFF" );
}

void versionMsgCallback( const turtlebot3_msgs::VersionInfo::ConstPtr& msg )
{
    static bool check_version = false;

    if ( check_version == false )
    {
        if ( std::string( msg->software ) != std::string( SOFTWARE_VERSION ) )
            ROS_WARN( "Check turtlebot3 repository and Update your software!!" );

        if ( std::string( msg->hardware ) != std::string( HARDWARE_VERSION ) )
            ROS_WARN( "Check turtlebot3 wiki page and Update your hardware!!" );

        if ( std::string( msg->firmware ) != std::string( FIRMWARE_VERSION ) )
            ROS_WARN( "Check OpenCR update and change your firmware!!" );

        check_version = true;
    }
}

void TemperatureCallback( const sensor_msgs::Temperature::ConstPtr& msg )
{
    if ( msg->temperature > 0 && msg->temperature < 70 )
    {
        setTemperatureDiagnosis( diagnostic_msgs::DiagnosticStatus::OK, "Good Condition" );
    }
    else if ( msg->temperature <= 0 )
    {
        setTemperatureDiagnosis( diagnostic_msgs::DiagnosticStatus::WARN, "Too Cold" );
    }
    else
    {
        setTemperatureDiagnosis( diagnostic_msgs::DiagnosticStatus::ERROR, "Too Hot!!!" );
    }
}

void CameraCallback( const sensor_msgs::CameraInfo::ConstPtr& msg )
{
    if ( msg->width > 0 && msg->height > 0 )
    {
        setCameraDiagnosis( diagnostic_msgs::DiagnosticStatus::OK, "Good Condition" );
    }
    else
    {
        setCameraDiagnosis( diagnostic_msgs::DiagnosticStatus::WARN, "No Image" );
    }
}

void msgPub()
{
    tb3_diagnostics.header.stamp = ros::Time::now();

    tb3_diagnostics.status.clear();
    tb3_diagnostics.status.push_back( imu_state );
    tb3_diagnostics.status.push_back( motor_state );
    tb3_diagnostics.status.push_back( LDS_state );
    tb3_diagnostics.status.push_back( battery_state );
    tb3_diagnostics.status.push_back( button_state );
    tb3_diagnostics.status.push_back( temperatur_state );
    tb3_diagnostics.status.push_back( camera_state );

    tb3_diagnostics_pub.publish( tb3_diagnostics );
}

int main( int argc, char** argv )
{
    ros::init( argc, argv, "turtlebot3_diagnostic" );
    ros::NodeHandle nh;

    tb3_diagnostics_pub = nh.advertise< diagnostic_msgs::DiagnosticArray >( "diagnostics", 10 );

    ros::Subscriber imu = nh.subscribe( "imu", 10, imuMsgCallback );
    ros::Subscriber lds = nh.subscribe( "scan", 10, LDSMsgCallback );
    ros::Subscriber tb3_sensor = nh.subscribe( "sensor_state", 10, sensorStateMsgCallback );
    ros::Subscriber version = nh.subscribe( "version_info", 10, versionMsgCallback );
    ros::Subscriber tem = nh.subscribe( "raspi_temperature", 10, TemperatureCallback );
    ros::Subscriber cam = nh.subscribe( "raspicam_node/camera_info", 10, CameraCallback );

    ros::Rate loop_rate( 1 );

    while ( ros::ok() )
    {
        msgPub();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
