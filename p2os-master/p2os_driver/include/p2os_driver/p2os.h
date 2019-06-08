/*
 *  P2OS for ROS
 *  Copyright (C) 2009
 *     David Feil-Seifer, Brian Gerkey, Kasper Stoy,
 *      Richard Vaughan, & Andrew Howard
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef _P2OSDEVICE_H
#define _P2OSDEVICE_H

#include <pthread.h>
#include <sys/time.h>
#include <iostream>
#include <string.h>

#include <p2os_driver/packet.h>
#include <p2os_driver/robot_params.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <p2os_msgs/BatteryState.h>
#include <p2os_msgs/MotorState.h>
#include <p2os_msgs/GripperState.h>
#include <p2os_msgs/SonarArray.h>
#include <p2os_msgs/DIO.h>
#include <p2os_msgs/AIO.h>

#include <diagnostic_updater/publisher.h>
#include <diagnostic_updater/diagnostic_updater.h>

/*! \brief Container struct.
 *  
 *  Create a struct that holds the 
 *  Robot's sensors.
 */
typedef struct ros_p2os_data
{
    //! Provides the position of the robot
    nav_msgs::Odometry  position;
    //! Provides the battery voltage
    p2os_msgs::BatteryState batt;
    //! Provides the state of the motors (enabled or disabled)
    p2os_msgs::MotorState motors;
    //! Provides the state of the gripper
    p2os_msgs::GripperState gripper;
    //! Container for sonar data
    p2os_msgs::SonarArray sonar;
    //! Digital In/Out
    p2os_msgs::DIO dio;
    //! Analog In/Out
    p2os_msgs::AIO aio;
    //! Transformed odometry frame.
    geometry_msgs::TransformStamped odom_trans;
} ros_p2os_data_t;

// this is here because we need the above typedef's before including it.
#include "sip.h"
#include "kinecalc.h"

#include "p2os_ptz.h"

class SIP;

// Forward declaration of the KineCalc_Base class declared in kinecalc_base.h
//class KineCalc;


class P2OSNode
{
    /*! \brief P2OS robot driver node.
     *
     *  This class contains, essentially, the main means of communication
     *  between the robot and ROS.
     */
  public:
    P2OSNode( ros::NodeHandle n );
    virtual ~P2OSNode();

  public:
    //! Setup the robot for use. Communicates with the robot directly.
    int Setup();
    //! Prepare for shutdown.
    int Shutdown();

    int SendReceive(P2OSPacket* pkt, bool publish_data = true );

    void updateDiagnostics();

    void ResetRawPositions();
    void ToggleSonarPower(unsigned char val);
    void ToggleMotorPower(unsigned char val);
    void StandardSIPPutData(ros::Time ts);

    inline double TicksToDegrees (int joint, unsigned char ticks);
    inline unsigned char DegreesToTicks (int joint, double degrees);
    inline double TicksToRadians (int joint, unsigned char ticks);
    inline unsigned char RadiansToTicks (int joint, double rads);
    inline double RadsPerSectoSecsPerTick (int joint, double speed);
    inline double SecsPerTicktoRadsPerSec (int joint, double secs);

    void SendPulse (void);
    //void spin();
    void check_and_set_vel();
    void cmdvel_cb( const geometry_msgs::TwistConstPtr &);

    void check_and_set_motor_state();
    void cmdmotor_state( const p2os_msgs::MotorStateConstPtr &);

    void check_and_set_gripper_state();
    void gripperCallback(const p2os_msgs::GripperStateConstPtr &msg);
    double get_pulse() {return pulse;}

		// diagnostic messages
		void check_voltage( diagnostic_updater::DiagnosticStatusWrapper &stat );
		void check_stall( diagnostic_updater::DiagnosticStatusWrapper &stat );

  protected:
    //! Node Handler used for publication of data.
    ros::NodeHandle n;
    //! Node Handler used for private data publication.
    ros::NodeHandle nh_private;
 
    diagnostic_updater::Updater diagnostic_;

    diagnostic_updater::DiagnosedPublisher<p2os_msgs::BatteryState> batt_pub_;
    ros::Publisher mstate_pub_, grip_state_pub_,
      ptz_state_pub_, sonar_pub_, aio_pub_, dio_pub_;
    ros::Publisher pose_pub_;
    ros::Subscriber cmdvel_sub_, cmdmstate_sub_, gripper_sub_, ptz_cmd_sub_;

    tf::TransformBroadcaster odom_broadcaster;
    ros::Time veltime;

    SIP* sippacket;
    std::string psos_serial_port;
    std::string psos_tcp_host;
    std::string odom_frame_id;
    std::string base_link_frame_id;
    int         psos_fd;
    bool        psos_use_tcp;
    int         psos_tcp_port;
    bool        vel_dirty, motor_dirty;
    bool        gripper_dirty_;
    int         param_idx;
    // PID settings
    int rot_kp, rot_kv, rot_ki, trans_kp, trans_kv, trans_ki;

    //! Stall I hit a wall?
    int bumpstall; // should we change the bumper-stall behavior?
    //! Use Joystick?
    int joystick;
    //! Control wheel velocities individually?
    int direct_wheel_vel_control;
    int radio_modemp;

    //! Maximum motor speed in Meters per second.
    int motor_max_speed;
    //! Maximum turn speed in radians per second.
    int motor_max_turnspeed;
    //! Maximum translational acceleration in Meters per second per second.
    short motor_max_trans_accel;
    //! Minimum translational acceleration in Meters per second per second.
    short motor_max_trans_decel;
    //! Maximum rotational acceleration in radians per second per second.
    short motor_max_rot_accel;
    //! Minimum rotational acceleration in Meters per second per second.
    short motor_max_rot_decel;
    //! Pulse time
    double pulse;
    double desired_freq;
    //! Last time the node received or sent a pulse.
    double lastPulseTime;
    //! Use the sonar array?
    bool use_sonar_;

    P2OSPtz ptz_;

  public:
    //! Command Velocity subscriber
    geometry_msgs::Twist cmdvel_;
    //! Motor state publisher
    p2os_msgs::MotorState  cmdmotor_state_;
    //! Gripper state publisher
    p2os_msgs::GripperState gripper_state_;
    //! sensor data container
    ros_p2os_data_t p2os_data;
};

#endif
