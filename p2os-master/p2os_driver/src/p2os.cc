/*
 *  P2OS for ROS
 *  Copyright (C) 2009
 *     Hunter Allen, David Feil-Seifer, Brian Gerkey, Kasper Stoy,
 *     Richard Vaughan, & Andrew Howard
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
#include <termios.h>
#include <fcntl.h>
#include <string.h>
 
#include <ros/ros.h>
#include <p2os_driver/p2os.h>

P2OSNode::P2OSNode(ros::NodeHandle nh) :
	n(nh), gripper_dirty_(false),
	batt_pub_(n.advertise<p2os_msgs::BatteryState>("battery_state",1000),
			  diagnostic_,
			  diagnostic_updater::FrequencyStatusParam(&desired_freq, &desired_freq, 0.1),
			  diagnostic_updater::TimeStampStatusParam()),
	ptz_(this)
{
	/*! \brief Constructor for P2OS node.
	 *   
	 *  This brings up the P2OS system for ROS operation.
	 */
    
	// Use sonar
	ros::NodeHandle n_private("~");
	n_private.param(std::string("odom_frame_id"), odom_frame_id,std::string("odom"));
	n_private.param(std::string("base_link_frame_id"), base_link_frame_id,std::string("base_link"));
	n_private.param("use_sonar", use_sonar_, false);
    
	// read in config options
	// bumpstall
	n_private.param("bumpstall", bumpstall, -1);
	// pulse
	n_private.param("pulse", pulse, -1.0);
	// rot_kp
	n_private.param("rot_kp", rot_kp, -1);
	// rot_kv
	n_private.param("rot_kv", rot_kv, -1);
	// rot_ki
	n_private.param("rot_ki", rot_ki, -1);
	// trans_kp
	n_private.param("trans_kp", trans_kp, -1);
	// trans_kv
	n_private.param("trans_kv", trans_kv, -1);
	// trans_ki
	n_private.param("trans_ki", trans_ki, -1);
	// !!! port !!!
	std::string def = DEFAULT_P2OS_PORT;
	n_private.param("port", psos_serial_port, def);
	ROS_INFO("using serial port: [%s]", psos_serial_port.c_str());
	n_private.param("use_tcp", psos_use_tcp, false);
	std::string host = DEFAULT_P2OS_TCP_REMOTE_HOST;
	n_private.param("tcp_remote_host", psos_tcp_host, host);
	n_private.param("tcp_remote_port", psos_tcp_port, DEFAULT_P2OS_TCP_REMOTE_PORT);
	// radio
	n_private.param("radio", radio_modemp, 0);
	// joystick
	n_private.param("joystick", joystick, 0);
	// direct_wheel_vel_control
	n_private.param("direct_wheel_vel_control", direct_wheel_vel_control, 0);
	// max xpeed
	double spd;
	n_private.param("max_xspeed", spd, MOTOR_DEF_MAX_SPEED);
	motor_max_speed = (int)rint(1e3*spd);
	// max_yawspeed
	n_private.param("max_yawspeed", spd, MOTOR_DEF_MAX_TURNSPEED);
	motor_max_turnspeed = (short)rint(RTOD(spd));
	// max_xaccel
	n_private.param("max_xaccel", spd, 0.0);
	motor_max_trans_accel = (short)rint(1e3*spd);
	// max_xdecel
	n_private.param("max_xdecel", spd, 0.0);
	motor_max_trans_decel = (short)rint(1e3*spd);
	// max_yawaccel
	n_private.param("max_yawaccel", spd, 0.0);
	motor_max_rot_accel = (short)rint(RTOD(spd));
	// max_yawdecel
	n_private.param("max_yawdecel", spd, 0.0);
	motor_max_rot_decel = (short)rint(RTOD(spd));
    
	desired_freq = 10;
    
	// advertise services
	pose_pub_ = n.advertise<nav_msgs::Odometry>("pose", 1000);
	//advertise topics
	mstate_pub_ = n.advertise<p2os_msgs::MotorState>("motor_state",1000);
	grip_state_pub_ = n.advertise<p2os_msgs::GripperState>("gripper_state",1000);
	ptz_state_pub_ = n.advertise<p2os_msgs::PTZState>("ptz_state",1000);
	sonar_pub_ = n.advertise<p2os_msgs::SonarArray>("sonar", 1000);
	aio_pub_ = n.advertise<p2os_msgs::AIO>("aio", 1000);
	dio_pub_ = n.advertise<p2os_msgs::DIO>("dio", 1000);
    
	// subscribe to services
	cmdvel_sub_ = n.subscribe("cmd_vel", 1, &P2OSNode::cmdvel_cb, this);
	cmdmstate_sub_ = n.subscribe("cmd_motor_state", 1, &P2OSNode::cmdmotor_state,
								 this);
	gripper_sub_ = n.subscribe("gripper_control", 1, &P2OSNode::gripperCallback,
							   this);
	ptz_cmd_sub_ = n.subscribe("ptz_control", 1, &P2OSPtz::callback, &ptz_);
    
	veltime = ros::Time::now();
    
	// add diagnostic functions
	diagnostic_.add("Motor Stall", this, &P2OSNode::check_stall);
	diagnostic_.add("Battery Voltage", this, &P2OSNode::check_voltage);
    
	// initialize robot parameters (player legacy)
	initialize_robot_params();
}

P2OSNode::~P2OSNode() {/** Destructor **/}

void P2OSNode::cmdmotor_state(const p2os_msgs::MotorStateConstPtr & msg)
{
	motor_dirty = true;
	cmdmotor_state_ = *msg;
}

void P2OSNode::check_and_set_motor_state()
{
	if(!motor_dirty) return;
	motor_dirty = false;
    
	unsigned char val = (unsigned char) cmdmotor_state_.state;
	unsigned char command[4];
	P2OSPacket packet;
	command[0] = ENABLE;
	command[1] = ARGINT;
	command[2] = val;
	command[3] = 0;
	packet.Build(command,4);
    
	// Store the current motor state so that we can set it back
	p2os_data.motors.state = cmdmotor_state_.state;
	SendReceive(&packet,false);
}

void P2OSNode::check_and_set_gripper_state()
{
	if(!gripper_dirty_) return;
	gripper_dirty_ = false;
    
	// Send the gripper command
	unsigned char grip_val = (unsigned char) gripper_state_.grip.state;
	unsigned char grip_command[4];

	P2OSPacket grip_packet;
	grip_command[0] = GRIPPER;
	grip_command[1] = ARGINT;
	grip_command[2] = grip_val;
	grip_command[3] = 0;
	grip_packet.Build(grip_command,4);
	SendReceive(&grip_packet, false);
    
	// Send the lift command
	unsigned char lift_val = (unsigned char) gripper_state_.lift.state;
	unsigned char lift_command[4];

	P2OSPacket lift_packet;
	lift_command[0] = GRIPPER;
	lift_command[1] = ARGINT;
	lift_command[2] = lift_val;
	lift_command[3] = 0;
	lift_packet.Build(lift_command,4);

	SendReceive(&lift_packet,false);
}

void P2OSNode::cmdvel_cb(const geometry_msgs::TwistConstPtr &msg)
{
	if(fabs(msg->linear.x - cmdvel_.linear.x) > 0.01 || fabs(msg->angular.z - cmdvel_.angular.z) > 0.01) {
		veltime = ros::Time::now();
		ROS_DEBUG("new speed: [%0.2f,%0.2f](%0.3f)", msg->linear.x*1e3, msg->angular.z, veltime.toSec());
		vel_dirty = true;
		cmdvel_ = *msg;
	} else {
		ros::Duration veldur = ros::Time::now() - veltime;
		if(veldur.toSec() > 2.0 && ((fabs(cmdvel_.linear.x) > 0.01) || (fabs(cmdvel_.angular.z) > 0.01))) {
			ROS_DEBUG("maintaining old speed: %0.3f|%0.3f", veltime.toSec(), ros::Time::now().toSec());
			vel_dirty = true;
			veltime = ros::Time::now();
		}
	}
    
}

void P2OSNode::check_and_set_vel()
{
	if(!vel_dirty) return;
    
	ROS_DEBUG("setting vel: [%0.2f,%0.2f]",cmdvel_.linear.x,cmdvel_.angular.z);
	vel_dirty = false;
    
	unsigned short absSpeedDemand, absturnRateDemand;
	unsigned char motorcommand[4];
	P2OSPacket motorpacket;
    
	int vx = (int) (cmdvel_.linear.x*1e3);
	int va = (int) rint(RTOD(cmdvel_.angular.z));
    
	// non-direct wheel control
	motorcommand[0] = VEL;
	motorcommand[1] = (vx >= 0) ? ARGINT : ARGNINT;
    
	absSpeedDemand = (unsigned short) abs(vx);
    
	if(absSpeedDemand <= this->motor_max_speed) {
		motorcommand[2] = absSpeedDemand & 0x00FF;
		motorcommand[3] = (absSpeedDemand & 0xFF00) >> 8;
	} else {
		ROS_WARN("speed demand thresholded! (true: %u, max: %u)", absSpeedDemand, motor_max_speed);
		motorcommand[2] = motor_max_speed & 0x00FF;
		motorcommand[3] = (motor_max_speed & 0xFF00) >> 8;
	}
    
	motorpacket.Build(motorcommand, 4);
	SendReceive(&motorpacket);
        
	motorcommand[0] = RVEL;
	motorcommand[1] = (va >= 0) ? ARGINT : ARGNINT;
    
	absturnRateDemand = (unsigned short) (va >= 0) ? va : (-1 * va);

	if(absturnRateDemand <= motor_max_turnspeed) {
		motorcommand[2] = absturnRateDemand & 0x00FF;
		motorcommand[3] = (absturnRateDemand & 0xFF00) >> 8;
	} else {
		ROS_WARN("Turn rate demand threshholded!");
		motorcommand[2] = this->motor_max_turnspeed & 0x00FF;
		motorcommand[3] = (this->motor_max_turnspeed & 0xFF00) >> 8;
	}
  
	motorpacket.Build(motorcommand,4);
	SendReceive(&motorpacket);
}

void P2OSNode::gripperCallback(const p2os_msgs::GripperStateConstPtr &msg)
{
	gripper_dirty_ = true;
	gripper_state_ = *msg;
}

int P2OSNode::Setup()
{
	int i;
	int bauds[] = {B9600, B38400, B19200, B115200, B57600};
	int numbauds = sizeof(bauds);
	int currbaud = 0;
	sippacket = NULL;
	lastPulseTime = 0.0;
    
	struct termios term;
	unsigned char command;
	P2OSPacket packet, receivedpacket;
	int flags=0;
	bool sent_close = false;

	enum
	{
		NO_SYNC,
		AFTER_FIRST_SYNC,
		AFTER_SECOND_SYNC,
		READY
	} psos_state;
    
	psos_state = NO_SYNC;
    
	char name[20], type[20], subtype[20];
	int cnt;
    
    
	// use serial port
    
	ROS_INFO("P2OS connection opening serial port %s...",psos_serial_port.c_str());
    
	if((this->psos_fd = open(this->psos_serial_port.c_str(),
							 O_RDWR | O_SYNC | O_NONBLOCK, S_IRUSR | S_IWUSR)) < 0)
    {
		ROS_ERROR("P2OS::Setup():open():");
		return(1);
    }
    
	if(tcgetattr(this->psos_fd, &term) < 0)
    {
		ROS_ERROR("P2OS::Setup():tcgetattr():");
		close(this->psos_fd);
		this->psos_fd = -1;
		return(1);
    }
    
	cfmakeraw(&term);
	cfsetispeed(&term, bauds[currbaud]);
	cfsetospeed(&term, bauds[currbaud]);
    
	if(tcsetattr(this->psos_fd, TCSAFLUSH, &term) < 0)
    {
		ROS_ERROR("P2OS::Setup():tcsetattr():");
		close(this->psos_fd);
		this->psos_fd = -1;
		return(1);
    }
    
	if(tcflush(this->psos_fd, TCIOFLUSH) < 0)
    {
		ROS_ERROR("P2OS::Setup():tcflush():");
		close(this->psos_fd);
		this->psos_fd = -1;
		return(1);
    }
    
	if((flags = fcntl(this->psos_fd, F_GETFL)) < 0)
    {
		ROS_ERROR("P2OS::Setup():fcntl()");
		close(this->psos_fd);
		this->psos_fd = -1;
		return(1);
    }
	// Sync:
    
	int num_sync_attempts = 3;
	while(psos_state != READY)
    {
		switch(psos_state)
        {
        case NO_SYNC:
			command = SYNC0;
			packet.Build(&command, 1);
			packet.Send(this->psos_fd);
			usleep(P2OS_CYCLETIME_USEC);
			break;
        case AFTER_FIRST_SYNC:
			ROS_INFO("turning off NONBLOCK mode...");
			if(fcntl(this->psos_fd, F_SETFL, flags ^ O_NONBLOCK) < 0)
            {
				ROS_ERROR("P2OS::Setup():fcntl()");
				close(this->psos_fd);
				this->psos_fd = -1;
				return(1);
            }
			command = SYNC1;
			packet.Build(&command, 1);
			packet.Send(this->psos_fd);
			break;
        case AFTER_SECOND_SYNC:
			command = SYNC2;
			packet.Build(&command, 1);
			packet.Send(this->psos_fd);
			break;
        default:
			ROS_WARN("P2OS::Setup():shouldn't be here...");
			break;
        }
		usleep(P2OS_CYCLETIME_USEC);
        
		if(receivedpacket.Receive(this->psos_fd))
        {
			if((psos_state == NO_SYNC) && (num_sync_attempts >= 0))
            {
				num_sync_attempts--;
				usleep(P2OS_CYCLETIME_USEC);
				continue;
            }
			else
            {
				// couldn't connect; try different speed.
				if(++currbaud < numbauds)
                {
					cfsetispeed(&term, bauds[currbaud]);
					cfsetospeed(&term, bauds[currbaud]);
					if(tcsetattr(this->psos_fd, TCSAFLUSH, &term) < 0)
                    {
						ROS_ERROR("P2OS::Setup():tcsetattr():");
						close(this->psos_fd);
						this->psos_fd = -1;
						return(1);
                    }
                    
					if(tcflush(this->psos_fd, TCIOFLUSH) < 0)
                    {
						ROS_ERROR("P2OS::Setup():tcflush():");
						close(this->psos_fd);
						this->psos_fd = -1;
						return(1);
                    }
					num_sync_attempts = 3;
					continue;
                }
				else
                {
					// tried all speeds; bail
					break;
                }
            }
        }
		switch(receivedpacket.packet[3])
        {
        case SYNC0:
			ROS_INFO("SYNC0");
			psos_state = AFTER_FIRST_SYNC;
			break;
        case SYNC1:
			ROS_INFO("SYNC1");
			psos_state = AFTER_SECOND_SYNC;
			break;
        case SYNC2:
			ROS_INFO("SYNC2");
			psos_state = READY;
			break;
        default:
			// maybe P2OS is still running from last time.  let's try to CLOSE
			// and reconnect
			if(!sent_close)
            {
				ROS_DEBUG("sending CLOSE");
				command = CLOSE;
				packet.Build(&command, 1);
				packet.Send(this->psos_fd);
				sent_close = true;
				usleep(2*P2OS_CYCLETIME_USEC);
				tcflush(this->psos_fd,TCIFLUSH);
				psos_state = NO_SYNC;
            }
			break;
        }
		usleep(P2OS_CYCLETIME_USEC);
    }
	if(psos_state != READY)
    {
		if(this->psos_use_tcp)
			ROS_INFO("Couldn't synchronize with P2OS.\n"
					 "  Most likely because the robot is not connected %s %s",
					 this->psos_use_tcp ? "to the ethernet-serial bridge device " : "to the serial port",
					 this->psos_use_tcp ? this->psos_tcp_host.c_str() : this->psos_serial_port.c_str());
		close(this->psos_fd);
		this->psos_fd = -1;
		return(1);
    }
	cnt = 4;
	cnt += snprintf(name, sizeof(name), "%s", &receivedpacket.packet[cnt]);
	cnt++;
	cnt += snprintf(type, sizeof(type), "%s", &receivedpacket.packet[cnt]);
	cnt++;
	cnt += snprintf(subtype, sizeof(subtype), "%s", &receivedpacket.packet[cnt]);
	cnt++;
    
	std::string hwID = std::string(name) + ": " + std::string(type) + "/" + std::string(subtype);
	diagnostic_.setHardwareID(hwID);
    
	command = OPEN;
	packet.Build(&command, 1);
	packet.Send(this->psos_fd);
	usleep(P2OS_CYCLETIME_USEC);
	command = PULSE;
	packet.Build(&command, 1);
	packet.Send(this->psos_fd);
	usleep(P2OS_CYCLETIME_USEC);
    
	ROS_INFO("Done.\n   Connected to %s, a %s %s", name, type, subtype);
    
	// now, based on robot type, find the right set of parameters
	for(i=0;i<PLAYER_NUM_ROBOT_TYPES;i++)
    {
		if(!strcasecmp(PlayerRobotParams[i].Class.c_str(),type) &&
		   !strcasecmp(PlayerRobotParams[i].Subclass.c_str(),subtype))
        {
			param_idx = i;
			break;
        }
    }
	if(i == PLAYER_NUM_ROBOT_TYPES)
    {
		ROS_WARN("P2OS: Warning: couldn't find parameters for this robot; "
				 "using defaults");
		param_idx = 0;
    }
    
	//sleep(1);
    
	// first, receive a packet so we know we're connected.
	if(!sippacket)
    {
		sippacket = new SIP(param_idx);
		sippacket->odom_frame_id = odom_frame_id;
		sippacket->base_link_frame_id = base_link_frame_id;
    }
	/*
	  sippacket->x_offset = 0;
	  sippacket->y_offset = 0;
	  sippacket->angle_offset = 0;
  
	  SendReceive((P2OSPacket*)NULL,false);
	*/
	// turn off the sonars at first
	this->ToggleSonarPower(0);
	// if requested, set max accel/decel limits
	P2OSPacket accel_packet;
	unsigned char accel_command[4];
	if(this->motor_max_trans_accel > 0)
    {
		accel_command[0] = SETA;
		accel_command[1] = ARGINT;
		accel_command[2] = this->motor_max_trans_accel & 0x00FF;
		accel_command[3] = (this->motor_max_trans_accel & 0xFF00) >> 8;
		accel_packet.Build(accel_command, 4);
		this->SendReceive(&accel_packet,false);
    }
    
	if(this->motor_max_trans_decel < 0)
    {
		accel_command[0] = SETA;
		accel_command[1] = ARGNINT;
		accel_command[2] =  -1 *(this->motor_max_trans_decel) & 0x00FF;
		accel_command[3] =  (-1 *(this->motor_max_trans_decel) & 0xFF00) >> 8;
		accel_packet.Build(accel_command, 4);
		this->SendReceive(&accel_packet,false);
    }
	if(this->motor_max_rot_accel > 0)
    {
		accel_command[0] = SETRA;
		accel_command[1] = ARGINT;
		accel_command[2] = this->motor_max_rot_accel & 0x00FF;
		accel_command[3] = (this->motor_max_rot_accel & 0xFF00) >> 8;
		accel_packet.Build(accel_command, 4);
		this->SendReceive(&accel_packet,false);
    }
	if(this->motor_max_rot_decel < 0)
    {
		accel_command[0] = SETRA;
		accel_command[1] = ARGNINT;
		accel_command[2] = -1*(this->motor_max_rot_decel) & 0x00FF;
		accel_command[3] = (-1*(this->motor_max_rot_decel) & 0xFF00) >> 8;
		accel_packet.Build(accel_command, 4);
		this->SendReceive(&accel_packet,false);
    }
    
    
	// if requested, change PID settings
	P2OSPacket pid_packet;
	unsigned char pid_command[4];
	if(this->rot_kp >= 0)
    {
		pid_command[0] = ROTKP;
		pid_command[1] = ARGINT;
		pid_command[2] = this->rot_kp & 0x00FF;
		pid_command[3] = (this->rot_kp & 0xFF00) >> 8;
		pid_packet.Build(pid_command, 4);
		this->SendReceive(&pid_packet);
    }
	if(this->rot_kv >= 0)
    {
		pid_command[0] = ROTKV;
		pid_command[1] = ARGINT;
		pid_command[2] = this->rot_kv & 0x00FF;
		pid_command[3] = (this->rot_kv & 0xFF00) >> 8;
		pid_packet.Build(pid_command, 4);
		this->SendReceive(&pid_packet);
    }
	if(this->rot_ki >= 0)
    {
		pid_command[0] = ROTKI;
		pid_command[1] = ARGINT;
		pid_command[2] = this->rot_ki & 0x00FF;
		pid_command[3] = (this->rot_ki & 0xFF00) >> 8;
		pid_packet.Build(pid_command, 4);
		this->SendReceive(&pid_packet);
    }
	if(this->trans_kp >= 0)
    {
		pid_command[0] = TRANSKP;
		pid_command[1] = ARGINT;
		pid_command[2] = this->trans_kp & 0x00FF;
		pid_command[3] = (this->trans_kp & 0xFF00) >> 8;
		pid_packet.Build(pid_command, 4);
		this->SendReceive(&pid_packet);
    }
	if(this->trans_kv >= 0)
    {
		pid_command[0] = TRANSKV;
		pid_command[1] = ARGINT;
		pid_command[2] = this->trans_kv & 0x00FF;
		pid_command[3] = (this->trans_kv & 0xFF00) >> 8;
		pid_packet.Build(pid_command, 4);
		this->SendReceive(&pid_packet);
    }
	if(this->trans_ki >= 0)
    {
		pid_command[0] = TRANSKI;
		pid_command[1] = ARGINT;
		pid_command[2] = this->trans_ki & 0x00FF;
		pid_command[3] = (this->trans_ki & 0xFF00) >> 8;
		pid_packet.Build(pid_command, 4);
		this->SendReceive(&pid_packet);
    }
    
    
	// if requested, change bumper-stall behavior
	// 0 = don't stall
	// 1 = stall on front bumper contact
	// 2 = stall on rear bumper contact
	// 3 = stall on either bumper contact
	if(this->bumpstall >= 0)
    {
		if(this->bumpstall > 3)
			ROS_INFO("ignoring bumpstall value %d; should be 0, 1, 2, or 3",
					 this->bumpstall);
		else
        {
			ROS_INFO("setting bumpstall to %d", this->bumpstall);
			P2OSPacket bumpstall_packet;;
			unsigned char bumpstall_command[4];
			bumpstall_command[0] = BUMP_STALL;
			bumpstall_command[1] = ARGINT;
			bumpstall_command[2] = (unsigned char)this->bumpstall;
			bumpstall_command[3] = 0;
			bumpstall_packet.Build(bumpstall_command, 4);
			this->SendReceive(&bumpstall_packet,false);
        }
    }
    
	// Turn on the sonar
	if(use_sonar_) {
		this->ToggleSonarPower(1);
		ROS_DEBUG("Sonar array powered on.");
	}
	ptz_.setup();
    
	return(0);
}

int P2OSNode::Shutdown()
{
	unsigned char command[20],buffer[20];
	P2OSPacket packet;
    
	if (ptz_.isOn())
    {
		ptz_.shutdown();
    }
    
	memset(buffer,0,20);
    
	if(this->psos_fd == -1)
		return -1;
    
	command[0] = STOP;
	packet.Build(command, 1);
	packet.Send(this->psos_fd);
	usleep(P2OS_CYCLETIME_USEC);
    
	command[0] = CLOSE;
	packet.Build(command, 1);
	packet.Send(this->psos_fd);
	usleep(P2OS_CYCLETIME_USEC);
    
	close(this->psos_fd);
	this->psos_fd = -1;
	ROS_INFO("P2OS has been shutdown");
	delete this->sippacket;
	this->sippacket = NULL;
    
	return 0;
}


void
P2OSNode::StandardSIPPutData(ros::Time ts)
{
    
	p2os_data.position.header.stamp = ts;
	pose_pub_.publish(p2os_data.position);
	p2os_data.odom_trans.header.stamp = ts;
	odom_broadcaster.sendTransform(p2os_data.odom_trans);
    
	p2os_data.batt.header.stamp = ts;
	batt_pub_.publish(p2os_data.batt);
	mstate_pub_.publish(p2os_data.motors);
    
	// put sonar data
	p2os_data.sonar.header.stamp = ts;
	sonar_pub_.publish(p2os_data.sonar);
    
	// put aio data
	aio_pub_.publish(p2os_data.aio);
	// put dio data
	dio_pub_.publish(p2os_data.dio);
    
	// put gripper and lift data
	grip_state_pub_.publish(p2os_data.gripper);
	ptz_state_pub_.publish(ptz_.getCurrentState());
    
	// put bumper data
	// put compass data
    
}

/* send the packet, then receive and parse an SIP */
int P2OSNode::SendReceive(P2OSPacket* pkt, bool publish_data)
{
	P2OSPacket packet;
    
	if((this->psos_fd >= 0) && this->sippacket)
    {
		if(pkt)
			pkt->Send(this->psos_fd);
        
		/* receive a packet */
		pthread_testcancel();
		if(packet.Receive(this->psos_fd))
        {
			ROS_ERROR("RunPsosThread(): Receive errored");
			pthread_exit(NULL);
        }
        
		if(packet.packet[0] == 0xFA && packet.packet[1] == 0xFB &&
		   (packet.packet[3] == 0x30 || packet.packet[3] == 0x31 ||
			packet.packet[3] == 0x32 || packet.packet[3] == 0x33 ||
			packet.packet[3] == 0x34))
        {
            
			/* It is a server packet, so process it */
			this->sippacket->ParseStandard(&packet.packet[3]);
			this->sippacket->FillStandard(&(this->p2os_data));
            
			if(publish_data)
				this->StandardSIPPutData(packet.timestamp);
        }
		else if(packet.packet[0] == 0xFA && packet.packet[1] == 0xFB &&
				packet.packet[3] == SERAUX)
        {
			// This is an AUX serial packet
			if(ptz_.isOn())
            {
				int len = packet.packet[2] - 3;
				if (ptz_.cb_.gotPacket())
                {
					ROS_ERROR("PTZ got a message, but alread has the complete packet.");
                }
				else
                {
					for (int i=4; i < 4+len; ++i)
                    {
						ptz_.cb_.putOnBuf(packet.packet[i]);
                    }
                }
            }
        }
		else
        {
			ROS_ERROR("Received other packet!");
			packet.PrintHex();
        }
    }
    
	return(0);
}

void P2OSNode::updateDiagnostics()
{
	diagnostic_.update();
}

void P2OSNode::check_voltage(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	double voltage = sippacket->battery / 10.0;
	if(voltage < 11.0) stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "battery voltage critically low");
	else if (voltage < 11.75) stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "battery voltage getting low");
	else stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "battery voltage OK");
    
	stat.add("voltage", voltage);
}

void P2OSNode::check_stall(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	if(sippacket->lwstall||sippacket->rwstall) stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
															"wheel stalled");
	else stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "no wheel stall");
    
	stat.add("left wheel stall", sippacket->lwstall);
	stat.add("right wheel stall", sippacket->rwstall);
}

void P2OSNode::ResetRawPositions()
{
	P2OSPacket pkt;
	unsigned char p2oscommand[4];
    
	if(this->sippacket) {
		this->sippacket->rawxpos = 0;
		this->sippacket->rawypos = 0;
		this->sippacket->xpos = 0;
		this->sippacket->ypos = 0;
		p2oscommand[0] = SETO;
		p2oscommand[1] = ARGINT;
		pkt.Build(p2oscommand, 2);
		this->SendReceive(&pkt,false);
		ROS_INFO("resetting raw positions");
	}
}

/* toggle sonars on/off, according to val */
void P2OSNode::ToggleSonarPower(unsigned char val)
{
	unsigned char command[4];
	P2OSPacket packet;
    
	command[0] = SONAR;
	command[1] = ARGINT;
	command[2] = val;
	command[3] = 0;
	packet.Build(command, 4);
	SendReceive(&packet,false);
}

/*! \brief Toggle motors on/off.
 *  
 *  Turn on/off the robots in accordance to val.
 *  @param val Determines what state the motor should be. 1 = enabled, 0 = disabled.
 */
void P2OSNode::ToggleMotorPower(unsigned char val)
{
	unsigned char command[4];
	P2OSPacket packet;
	ROS_INFO("motor state: %d\n", p2os_data.motors.state);
	p2os_data.motors.state = (int) val;
	command[0] = ENABLE;
	command[1] = ARGINT;
	command[2] = val;
	command[3] = 0;
	packet.Build(command, 4);
	SendReceive(&packet,false);
}

/////////////////////////////////////////////////////
//  Actarray stuff
/////////////////////////////////////////////////////

// Ticks to degrees from the ARIA software
//! Convert ticks to degrees.
inline double P2OSNode::TicksToDegrees (int joint, unsigned char ticks)
{
	if ((joint < 0) || (joint >= sippacket->armNumJoints))
		return 0;
    
	double result;
	int pos = ticks - sippacket->armJoints[joint].centre;
	result = 90.0 / static_cast<double> (sippacket->armJoints[joint].ticksPer90);
	result = result * pos;
	if ((joint >= 0) && (joint <= 2))
		result = -result;
    
	return result;
}

// Degrees to ticks from the ARIA software
//! convert degrees to ticks
inline unsigned char P2OSNode::DegreesToTicks (int joint, double degrees)
{
	double val;
    
	if ((joint < 0) || (joint >= sippacket->armNumJoints))
		return 0;
    
	val = static_cast<double> (sippacket->armJoints[joint].ticksPer90) * degrees / 90.0;
	val = round (val);
	if ((joint >= 0) && (joint <= 2))
		val = -val;
	val += sippacket->armJoints[joint].centre;
    
	if (val < sippacket->armJoints[joint].min) return sippacket->armJoints[joint].min;
	else if (val > sippacket->armJoints[joint].max) return sippacket->armJoints[joint].max;
	else return static_cast<int> (round (val));
}

//! Convert ticks to radians
inline double P2OSNode::TicksToRadians (int joint, unsigned char ticks)
{
	double result = DTOR (TicksToDegrees (joint, ticks));
	return result;
}

//! Convert radians to ticks.
inline unsigned char P2OSNode::RadiansToTicks (int joint, double rads)
{
	unsigned char result = static_cast<unsigned char> (DegreesToTicks (joint, RTOD (rads)));
	return result;
}

//! Convert radians per second to radians per encoder tick.
inline double P2OSNode::RadsPerSectoSecsPerTick (int joint, double speed)
{
	double degs = RTOD (speed);
	double ticksPerDeg = static_cast<double> (sippacket->armJoints[joint].ticksPer90) / 90.0;
	double ticksPerSec = degs * ticksPerDeg;
	double secsPerTick = 1000.0f / ticksPerSec;
    
	if (secsPerTick > 127)
		return 127;
	else if (secsPerTick < 1)
		return 1;
	return secsPerTick;
}

//! Convert Seconds per encoder tick to radians per second.
inline double P2OSNode::SecsPerTicktoRadsPerSec (int joint, double msecs)
{
	double ticksPerSec = 1.0 / (static_cast<double> (msecs) / 1000.0);
	double ticksPerDeg = static_cast<double> (sippacket->armJoints[joint].ticksPer90) / 90.0;
	double degs = ticksPerSec / ticksPerDeg;
	double rads = DTOR (degs);
    
	return rads;
}

void P2OSNode::SendPulse (void)
{
	unsigned char command;
	P2OSPacket packet;
    
	command = PULSE;
	packet.Build(&command, 1);
	SendReceive(&packet);
}
