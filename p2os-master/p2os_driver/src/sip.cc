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


#include <stdio.h>
#include <limits.h>
#include <math.h>  /* rint(3) */
#include <sys/types.h>
#include <stdlib.h> /* for abs() */
#include <unistd.h>

#include <p2os_driver/sip.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <sstream>
#include <boost/assign/list_of.hpp>

void SIP::FillStandard(ros_p2os_data_t* data)
{
  ///////////////////////////////////////////////////////////////
  // odometry

  double px, py, pa;

  // initialize position to current offset
  px = this->x_offset / 1e3;
  py = this->y_offset / 1e3;
  // now transform current position by rotation if there is one
  // and add to offset
  if(this->angle_offset != 0)
  {
    double rot = DTOR(this->angle_offset);    // convert rotation to radians
    px +=  ((this->xpos/1e3) * cos(rot) -
                               (this->ypos/1e3) * sin(rot));
    py +=  ((this->xpos/1e3) * sin(rot) +
                               (this->ypos/1e3) * cos(rot));
    pa = DTOR(this->angle_offset + angle);
  }
  else
  {
    px += this->xpos / 1e3;
    py += this->ypos / 1e3;
    pa = DTOR(this->angle);
  }

  // timestamps get set in the p2os::StandardSIPPutData fcn
  data->position.header.frame_id = odom_frame_id;
  data->position.child_frame_id = base_link_frame_id;

  data->position.pose.pose.position.x = px;
  data->position.pose.pose.position.y = py;
  data->position.pose.pose.position.z = 0.0;
  data->position.pose.pose.orientation = tf::createQuaternionMsgFromYaw(pa);

    // add rates
  data->position.twist.twist.linear.x = ((lvel + rvel) / 2) / 1e3;
  data->position.twist.twist.linear.y = 0.0;
  data->position.twist.twist.angular.z = ((double)(rvel-lvel)/(2.0/PlayerRobotParams[param_idx].DiffConvFactor));

  
  data->position.pose.covariance = boost::assign::list_of	(1e-3) (0)    (0)   (0)   (0)   (0)
                                                          (0)    (1e-3) (0)   (0)   (0)   (0)
                                                          (0)    (0)    (1e6) (0)   (0)   (0)
                                                          (0)    (0)    (0)   (1e6) (0)   (0)
                                                          (0)    (0)    (0)   (0)   (1e6) (0)
                                                          (0)    (0)    (0)   (0)   (0)   (1e3) ;

	data->position.twist.covariance = boost::assign::list_of (1e-3) (0)    (0)   (0)   (0)   (0)
                                                           (0)    (1e-3) (0)   (0)   (0)   (0)
                                                           (0)    (0)    (1e6) (0)   (0)   (0)
                                                           (0)    (0)    (0)   (1e6) (0)   (0)
                                                           (0)    (0)    (0)   (0)   (1e6) (0)
                                                           (0)    (0)    (0)   (0)   (0)   (1e3) ;


  //publish transform
  data->odom_trans.header = data->position.header;
  data->odom_trans.child_frame_id = data->position.child_frame_id;
  data->odom_trans.transform.translation.x = px;
  data->odom_trans.transform.translation.y = py;
  data->odom_trans.transform.translation.z = 0;
  data->odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(pa); 

  // battery
  data->batt.voltage = battery / 10.0;

  // motor state
  // The below will tell us if the motors are currently moving or not, it does
  // not tell us whether they have been enabled
  // data->motors.state = (status & 0x01);
	data->motors.state = (motors_enabled & 0x01);
  /*
  ///////////////////////////////////////////////////////////////
  // compass
  memset(&(data->compass),0,sizeof(data->compass));
  data->compass.pos.pa = DTOR(this->compass);
  */

  ///////////////////////////////////////////////////////////////
  // sonar
  data->sonar.ranges_count = static_cast<int>(sonarreadings);
  data->sonar.ranges.clear();
  for(int i=0; i < data->sonar.ranges_count; i++)
    data->sonar.ranges.push_back(sonars[i] / 1e3);

  ///////////////////////////////////////////////////////////////
  // gripper
  unsigned char gripState = timer;
  if ((gripState & 0x01) && (gripState & 0x02) && !(gripState & 0x04))
  {
      data->gripper.grip.state = PLAYER_GRIPPER_STATE_ERROR;
      data->gripper.grip.dir = 0;
  }
  else if(gripState & 0x04)
  {
    data->gripper.grip.state = PLAYER_GRIPPER_STATE_MOVING;
    if(gripState & 0x01)
      data->gripper.grip.dir = 1;
    if(gripState & 0x02)
      data->gripper.grip.dir = -1;
  }
  else if(gripState & 0x01)
  {
    data->gripper.grip.state = PLAYER_GRIPPER_STATE_OPEN;
    data->gripper.grip.dir = 0;
  }
  else if(gripState & 0x02)
  {
    data->gripper.grip.state = PLAYER_GRIPPER_STATE_CLOSED;
    data->gripper.grip.dir = 0;
  }

  // Reset data to false
  data->gripper.grip.inner_beam = false;
  data->gripper.grip.outer_beam = false;
  data->gripper.grip.left_contact = false;
  data->gripper.grip.right_contact = false;

  if (digin & 0x08)
  {
    data->gripper.grip.inner_beam = true;
  }
  if (digin & 0x04)
  {
    data->gripper.grip.outer_beam = true;
  }
  if (!(digin & 0x10))
  {
    data->gripper.grip.left_contact = true;
  }
  if (!(digin & 0x20))
  {
    data->gripper.grip.right_contact = true;
  }

  // lift
  data->gripper.lift.dir = 0;

  if ((gripState & 0x10) && (gripState & 0x20) && !(gripState & 0x40))
  {
      // In this case, the lift is somewhere in between, so
      // must be at an intermediate carry position. Use last commanded position
      data->gripper.lift.state = PLAYER_ACTARRAY_ACTSTATE_IDLE;
      data->gripper.lift.position = lastLiftPos;
  }
  else if (gripState & 0x40)  // Moving
  {
      data->gripper.lift.state = PLAYER_ACTARRAY_ACTSTATE_MOVING;
      // There is no way to know where it is for sure, so use last commanded
      // position.
      data->gripper.lift.position = lastLiftPos;
      if (gripState & 0x10)
        data->gripper.lift.dir = 1;
      else if (gripState & 0x20)
        data->gripper.lift.dir = -1;
  }
  else if (gripState & 0x10)  // Up
  {
      data->gripper.lift.state = PLAYER_ACTARRAY_ACTSTATE_IDLE;
      data->gripper.lift.position = 1.0f;
      data->gripper.lift.dir = 0;
  }
  else if (gripState & 0x20)  // Down
  {
      data->gripper.lift.state = PLAYER_ACTARRAY_ACTSTATE_IDLE;
      data->gripper.lift.position = 0.0f;
      data->gripper.lift.dir = 0;
  }
  else    // Assume stalled
  {
      data->gripper.lift.state = PLAYER_ACTARRAY_ACTSTATE_STALLED;
  }
  // Store the last lift position
  lastLiftPos = data->gripper.lift.position;

  /*
  ///////////////////////////////////////////////////////////////
  // bumper
  unsigned int bump_count = PlayerRobotParams[param_idx].NumFrontBumpers + PlayerRobotParams[param_idx].NumRearBumpers;
  if (data->bumper.bumpers_count != bump_count)
  {
    data->bumper.bumpers_count = bump_count;
    delete [] data->bumper.bumpers;
    data->bumper.bumpers = new uint8_t[bump_count];
  }
  int j = 0;
  for(int i=PlayerRobotParams[param_idx].NumFrontBumpers-1;i>=0;i--)
    data->bumper.bumpers[j++] =
      (unsigned char)((this->frontbumpers >> i) & 0x01);
  for(int i=PlayerRobotParams[param_idx].NumRearBumpers-1;i>=0;i--)
    data->bumper.bumpers[j++] =
      (unsigned char)((this->rearbumpers >> i) & 0x01);
  */

  ///////////////////////////////////////////////////////////////
  // digital I/O
  data->dio.count = 8;
  data->dio.bits = (unsigned int)this->digin;

  ///////////////////////////////////////////////////////////////
  // analog I/O
  //TODO: should do this smarter, based on which analog input is selected
  data->aio.voltages_count = (unsigned char)1;
  // if (!data->aio.voltages)
  //   data->aio.voltages = new float[1];
  // data->aio.voltages[0] = (this->analog / 255.0) * 5.0;
  data->aio.voltages.clear();
  data->aio.voltages.push_back((this->analog / 255.0) * 5.0);
}

int SIP::PositionChange( unsigned short from, unsigned short to )
{
  int diff1, diff2;

  /* find difference in two directions and pick shortest */
  if ( to > from ) {
    diff1 = to - from;
    diff2 = - ( from + 4096 - to );
  }
  else {
    diff1 = to - from;
    diff2 = 4096 - from + to;
  }

  if ( abs(diff1) < abs(diff2) )
    return(diff1);
  else
    return(diff2);

}

void SIP::Print()
{
  int i;

  ROS_DEBUG("lwstall:%d rwstall:%d\n", lwstall, rwstall);

  std::stringstream front_bumper_info;
  for(int i=0;i<5;i++) {
    front_bumper_info << " "
                      << static_cast<int>((frontbumpers >> i) & 0x01 );
  }
  ROS_DEBUG("Front bumpers:%s", front_bumper_info.str().c_str());
  std::stringstream rear_bumper_info;
  for(int i=0;i<5;i++) {
    rear_bumper_info << " "
                     << static_cast<int>((rearbumpers >> i) & 0x01 );
  }
  ROS_DEBUG("Rear bumpers:%s", rear_bumper_info.str().c_str());

  ROS_DEBUG("status: 0x%x analog: %d param_id: %d ", status, analog, param_idx);
  std::stringstream status_info;
  for(i=0;i<11;i++) {
    status_info << " "
               << static_cast<int>((status >> (7-i) ) & 0x01);
  }
  ROS_DEBUG("status:%s", status_info.str().c_str());
  std::stringstream digin_info;
  for(i=0;i<8;i++) {
    digin_info << " "
               << static_cast<int>((digin >> (7-i) ) & 0x01);
  }
  ROS_DEBUG("digin:%s", digin_info.str().c_str());
  std::stringstream digout_info;
  for(i=0;i<8;i++) {
    digout_info << " "
               << static_cast<int>((digout >> (7-i) ) & 0x01);
  }
  ROS_DEBUG("digout:%s", digout_info.str().c_str());
  ROS_DEBUG("battery: %d compass: %d sonarreadings: %d\n", battery,
            compass, sonarreadings);
  ROS_DEBUG("xpos: %d ypos:%d ptu:%hu timer:%hu\n", xpos, ypos, ptu, timer);
  ROS_DEBUG("angle: %d lvel: %d rvel: %d control: %d\n", angle, lvel,
            rvel, control);

  PrintSonars();
  PrintArmInfo ();
  PrintArm ();
}

void SIP::PrintSonars()
{
  if(sonarreadings <= 0)
    return;
  std::stringstream sonar_info;
  for(int i = 0; i < sonarreadings; i++){
    sonar_info << " " << static_cast<int>(sonars[i]);
  }
  ROS_DEBUG("Sonars: %s", sonar_info.str().c_str());
}

void SIP::PrintArm ()
{
	ROS_DEBUG ("Arm power is %s\tArm is %sconnected\n", (armPowerOn ? "on" : "off"), (armConnected ? "" : "not "));
	ROS_DEBUG ("Arm joint status:\n");
	for (int ii = 0; ii < 6; ii++)
		ROS_DEBUG ("Joint %d   %s   %d\n", ii + 1, (armJointMoving[ii] ? "Moving " : "Stopped"), armJointPos[ii]);
}

void SIP::PrintArmInfo ()
{
	ROS_DEBUG ("Arm version:\t%s\n", armVersionString);
	ROS_DEBUG ("Arm has %d joints:\n", armNumJoints);
	ROS_DEBUG ("  |\tSpeed\tHome\tMin\tCentre\tMax\tTicks/90\n");
	for (int ii = 0; ii < armNumJoints; ii++)
		ROS_DEBUG ("%d |\t%d\t%d\t%d\t%d\t%d\t%d\n", ii, armJoints[ii].speed, armJoints[ii].home, armJoints[ii].min, armJoints[ii].centre, armJoints[ii].max, armJoints[ii].ticksPer90);
}

void SIP::ParseStandard( unsigned char *buffer )
{
  int cnt = 0, change;
  unsigned short newxpos, newypos;

  status = buffer[cnt];
  cnt += sizeof(unsigned char);
  /*
   * Remember P2OS uses little endian:
   * for a 2 byte short (called integer on P2OS)
   * byte0 is low byte, byte1 is high byte
   * The following code is host-machine endian independant
   * Also we must or (|) bytes together instead of casting to a
   * short * since on ARM architectures short * must be even byte aligned!
   * You can get away with this on a i386 since shorts * can be
   * odd byte aligned. But on ARM, the last bit of the pointer will be ignored!
   * The or'ing will work on either arch.
   */
  newxpos = ((buffer[cnt] | (buffer[cnt+1] << 8))
	     & 0xEFFF) % 4096; /* 15 ls-bits */

  if (xpos!=INT_MAX) {
    change = (int) rint(PositionChange( rawxpos, newxpos ) *
			PlayerRobotParams[param_idx].DistConvFactor);
    if (abs(change)>100)
      ROS_DEBUG("invalid odometry change [%d]; odometry values are tainted", change);
    else
      xpos += change;
  }
  else
    xpos = 0;
  rawxpos = newxpos;
  cnt += sizeof(short);

  newypos = ((buffer[cnt] | (buffer[cnt+1] << 8))
	     & 0xEFFF) % 4096; /* 15 ls-bits */

  if (ypos!=INT_MAX) {
    change = (int) rint(PositionChange( rawypos, newypos ) *
			PlayerRobotParams[param_idx].DistConvFactor);
    if (abs(change)>100)
      ROS_DEBUG("invalid odometry change [%d]; odometry values are tainted", change);
    else
      ypos += change;
  }
  else
    ypos = 0;
  rawypos = newypos;
  cnt += sizeof(short);

  angle = (short)
    rint(((short)(buffer[cnt] | (buffer[cnt+1] << 8))) *
	 PlayerRobotParams[param_idx].AngleConvFactor * 180.0/M_PI);
  cnt += sizeof(short);

  lvel = (short)
    rint(((short)(buffer[cnt] | (buffer[cnt+1] << 8))) *
	 PlayerRobotParams[param_idx].VelConvFactor);
  cnt += sizeof(short);

  rvel = (short)
    rint(((short)(buffer[cnt] | (buffer[cnt+1] << 8))) *
	 PlayerRobotParams[param_idx].VelConvFactor);
  cnt += sizeof(short);

  battery = buffer[cnt];
  cnt += sizeof(unsigned char);
  ROS_DEBUG( "battery value: %d", battery );

  lwstall = buffer[cnt] & 0x01;
  rearbumpers = buffer[cnt] >> 1;
  cnt += sizeof(unsigned char);

  rwstall = buffer[cnt] & 0x01;
  frontbumpers = buffer[cnt] >> 1;
  cnt += sizeof(unsigned char);

  control = (short)
    rint(((short)(buffer[cnt] | (buffer[cnt+1] << 8))) *
	 PlayerRobotParams[param_idx].AngleConvFactor);
  cnt += sizeof(short);

  ptu = (buffer[cnt] | (buffer[cnt+1] << 8));
	motors_enabled = buffer[cnt];
	sonar_flag = buffer[cnt+1];
  cnt += sizeof(short);

  //compass = buffer[cnt]*2;
  if(buffer[cnt] != 255 && buffer[cnt] != 0 && buffer[cnt] != 181)
    compass = (buffer[cnt]-1)*2;
  cnt += sizeof(unsigned char);

  unsigned char numSonars=buffer[cnt];
  cnt+=sizeof(unsigned char);

  if(numSonars > 0)
  {
    //find the largest sonar index supplied
    unsigned char maxSonars=sonarreadings;
    for(unsigned char i=0;i<numSonars;i++)
    {
      unsigned char sonarIndex=buffer[cnt+i*(sizeof(unsigned char)+sizeof(unsigned short))];
      if((sonarIndex+1)>maxSonars) maxSonars=sonarIndex+1;
    }

    //if necessary make more space in the array and preserve existing readings
    if(maxSonars>sonarreadings)
    {
      unsigned short *newSonars=new unsigned short[maxSonars];
      for(unsigned char i=0;i<sonarreadings;i++)
        newSonars[i]=sonars[i];
      if(sonars!=NULL) delete[] sonars;
      sonars=newSonars;
      sonarreadings=maxSonars;
    }

    //update the sonar readings array with the new readings
    for(unsigned char i=0;i<numSonars;i++)
    {
    sonars[buffer[cnt]]=   (unsigned short)
      rint((buffer[cnt+1] | (buffer[cnt+2] << 8)) *
	   PlayerRobotParams[param_idx].RangeConvFactor);
      cnt+=sizeof(unsigned char)+sizeof(unsigned short);
  }
  }

  timer = (buffer[cnt] | (buffer[cnt+1] << 8));
  cnt += sizeof(short);

  analog = buffer[cnt];
  cnt += sizeof(unsigned char);

  digin = buffer[cnt];
  cnt += sizeof(unsigned char);

  digout = buffer[cnt];
  cnt += sizeof(unsigned char);
  // for debugging:
  Print();
  // PrintSonars();
}

/** Parse a SERAUX SIP packet.  For a CMUcam, this will have blob
 **  tracking messages in the format (all one-byte values, no spaces):
 **
 **      255 M mx my x1 y1 x2 y2 pixels confidence  (10-bytes)
 **
 ** Or color info messages of the format:
 **
 **      255 S Rval Gval Bval Rvar Gvar Bvar    (8-bytes)
 */
void SIP::ParseSERAUX( unsigned char *buffer )
{
  unsigned char type = buffer[1];
  if (type != SERAUX && type != SERAUX2)
  {
    // Really should never get here...
    ROS_ERROR("Attempt to parse non SERAUX packet as serial data.\n");
    return;
  }

  int len  = (int)buffer[0]-3;		// Get the string length

  /* First thing is to find the beginning of last full packet (if possible).
  ** If there are less than CMUCAM_MESSAGE_LEN*2-1 bytes (19), we're not
  ** guaranteed to have a full packet.  If less than CMUCAM_MESSAGE_LEN
  ** bytes, it is impossible to have a full packet.
  ** To find the beginning of the last full packet, search between bytes
  ** len-17 and len-8 (inclusive) for the start flag (255).
  */
  int ix;
  for (ix = (len>18 ? len-17 : 2); ix<=len-8; ix++)
    if (buffer[ix] == 255)
      break;		// Got it!
  if (len < 10 || ix > len-8) {
    ROS_ERROR("Failed to get a full blob tracking packet.\n");
    return;
  }

  // There is a special 'S' message containing the tracking color info
  if (buffer[ix+1] == 'S')
  {
     // Color information (track color)
     ROS_DEBUG("Tracking color (RGB):  %d %d %d\n"
            "       with variance:  %d %d %d\n",
              buffer[ix+2], buffer[ix+3], buffer[ix+4],
              buffer[ix+5], buffer[ix+6], buffer[ix+7]);
     blobcolor = buffer[ix+2]<<16 | buffer[ix+3]<<8 | buffer[ix+4];
     return;
  }

  // Tracking packets with centroid info are designated with a 'M'
  if (buffer[ix+1] == 'M')
  {
     // Now it's easy.  Just parse the packet.
     blobmx	= buffer[ix+2];
     blobmy	= buffer[ix+3];
     blobx1	= buffer[ix+4];
     bloby1	= buffer[ix+5];
     blobx2	= buffer[ix+6];
     bloby2	= buffer[ix+7];
     blobconf	= buffer[ix+9];
     // Xiaoquan Fu's calculation for blob area (max 11297).
     blobarea	= (bloby2 - bloby1 +1)*(blobx2 - blobx1 + 1)*blobconf/255;
     return;
  }

  ROS_ERROR("Unknown blob tracker packet type: %c\n", buffer[ix+1]);
  return;
}

// Parse a set of gyro measurements.  The buffer is formatted thusly:
//     length (2 bytes), type (1 byte), count (1 byte)
// followed by <count> pairs of the form:
//     rate (2 bytes), temp (1 byte)
// <rate> falls in [0,1023]; less than 512 is CCW rotation and greater
// than 512 is CW
void
SIP::ParseGyro(unsigned char* buffer)
{
  // Get the message length (account for the type byte and the 2-byte
  // checksum)
  int len  = (int)buffer[0]-3;

  unsigned char type = buffer[1];
  if(type != GYROPAC)
  {
    // Really should never get here...
    ROS_ERROR("Attempt to parse non GYRO packet as gyro data.\n");
    return;
  }

  if(len < 1)
  {
    ROS_DEBUG("Couldn't get gyro measurement count");
    return;
  }

  // get count
  int count = (int)buffer[2];

  // sanity check
  if((len-1) != (count*3))
  {
    ROS_DEBUG("Mismatch between gyro measurement count and packet length");
    return;
  }

  // Actually handle the rate values.  Any number of things could (and
  // probably should) be done here, like filtering, calibration, conversion
  // from the gyro's arbitrary units to something meaningful, etc.
  //
  // As a first cut, we'll just average all the rate measurements in this
  // set, and ignore the temperatures.
  float ratesum = 0;
  int bufferpos = 3;
  unsigned short rate;
  unsigned char temp;
  for(int i=0; i<count; i++)
  {
    rate = (unsigned short)(buffer[bufferpos++]);
    rate |= buffer[bufferpos++] << 8;
    temp = bufferpos++;

    ratesum += rate;
  }

  int32_t average_rate = (int32_t)rint(ratesum / (float)count);

  // store the result for sending
  gyro_rate = average_rate;
}

void SIP::ParseArm (unsigned char *buffer)
{
	int length = (int) buffer[0] - 2;

	if (buffer[1] != ARMPAC)
	{
		ROS_ERROR("Attempt to parse a non ARM packet as arm data.\n");
		return;
	}

	if (length < 1 || length != 9)
	{
		ROS_DEBUG ("ARMpac length incorrect size");
		return;
	}

	unsigned char status = buffer[2];
	if (status & 0x01)
		armPowerOn = true;		// Power is on
	else
		armPowerOn = false;		// Power is off

	if (status & 0x02)
		armConnected = true;	// Connection is established
	else
		armConnected = false;	// Connection is not established

	unsigned char motionStatus = buffer[3];
	if (motionStatus & 0x01)
		armJointMoving[0] = true;
	if (motionStatus & 0x02)
		armJointMoving[1] = true;
	if (motionStatus & 0x04)
		armJointMoving[2] = true;
	if (motionStatus & 0x08)
		armJointMoving[3] = true;
	if (motionStatus & 0x10)
		armJointMoving[4] = true;
	if (motionStatus & 0x20)
		armJointMoving[5] = true;

	memcpy (armJointPos, &buffer[4], 6);
	memset (armJointPosRads, 0, 6 * sizeof (double));
}

void SIP::ParseArmInfo (unsigned char *buffer)
{
	int length = (int) buffer[0] - 2;
	if (buffer[1] != ARMINFOPAC)
	{
		ROS_ERROR ("Attempt to parse a non ARMINFO packet as arm info.\n");
		return;
	}

	if (length < 1)
	{
		ROS_DEBUG ("ARMINFOpac length bad size");
		return;
	}

	// Copy the version string
	if (armVersionString)
		free (armVersionString);
        // strndup() isn't available everywhere (e.g., Darwin)
	//armVersionString = strndup ((char*) &buffer[2], length);		// Can't be any bigger than length
        armVersionString = (char*)calloc(length+1,sizeof(char));
        assert(armVersionString);
        strncpy(armVersionString,(char*)&buffer[2], length);
	int dataOffset = strlen (armVersionString) + 3;		// +1 for packet size byte, +1 for packet ID, +1 for null byte

	armNumJoints = buffer[dataOffset];
	if (armJoints)
		delete[] armJoints;
	if (armNumJoints <= 0)
		return;
	armJoints = new ArmJoint[armNumJoints];
	dataOffset += 1;
	for (int ii = 0; ii < armNumJoints; ii++)
	{
		armJoints[ii].speed = buffer[dataOffset + (ii * 6)];
		armJoints[ii].home = buffer[dataOffset + (ii * 6) + 1];
		armJoints[ii].min = buffer[dataOffset + (ii * 6) + 2];
		armJoints[ii].centre = buffer[dataOffset + (ii * 6) + 3];
		armJoints[ii].max = buffer[dataOffset + (ii * 6) + 4];
		armJoints[ii].ticksPer90 = buffer[dataOffset + (ii * 6) + 5];
	}
}
