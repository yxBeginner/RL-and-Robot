/*
 *  P2OS for ROS
 *  Copyright (C) 2000
 *     Tucker Hermans
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

#include <p2os_driver/p2os_ptz.h>
#include <p2os_driver/p2os.h>

//
// Constants
//
const int P2OSPtz::MAX_COMMAND_LENGTH = 19;
const int P2OSPtz::MAX_REQUEST_LENGTH = 17;
const int P2OSPtz::COMMAND_RESPONSE_BYTES = 6;
const int P2OSPtz::PACKET_TIMEOUT = 300;
const int P2OSPtz::SLEEP_TIME_USEC = 300000;
const int P2OSPtz::PAN_THRESH = 1;
const int P2OSPtz::TILT_THRESH = 1;
const int P2OSPtz::ZOOM_THRESH = 1;

//
// Constructors
//
P2OSPtz::P2OSPtz(P2OSNode * p2os, bool bidirectional_com) :
    p2os_(p2os), max_zoom_(MAX_ZOOM_OPTIC), pan_(0), tilt_(0), zoom_(0),
    is_on_(false), error_code_(CAM_ERROR_NONE),
    bidirectional_com_(bidirectional_com)
{
  current_state_.pan = 0;
  current_state_.zoom = 0;
  current_state_.tilt = 0;
  current_state_.relative = false;
}


//
// Core Functions
//
int P2OSPtz::setup()
{
  int err = 0;
  int num_inits = 7;
  is_on_ = true;
  for (int i = 1; i < num_inits; i++) {
    switch(i)
    {
      // case 0:
      //   do
      //   {
      //     ROS_DEBUG("Waiting for camera to power off.");
      //     err = setPower(POWER_OFF);
      //   } while (error_code_ == CAM_ERROR_BUSY);
      //   break;
      case 1:
        do
        {
          ROS_DEBUG("Waiting for camera to power on.");
          err = setPower(POWER_ON);
        } while (error_code_ == CAM_ERROR_BUSY);
        break;
      case 2:
        do
        {
          ROS_DEBUG("Waiting for camera mode to set");
          err = setControlMode();
        } while (error_code_ == CAM_ERROR_BUSY);
        break;
      case 3:
        do
        {
          ROS_DEBUG("Waiting for camera to initialize");
          err = sendInit();
        } while (error_code_ == CAM_ERROR_BUSY);
        break;
      case 4:
        do
        {
          for(int i = 0; i < 3; i++)
          {
            ROS_DEBUG("Waiting for camera to set default tilt");
            err = setDefaultTiltRange();
          }
        } while (error_code_ == CAM_ERROR_BUSY);
        break;
      case 5:
        do
        {
          ROS_DEBUG("Waiting for camera to set initial pan and tilt");
          err = sendAbsPanTilt(0, 0);
        } while (error_code_ == CAM_ERROR_BUSY);
        break;
      case 6:
        do
        {
          ROS_DEBUG("Waiting for camera to set initial zoom");
          err = sendAbsZoom(0);
        } while (error_code_ == CAM_ERROR_BUSY);
        break;
      default:
        err = -7;
        break;
    }

    // Check for erros after each attempt
    if (err)
    {
      ROS_ERROR("Error initiliazing PTZ at stage %i", i);
      switch(error_code_)
      {
        case CAM_ERROR_BUSY:
          ROS_ERROR("Error: CAM_ERROR_BUSY");
          break;
        case CAM_ERROR_PARAM:
          ROS_ERROR("Error: CAM_ERROR_PARAM");
          break;
        case CAM_ERROR_MODE:
          ROS_ERROR("Error: CAM_ERROR_MODE");
          break;
        default:
          ROS_ERROR("Error: Unknown error response from camera.");
          break;
      }
      return(-1);
    }
    else
    {
      ROS_DEBUG("Passed stage %i of PTZ initialization.", i);
    }
  }
  ROS_DEBUG("Finished initialization of the PTZ.");
  return 0;
}

void P2OSPtz::shutdown()
{
  sendAbsPanTilt(0,0);
  usleep(SLEEP_TIME_USEC);
  sendAbsZoom(0);
  usleep(SLEEP_TIME_USEC);
  setPower(POWER_OFF);
  usleep(SLEEP_TIME_USEC);
  ROS_INFO("PTZ camera has been shutdown");
}

void P2OSPtz::callback(const p2os_msgs::PTZStateConstPtr &cmd)
{
  p2os_msgs::PTZState to_send;
  bool change_pan_tilt = false;
  bool change_zoom = false;
  to_send.pan = pan_;
  to_send.tilt = tilt_;
  to_send.zoom = zoom_;

  // Check if the command is relative to the current position
  if (cmd->relative)
  {
    if ( abs(cmd->pan) > PAN_THRESH)
    {
      to_send.pan = cmd->pan + pan_;
      change_pan_tilt = true;
    }
    if ( abs(cmd->tilt) > TILT_THRESH)
    {
      to_send.tilt = cmd->tilt + tilt_;
      change_pan_tilt = true;
    }
    if ( abs(cmd->zoom) > ZOOM_THRESH)
    {
      to_send.zoom = cmd->zoom + zoom_;
      change_zoom = true;
    }
  }
  else
  {
    if ( abs(cmd->pan - pan_) > PAN_THRESH)
    {
      to_send.pan = cmd->pan;
      change_pan_tilt = true;
    }
    if ( abs(cmd->tilt - tilt_) > TILT_THRESH)
    {
      to_send.tilt = cmd->tilt;
      change_pan_tilt = true;
    }
    if ( abs(cmd->zoom - zoom_) > ZOOM_THRESH)
    {
      to_send.zoom = cmd->zoom;
      change_zoom = true;
    }
  }

  if (change_pan_tilt)
  {
    sendAbsPanTilt(to_send.pan, to_send.tilt);
  }
  if (change_zoom)
  {
    sendAbsZoom(to_send.zoom);
  }

  current_state_.pan = pan_;
  current_state_.zoom = zoom_;
  current_state_.tilt = tilt_;
}

//
// Communication Functions
//
int P2OSPtz::sendCommand(unsigned char *str, int len)
{
  P2OSPacket ptz_packet;
  P2OSPacket request_pkt;
  unsigned char request[4];

  // Zero out the Receive Buffer
  request[0] = GETAUX;
  request[1] = ARGINT;
  request[2] = 0;
  request[3] = 0;
  request_pkt.Build(request,4);
  p2os_->SendReceive(&request_pkt,false);

  if(len > MAX_COMMAND_LENGTH)
  {
    ROS_ERROR("Command message is too large to send");
    return(-1);
  }

  // Since I'm hardcoding this to AUX1, basically we gotta stick the AUX1DATA
  // header on this and then give it to the p2os send command.
  unsigned char mybuf[MAX_COMMAND_LENGTH+3];
  mybuf[0] = TTY2;
  mybuf[1] = ARGSTR;
  mybuf[2] = len;
  // Copy the command
  memcpy(&mybuf[3], str, len);
  ptz_packet.Build(mybuf, len+3);

  // Send the packet
  p2os_->SendReceive(&ptz_packet, false);

  return(0);
}

int P2OSPtz::sendRequest(unsigned char *str, int len, unsigned char *reply)
{
  P2OSPacket ptz_packet;
  P2OSPacket request_pkt;
  unsigned char request[4];

  // Zero out the Receive Buffer
  request[0] = GETAUX;
  request[1] = ARGINT;
  request[2] = 0;
  request[3] = 0;
  request_pkt.Build(request,4);
  p2os_->SendReceive(&request_pkt,false);

  if (len > MAX_REQUEST_LENGTH)
  {
    ROS_ERROR("Request message is too large to send.");
    return -1;
  }

  // Since I'm hardcoding this to AUX1, basically we gotta stick the AUX1DATA
  // header on this and then give it to the p2os send command.
  unsigned char mybuf[MAX_REQUEST_LENGTH];
  mybuf[0] = TTY2;
  mybuf[1] = ARGSTR;
  mybuf[2] = len;
  // Copy the command
  memcpy(&mybuf[3], str, len);
  ptz_packet.Build(mybuf, len+3);

  // Send the packet
  p2os_->SendReceive(&ptz_packet, false);


  return 0;
}

int P2OSPtz::receiveCommandAnswer(int asize)
{
  int num;
  unsigned char reply[MAX_REQUEST_LENGTH];
  int len = 0;
  unsigned char byte;
  int t;
  memset(reply, 0, COMMAND_RESPONSE_BYTES);

  getPtzPacket(asize);

  for (num = 0; num <= COMMAND_RESPONSE_BYTES + 1; num++)
  {
    // if we don't get any bytes, or if we've just exceeded the limit
    // then return null
    t = cb_.getFromBuf();
    if ( t < 0 )
    {
      // Buf Error!
      ROS_ERROR("circbuf error!");
      return -1;
    }
    else
    {
      byte = (unsigned char)t;
    }
    if (byte == (unsigned char)RESPONSE)
    {
      reply[0] = byte;
      len ++;
      break;
    }
  }

  if (len == 0)
  {
    ROS_ERROR("Length is 0 on received packet.");
    return -1;
  }

  // we got the header character so keep reading bytes for MAX_RESPONSE_BYTES more
  for(num = 1; num <= MAX_REQUEST_LENGTH; num++)
  {
    t = cb_.getFromBuf();
    if (t < 0)
    {
      // there are no more bytes, so check the last byte for the footer
      if (reply[len - 1] !=  (unsigned char)FOOTER)
      {
        ROS_ERROR("canonvcc4::receiveCommandAnswer: Discarding bad packet.");
        return -1;
      }
      else
        break;
    }
    else
    {
      // add the byte to the array
      reply[len] = (unsigned char)t;
      len ++;
    }
  }

  // Check the response
  if (len != COMMAND_RESPONSE_BYTES)
  {
    ROS_ERROR("Answer does not equal command response bytes");
    return -1;
  }

  // check the header and footer
  if (reply[0] != (unsigned char)RESPONSE || reply[5] != (unsigned char)FOOTER)
  {
    ROS_ERROR("Header or Footer is wrong on received packet");
    return -1;
  }

  // so far so good.  Set myError to the error byte
  error_code_ = reply[3];
  if (error_code_ == CAM_ERROR_NONE)
  {
    return 0;
  }

  switch(error_code_)
  {
    case CAM_ERROR_BUSY:
      ROS_ERROR("Error: CAM_ERROR_BUSY");
      break;
    case CAM_ERROR_PARAM:
      ROS_ERROR("Error: CAM_ERROR_PARAM");
      break;
    case CAM_ERROR_MODE:
      ROS_ERROR("Error: CAM_ERROR_MODE");
      break;
    default:
      ROS_ERROR("Error: Unknown error response from camera.");
      break;
  }
  return -1;
}

/* These commands often have variable packet lengths, if there is an error,
 * there is a smaller packet size. If we request the larger packet size first,
 * then we will never get a response back. Because of this, we have to first
 * request the smaller size, check if its a full packet, if it's not, request
 * the rest of the packet. Also according to the source code for ARIA, we can
 * not do more than 2 requests for a single packet, therefor, we can't just
 * request 1 byte over and over again.
 *
 * So here, s1 is the size of the smaller packet.
 * And s2 is the size of the larger packet.
 */
int P2OSPtz::receiveRequestAnswer(unsigned char *data, int s1, int s2)
{
  int num;
  unsigned char reply[MAX_REQUEST_LENGTH];
  int len = 0;
  unsigned char byte;
  int t;

  memset(reply, 0, MAX_REQUEST_LENGTH);
  getPtzPacket(s1, s2);

  for (num = 0; num <= COMMAND_RESPONSE_BYTES + 1; num++)
  {
    // if we don't get any bytes, or if we've just exceeded the limit
    // then return null
    t = cb_.getFromBuf();
    if ( t < 0 ) { // Buf Error!
      ROS_ERROR("circbuf error!\n");
      return -1;
    }
    else {
      byte = (unsigned char)t;
    }
    if (byte == (unsigned char)RESPONSE)
    {
      reply[0] = byte;
      len ++;
      break;
    }
  }
  if (len == 0)
  {
    ROS_ERROR("Received Request Answer has length 0");
    return -1;
  }
  // we got the header character so keep reading bytes for MAX_RESPONSE_BYTES more
  for(num = 1; num <= MAX_REQUEST_LENGTH; num++)
  {
    t = cb_.getFromBuf();
    if (t < 0)
    {
      // there are no more bytes, so check the last byte for the footer
      if (reply[len - 1] !=  (unsigned char)FOOTER)
      {
        ROS_ERROR("Last Byte was not the footer!");
        return -1;
      }
      else
        break;
    }
    else
    {
      // add the byte to the array
      reply[len] = (unsigned char)t;
      len ++;
    }
  }
  // Check the response length: pt: 14; zoom: 10
  if (len != COMMAND_RESPONSE_BYTES && len != 8 && len != 10 && len != 14)
  {
    ROS_ERROR("Response Length was incorrect at %i.", len);
    return -1;
  }

  if (reply[0] !=  (unsigned char)RESPONSE ||
      reply[len - 1] != (unsigned char)FOOTER)
  {
    ROS_ERROR("Header or Footer is wrong on received packet");
    return -1;
  }

  // so far so good.  Set myError to the error byte
  error_code_ = reply[3];

  if (error_code_ == CAM_ERROR_NONE)
  {
    memcpy(data, reply, len);
    return len;
  }
  switch(error_code_)
  {
    case CAM_ERROR_BUSY:
      ROS_ERROR("Error: CAM_ERROR_BUSY");
      break;
    case CAM_ERROR_PARAM:
      ROS_ERROR("Error: CAM_ERROR_PARAM");
      break;
    case CAM_ERROR_MODE:
      ROS_ERROR("Error: CAM_ERROR_MODE");
      break;
    default:
      ROS_ERROR("Error: Unknown error response from camera.");
      break;
  }
  return -1;
}

void P2OSPtz::getPtzPacket(int s1, int s2)
{
  int packetCount = 0;
  unsigned char request[4];
  P2OSPacket request_pkt;
  bool secondSent = false;

  request[0] = GETAUX;
  request[1] = ARGINT;
  request[2] = s1;
  request[3] = 0;

  // Reset our receiving buffer.
  cb_.reset();

  //Request the request-size back
  request_pkt.Build(request,4);
  p2os_->SendReceive(&request_pkt,false);

  while ( !cb_.gotPacket() )
  {
    if ( packetCount++ > PACKET_TIMEOUT ) {
      // Give Up We're not getting it.
      ROS_ERROR("Waiting for packet timed out.");
      return;
    }
    if ( cb_.size() == s1 && !secondSent)
    {
      if ( s2 > s1 )
      {
        // We got the first packet size, but we don't have a full packet.
        int newsize = s2 - s1;
        ROS_ERROR("Requesting Second Packet of size %i.", newsize);
        request[2] = newsize;
        request_pkt.Build(request,4);
        secondSent = true;
        p2os_->SendReceive(&request_pkt,false);
      }
      else
      {
        // We got the first packet but don't have a full packet, this is an error.
        ROS_ERROR("Got reply from AUX1 But don't have a full packet.");
        break;
      }
    }

    // Keep reading data until we get a response from the camera.
    p2os_->SendReceive(NULL,false);
  }
}

//
// Device Commands
//
int P2OSPtz::setPower(Power on)
{
  unsigned char command[MAX_COMMAND_LENGTH];

  command[0] = HEADER;
  command[1] = DEVICEID;
  command[2] = DEVICEID;
  command[3] = DELIM;
  command[4] = POWER;
  if (on)
    command[5] = DEVICEID + 1;
  else
    command[5] = DEVICEID;
  command[6] = FOOTER;

  if (sendCommand(command, 7))
    return -1;
  if (bidirectional_com_)
  {
    return (receiveCommandAnswer(COMMAND_RESPONSE_BYTES));
  }
  else
  {
    usleep(SLEEP_TIME_USEC);
    return 0;
  }
}

int P2OSPtz::setControlMode()
{
  unsigned char command[MAX_COMMAND_LENGTH];

  command[0] = HEADER;
  command[1] = DEVICEID;
  command[2] = DEVICEID;
  command[3] = DELIM;
  command[4] = CONTROL;
  command[5] = DEVICEID;
  command[6] = FOOTER;

  if (sendCommand(command, 7))
    return -1;
  if (bidirectional_com_)
  {
    return (receiveCommandAnswer(COMMAND_RESPONSE_BYTES));
  }
  else
  {
    usleep(SLEEP_TIME_USEC);
    return 0;
  }
}

int P2OSPtz::sendInit()
{
  unsigned char command[MAX_COMMAND_LENGTH];

  command[0] = HEADER;
  command[1] = DEVICEID;
  command[2] = DEVICEID;
  command[3] = DELIM;
  command[4] = INIT;
  command[5] = DEVICEID;
  command[6] = FOOTER;

  if (sendCommand(command, 7))
    return -1;
  if (bidirectional_com_)
  {
    return (receiveCommandAnswer(COMMAND_RESPONSE_BYTES));
  }
  else
  {
    usleep(SLEEP_TIME_USEC);
    return 0;
  }
}

int P2OSPtz::getMaxZoom(int * maxzoom)
{
  unsigned char command[MAX_COMMAND_LENGTH];
  unsigned char reply[MAX_REQUEST_LENGTH];
  int reply_len;
  char byte;
  unsigned char buf[4];
  unsigned int u_zoom;
  int i;

  command[0] = HEADER;
  command[1] = DEVICEID;
  command[2] = DEVICEID;
  command[3] = DELIM;
  command[4] = ZOOMREQ;
  command[5] = DEVICEID;
  command[6] = FOOTER;

  if (sendCommand(command, 7))
    return -1;
  //  usleep(5000000);
  if (bidirectional_com_)
  {
    reply_len = receiveRequestAnswer(reply,10,0);
  }
  else
  {
    return 0;
  }

  if ( reply_len == COMMAND_RESPONSE_BYTES ){
    return -1;
  }

  // remove the ascii encoding, and put into 2 bytes
  for (i = 0; i < 4; i++)
  {
    byte = reply[i + 5];
    if (byte < 0x40)
      byte = byte - 0x30;
    else
      byte = byte - 'A' + 10;
    buf[i] = byte;
  }

  // convert the 2 bytes into a number
  u_zoom = 0;
  for (i = 0; i < 4; i++)
    u_zoom += buf[i] * (unsigned int) pow(16.0, (double)(3 - i));
  *maxzoom = u_zoom;

  return 0;
}
int P2OSPtz::getAbsZoom(int* zoom)
{
  unsigned char command[MAX_COMMAND_LENGTH];
  unsigned char reply[MAX_REQUEST_LENGTH];
  int reply_len;
  char byte;
  unsigned char buf[4];
  unsigned int u_zoom;
  int i;

  command[0] = HEADER;
  command[1] = DEVICEID;
  command[2] = DEVICEID;
  command[3] = DELIM;
  command[4] = ZOOMREQ;
  command[5] = DEVICEID;
  command[6] = FOOTER;

  if (sendRequest(command, 6, reply))
    return(-1);
  if (bidirectional_com_)
  {
    reply_len = receiveRequestAnswer(reply,10,0);
  }
  else
  {
    return 0;
  }

  if (reply_len == COMMAND_RESPONSE_BYTES)
    return -1;

  // remove the ascii encoding, and put into 2 bytes
  for (i = 0; i < 4; i++)
  {
    byte = reply[i + 5];
    if (byte < 0x40)
      byte = byte - 0x30;
    else
      byte = byte - 'A' + 10;
    buf[i] = byte;
  }

  // convert the 2 bytes into a number
  u_zoom = 0;
  for (i = 0; i < 4; i++)
    u_zoom += buf[i] * (unsigned int) pow(16.0, (double)(3 - i));
  *zoom = u_zoom;
  return(0);
}

int P2OSPtz::sendAbsZoom(int zoom)
{
  unsigned char command[MAX_COMMAND_LENGTH];
  unsigned char buf[5];
  int i;

  if(zoom < 0)
    zoom = 0;

  else
    if(zoom > max_zoom_){
      zoom = max_zoom_;
    }

  command[0] = HEADER;
  command[1] = DEVICEID;
  command[2] = DEVICEID;
  command[3] = DELIM;
  command[4] = ZOOM;

  sprintf((char *)buf, "%4X", zoom);

  for (i=0;i<3;i++)
    if (buf[i] == ' ')
      buf[i] = '0';

  // zoom position
  command[5] = buf[0];
  command[6] = buf[1];
  command[7] = buf[2];
  command[8] = buf[3];
  command[9] = FOOTER;

  zoom_ = zoom;

  if (sendCommand(command, 10))
    return -1;
  if (bidirectional_com_)
  {
    return (receiveCommandAnswer(COMMAND_RESPONSE_BYTES));
  }
  else
  {
    usleep(SLEEP_TIME_USEC);
    return 0;
  }
  //return (receiveCommandAnswer(COMMAND_RESPONSE_BYTES));
}

int P2OSPtz::setDefaultTiltRange()
{
  unsigned char command[MAX_COMMAND_LENGTH];
  unsigned char buf[5]; //4 values and string terminator
  int maxtilt, mintilt;

  command[0] = HEADER;
  command[1] = DEVICEID;
  command[2] = DEVICEID;
  command[3] = DELIM;
  command[4] = SETRANGE;
  command[5] = DEVICEID+1;

  mintilt = (int)(floor(MIN_TILT/.1125) + 0x8000);
  sprintf((char*)buf, "%X", mintilt);
  command[6] = buf[0];
  command[7] = buf[1];
  command[8] = buf[2];
  command[9] = buf[3];
  maxtilt = (int)(floor(MAX_TILT/.1125) + 0x8000);
  sprintf((char*)buf, "%X", maxtilt);

  command[10] = buf[0];
  command[11] = buf[1];
  command[12] = buf[2];
  command[13] = buf[3];
  command[14] = FOOTER;

  if(sendCommand(command, 15))
    return -1;
  if (bidirectional_com_)
  {
    return (receiveCommandAnswer(COMMAND_RESPONSE_BYTES));
  }
  else
  {
    usleep(SLEEP_TIME_USEC);
    return 0;
  }

  // return(receiveCommandAnswer(COMMAND_RESPONSE_BYTES));

}

int P2OSPtz::getAbsPanTilt(int* pan, int* tilt)
{
  unsigned char command[MAX_COMMAND_LENGTH];
  unsigned char reply[MAX_REQUEST_LENGTH];
  int reply_len;
  unsigned char buf[4];
  char byte;
  unsigned int u_val;
  int val;
  int i;

  command[0] = HEADER;
  command[1] = DEVICEID;
  command[2] = DEVICEID;
  command[3] = DELIM;
  command[4] = PANTILTREQ;
  command[5] = FOOTER;

  if (sendRequest(command, 6, reply))
    return(-1);
  if (bidirectional_com_)
  {
    reply_len = receiveRequestAnswer(reply,14,0);
  }
  else
  {
    return 0;
  }

  if ( reply_len != 14 ) {
    ROS_ERROR("Reply Len = %i; should equal 14", reply_len);
    return -1;
  }

  // remove the ascii encoding, and put into 4-byte array
  for (i = 0; i < 4; i++)
  {
    byte = reply[i+5];
    if (byte < 0x40)
      byte = byte - 0x30;
    else
      byte = byte - 'A' + 10;
    buf[i] = byte;
  }

  // convert the 4-bytes into a number
  u_val = buf[0] * 0x1000 + buf[1] * 0x100 + buf[2] * 0x10 + buf[3];

  // convert the number to a value that's meaningful, based on camera specs
  val = (int)(((int)u_val - (int)0x8000) * 0.1125);

  // now set myPan to the response received for where the camera thinks it is
  *pan = val;

  // repeat the steps for the tilt value
  for (i = 0; i < 4; i++)
  {
    byte = reply[i+9];
    if (byte < 0x40)
      byte = byte - 0x30;
    else
      byte = byte - 'A' + 10;
    buf[i] = byte;
  }
  u_val = buf[0] * 0x1000 + buf[1] * 0x100 + buf[2] * 0x10 + buf[3];
  val =(int)(((int)u_val  - (int)0x8000) * 0.1125);
  *tilt = val;

  return(0);
}

int P2OSPtz::sendAbsPanTilt(int pan, int tilt)
{
  unsigned char command[MAX_COMMAND_LENGTH];
  int convpan, convtilt;
  unsigned char buf[5];
  int ppan, ttilt;

  ppan = pan; ttilt = tilt;
  if(pan < MIN_PAN)
  {
    ppan = (int)MIN_PAN;
  }
  else if(pan > MAX_PAN)
  {
      ppan = (int)MAX_PAN;
  }

  if (tilt > MAX_TILT)
  {
    ttilt = (int)MAX_TILT;
  }
  else if(tilt < MIN_TILT)
  {
      ttilt = (int)MIN_TILT;
  }
  //puts("Camera pan angle thresholded");

  //puts("Camera tilt angle thresholded");

  convpan = (int)floor(ppan/.1125) + 0x8000;
  convtilt = (int)floor(ttilt/.1125) + 0x8000;
  //   fprintf(stdout, "ppan: %d ttilt: %d conpan: %d contilt: %d\n",
  // 	  ppan,ttilt,convpan,convtilt);
  command[0] = HEADER;
  command[1] = DEVICEID;
  command[2] = DEVICEID;
  command[3] = DELIM;
  command[4] = PANTILT;
  // pan position

  snprintf((char *)buf,sizeof(buf), "%X", convpan);

  command[5] = buf[0];
  command[6] = buf[1];
  command[7] = buf[2];
  command[8] = buf[3];
  // tilt position
  snprintf((char *)buf,sizeof(buf), "%X", convtilt);
  command[9]  = buf[0];
  command[10] = buf[1];
  command[11] = buf[2];
  command[12] = buf[3];
  command[13] = (unsigned char) FOOTER;
  if(sendCommand(command, 14))
    return -1;

  tilt_ = ttilt;
  pan_ = ppan;

  if (bidirectional_com_)
  {
    return (receiveCommandAnswer(COMMAND_RESPONSE_BYTES));
  }
  else
  {
    usleep(SLEEP_TIME_USEC);
    return 0;
  }

  //return(receiveCommandAnswer(COMMAND_RESPONSE_BYTES));
}

//
// Circular Buffer To deal with getting data back from AUX
//
circbuf::circbuf(int size) :
    start(0), end(0), mysize(size), gotPack(false)
{
  this->buf = new unsigned char[size];
}

void circbuf::printBuf(){
  int i = start;
  printf("circbuf: ");
  while ( i != end ){
    printf("0x%x ", buf[i]);
    i = (i+1)%mysize;
  }
  printf("\n");
}


void circbuf::putOnBuf(unsigned char c)
{
  buf[end] = c;
  end = (end+1)%mysize;
  if ( end == start )
  {
    // We're overwriting old data.
    start = (start + 1)%mysize;
  }

  // Check to see if we have the whole packet now. (ends with FOOTER)
  if ( c == P2OSPtz::FOOTER )
  {
    gotPack = true;
  }
}

bool circbuf::haveData()
{
  return !(this->start == this->end);
}

int circbuf::getFromBuf()
{
  if ( start != end ){
    unsigned char ret = buf[start];
    start = (start+1)%mysize;
    return (int)ret;
  }
  else
  {
    return -1;
  }
}

int circbuf::size()
{
  if ( end > start )
  {
    return end-start;
  }
  else if ( start > end )
  {
    return mysize - start - end - 1;
  }
  else
  {
    return 0;
  }
}

bool circbuf::gotPacket()
{
  return gotPack;
}

void circbuf::reset()
{
  memset(buf, 0, mysize);
  gotPack = false;
  start = end = 0;
}
