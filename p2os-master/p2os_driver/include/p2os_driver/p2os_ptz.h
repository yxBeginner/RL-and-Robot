/*
 *  P2OS for ROS
 *  Copyright (C) 2010
 *     Tucker Hermans, David Feil-Seifer, Brian Gerkey, Kasper Stoy,
 *     Richard Vaughan, & Andrew Howard
 * Copyright (C) 2004, 2005 ActivMedia Robotics LLC
 * Copyright (C) 2006, 2007, 2008, 2009 MobileRobots Inc.
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
#ifndef _P2OS_PTZ_H
#define _P2OS_PTZ_H

#include <p2os_msgs/PTZState.h>
#include <p2os_driver/packet.h>
#include <p2os_driver/robot_params.h>

class P2OSNode;

// Circular Buffer Used by PTZ camera
class circbuf
{
 public:
  circbuf(int size=512);

  void putOnBuf(unsigned char c);
  int  getFromBuf();
  bool haveData();
  int  size();
  void printBuf();

  bool gotPacket();
  void reset();

 private:
  unsigned char* buf;
  int  start;
  int  end;
  int  mysize;
  bool gotPack;
};

class P2OSPtz
{
 public:
  enum Command {
    DELIM = 0x00, ///<Delimeter character
    DEVICEID = 0x30, ///<Default device ID
    PANSLEW = 0x50, ///<Sets the pan slew
    TILTSLEW = 0x51, ///<Sets the tilt slew
    STOP = 0x53, ///<Stops current pan/tilt motion
    INIT = 0x58, ///<Initializes the camera
    SLEWREQ = 0x59, ///<Request pan/tilt min/max slew
    ANGLEREQ = 0x5c, ///<Request pan/tilt min/max angle
    PANTILT = 0x62, ///<Pan/tilt command
    SETRANGE = 0x64, ///<Pan/tilt min/max range assignment
    PANTILTREQ = 0x63, ///<Request pan/tilt position
    INFRARED = 0x76, ///<Controls operation of IR lighting
    PRODUCTNAME = 0x87, ///<Requests the product name
    LEDCONTROL = 0x8E, ///<Controls LED status
    CONTROL = 0x90, ///<Puts camera in Control mode
    POWER = 0xA0, ///<Turns on/off power
    AUTOFOCUS = 0xA1, ///<Controls auto-focusing functions
    ZOOMSTOP = 0xA2, ///<Stops zoom motion
    GAIN = 0xA5, ///<Sets gain adjustment on camera
    FOCUS = 0xB0, ///<Manual focus adjustment
    ZOOM = 0xB3, ///<Zooms camera lens
    ZOOMREQ = 0xB4, ///<Requests max zoom position
    IRCUTFILTER = 0xB5, ///<Controls the IR cut filter
    DIGITALZOOM = 0xB7, ///<Controls the digital zoom amount
    FOOTER = 0xEF, ///<Packet Footer
    RESPONSE = 0xFE, ///<Packet header for response
    HEADER = 0xFF ///<Packet Header
  };

  // the states for communication
  enum CommState {
    COMM_UNKNOWN,
    COMM_BIDIRECTIONAL,
    COMM_UNIDIRECTIONAL
  };

  enum CameraType {
    CAMERA_VCC4,
    CAMERA_C50I
  };

 protected:
  // preset limits on movements.  Based on empirical data
  enum Param {
    MAX_PAN = 98,		// 875 units is max pan assignment
    MIN_PAN = -98,		// -875 units is min pan assignment
    MAX_TILT = 88,		// 790 units is max tilt assignment
    MIN_TILT = -30,		// -267 units is min tilt assignment
    MAX_PAN_SLEW = 90,		// 800 positions per sec (PPS)
    MIN_PAN_SLEW = 1,		// 8 positions per sec (PPS)
    MAX_TILT_SLEW = 69,		// 662 positions per sec (PPS)
    MIN_TILT_SLEW = 1,		// 8 positions per sec (PPS)
    MAX_ZOOM_OPTIC = 1960,
    MIN_ZOOM = 0
  };

  // the various error states that the camera can return
  enum Error {
    CAM_ERROR_NONE = 0x30, ///<No error
    CAM_ERROR_BUSY = 0x31, ///<Camera busy, will not execute the command
    CAM_ERROR_PARAM = 0x35, ///<Illegal parameters to function call
    CAM_ERROR_MODE = 0x39,  ///<Not in host control mode
    CAM_ERROR_UNKNOWN = 0xFF ///<Unknown error condition.  Should never happen
  };

  // Types for turning on and off the camera
  enum Power {
    POWER_OFF = 0,
    POWER_ON = 1
  };

 public:
  // Constructor
  P2OSPtz (P2OSNode* p2os, bool bidirectional_com = false);

  // Core Functions
  int setup();
  void shutdown();
  void callback(const p2os_msgs::PTZStateConstPtr &msg);

  // Communication Functions
  int sendCommand(unsigned char *str, int len);
  int sendRequest(unsigned char *str, int len, unsigned char* reply);
  int receiveCommandAnswer(int asize);
  int receiveRequestAnswer(unsigned char *data, int s1, int s2);
  void getPtzPacket(int s1, int s2=0);

  // Device Command Functions
  int setPower(Power on);
  int setControlMode();
  int sendInit();

  int getMaxZoom(int * max_zoom);
  int getAbsZoom(int* zoom);
  int sendAbsZoom(int zoom);

  int setDefaultTiltRange();
  int getAbsPanTilt(int* pan, int* tilt);
  int sendAbsPanTilt(int pan, int tilt);

  // Simple getters and setters
  bool isOn() const { return is_on_; }
  p2os_msgs::PTZState getCurrentState() { return current_state_; }

  // Class members
 protected:
  P2OSNode* p2os_;
 public:
  circbuf cb_;
 protected:
  int max_zoom_;
  int pan_, tilt_, zoom_;
  bool is_on_;
  int error_code_;
  bool bidirectional_com_;
  p2os_msgs::PTZState current_state_;

  // Class constants
  static const int MAX_COMMAND_LENGTH;
  static const int MAX_REQUEST_LENGTH;
  static const int COMMAND_RESPONSE_BYTES;
  static const int PACKET_TIMEOUT;
  static const int SLEEP_TIME_USEC;
  static const int PAN_THRESH;
  static const int TILT_THRESH;
  static const int ZOOM_THRESH;
};

#endif
