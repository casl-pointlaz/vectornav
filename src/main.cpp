/*
 * MIT License (MIT)
 *
 * Copyright (c) 2018 Dereck Wonnacott <dereck@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 */

#include <cmath>
#include <iostream>

// No need to define PI twice if we already have it included...
//#define M_PI 3.14159265358979323846  /* M_PI */

// ROS Libraries
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vectornav/Ins.h>

#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/FluidPressure.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Temperature.h"
#include "std_srvs/Empty.h"

#include <vectornav/ImuWithCount.h>
#include <std_msgs/Int8.h>

ros::Subscriber subScannerState;
ros::Publisher pubIMU, pubMag, pubGPS, pubOdom, pubTemp, pubPres, pubIns;
ros::ServiceServer resetOdomSrv;

XmlRpc::XmlRpcValue rpc_temp;

// Include this header file to get access to VectorNav sensors.
#include "vn/compositedata.h"
#include "vn/sensors.h"
#include "vn/util.h"

using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

// Method declarations for future use.
void BinaryAsyncMessageReceived(void * userData, Packet & p, size_t index);
vn::protocol::uart::SyncInMode syncInModeSetUp(int sync_in_mode);
vn::protocol::uart::SyncInEdge syncInEdgeSetUp(int sync_in_edge);
uint16_t syncInSkipFactor(int sync_in_skip_factor);
vn::protocol::uart::CommonGroup getCommonGroupSetUp(ros::NodeHandle pn);
vn::protocol::uart::TimeGroup getTimeGroupSetUp(ros::NodeHandle pn);
vn::protocol::uart::ImuGroup getImuGroupSetUp(ros::NodeHandle pn);
vn::protocol::uart::GpsGroup getGpsGroupSetUp(ros::NodeHandle pn);
vn::protocol::uart::GpsGroup getGps2GroupSetUp(ros::NodeHandle pn);
vn::protocol::uart::AttitudeGroup getAttitudeGroupSetUp(ros::NodeHandle pn);
vn::protocol::uart::InsGroup getInsGroupSetUp(ros::NodeHandle pn);

// Create a VnSensor object and connect to sensor
VnSensor vs;

// Custom user data to pass to packet callback function
struct UserData
{
  // the vectornav device identifier
  int device_family;
  // frame id used only for Odom header.frame_id
  std::string map_frame_id;
  // frame id used for header.frame_id of other messages and for Odom child_frame_id
  std::string frame_id;
  // Boolean to use ned or enu frame. Defaults to enu which is data format from sensor.
  bool tf_ned_to_enu;
  bool frame_based_enu;
  // Initial position after getting a GPS fix.
  vec3d initial_position;
  bool initial_position_set = false;

  //Unused covariances initialized to zero's
  boost::array<double, 9ul> linear_accel_covariance = {};
  boost::array<double, 9ul> angular_vel_covariance = {};
  boost::array<double, 9ul> orientation_covariance = {};

  // ROS header time stamp adjustments
  double average_time_difference{0};
  ros::Time ros_start_time;
  bool adjust_ros_timestamp{false};
  int timestamp_type;

  // Use IMU msg with count field or not
  bool use_imu_with_syncincount_msg{false};

  // strides
  unsigned int imu_stride;
  unsigned int output_stride;
};

// Callback for /scanner_state topic : reset the SyncInCount when scanner_state == idling (0)
void callbackScannerState(const std_msgs::Int8::ConstPtr& scanner_state_msg)
{
  // If scanner_state == idling
  if(scanner_state_msg->data == 0)
  {
    // Sleep to wait for the reset
    usleep(100000);
    // Reset the SyncInCount, SyncInTime, and SyncOutCount to 0 (useful for the SyncInCount sync with MCU count)
    vs.writeSynchronizationStatus(0, 0, 0);
  }
}

// Basic loop so we can initilize our covariance parameters above
boost::array<double, 9ul> setCov(XmlRpc::XmlRpcValue rpc)
{
  // Output covariance vector
  boost::array<double, 9ul> output = {0.0};

  // Convert the RPC message to array
  ROS_ASSERT(rpc.getType() == XmlRpc::XmlRpcValue::TypeArray);

  for (int i = 0; i < 9; i++) {
    ROS_ASSERT(rpc[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    output[i] = (double)rpc[i];
  }
  return output;
}

// Reset initial position to current position
bool resetOdom(
  std_srvs::Empty::Request & req, std_srvs::Empty::Response & resp, UserData * user_data)
{
  ROS_INFO("Reset Odometry");
  user_data->initial_position_set = false;
  return true;
}

// Assure that the serial port is set to async low latency in order to reduce delays and package pilup.
// These changes will stay effective until the device is unplugged
#if __linux__ || __CYGWIN__
#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
bool optimize_serial_communication(std::string portName)
{
  int portFd = -1;

  portFd = ::open(portName.c_str(), O_RDWR | O_NOCTTY);

  if (portFd == -1) {
    ROS_WARN("Can't open port for optimization");
    return false;
  }

  ROS_INFO("Set port to ASYNCY_LOW_LATENCY");
  struct serial_struct serial;
  ioctl(portFd, TIOCGSERIAL, &serial);
  serial.flags |= ASYNC_LOW_LATENCY;
  ioctl(portFd, TIOCSSERIAL, &serial);
  ::close(portFd);
  return true;
}
#elif
bool optimize_serial_communication(str::string portName) { return true; }
#endif

int main(int argc, char * argv[])
{
  // keeping all information passed to callback
  UserData user_data;

  // ROS node init
  ros::init(argc, argv, "vectornav");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  resetOdomSrv = n.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(
    "reset_odom", boost::bind(&resetOdom, _1, _2, &user_data));

  // Serial Port Settings
  string SensorPort;
  int SensorBaudrate;
  int async_output_rate;
  int imu_output_rate;

  // Sensor IMURATE (800Hz by default, used to configure device)
  int SensorImuRate;

  int async_mode;

  int sync_in_mode, sync_in_edge, sync_in_skip_factor;

  // Load all params
  pn.param<std::string>("map_frame_id", user_data.map_frame_id, "map");
  pn.param<std::string>("frame_id", user_data.frame_id, "vectornav");
  pn.param<bool>("tf_ned_to_enu", user_data.tf_ned_to_enu, false);
  pn.param<bool>("frame_based_enu", user_data.frame_based_enu, false);
  pn.param<bool>("adjust_ros_timestamp", user_data.adjust_ros_timestamp, false);
  pn.param<int>("async_output_rate", async_output_rate, 40);
  pn.param<int>("imu_output_rate", imu_output_rate, async_output_rate);
  pn.param<std::string>("serial_port", SensorPort, "/dev/ttyUSB0");
  pn.param<int>("serial_baud", SensorBaudrate, 115200);
  pn.param<int>("fixed_imu_rate", SensorImuRate, 800);
  pn.param<int>("async_mode", async_mode, 2);
  pn.param<int>("timestamp_type", user_data.timestamp_type, 2);
  pn.param<bool>("use_imu_with_syncincount_msg", user_data.use_imu_with_syncincount_msg, true);
  pn.param<int>("sync_in_mode", sync_in_mode, 3);
  pn.param<int>("sync_in_edge", sync_in_edge, 0);
  pn.param<int>("sync_in_skip_factor", sync_in_skip_factor, 1);

  //Call to set covariances
  if (pn.getParam("linear_accel_covariance", rpc_temp)) {
    user_data.linear_accel_covariance = setCov(rpc_temp);
  }
  if (pn.getParam("angular_vel_covariance", rpc_temp)) {
    user_data.angular_vel_covariance = setCov(rpc_temp);
  }
  if (pn.getParam("orientation_covariance", rpc_temp)) {
    user_data.orientation_covariance = setCov(rpc_temp);
  }

  ROS_INFO("Connecting to : %s @ %d Baud", SensorPort.c_str(), SensorBaudrate);

  // try to optimize the serial port
  optimize_serial_communication(SensorPort);


  // Default baudrate variable
  int defaultBaudrate;
  // Run through all of the acceptable baud rates until we are connected
  // Looping in case someone has changed the default
  bool baudSet = false;
  // Lets add the set baudrate to the top of the list, so that it will try
  // to connect with that value first (speed initialization up)
  std::vector<unsigned int> supportedBaudrates = vs.supportedBaudrates();
  supportedBaudrates.insert(supportedBaudrates.begin(), SensorBaudrate);
  while (!baudSet)
  {
    // Make this variable only accessible in the while loop
    static int i = 0;
    defaultBaudrate = supportedBaudrates[i];
    ROS_INFO("Connecting with default at %d", defaultBaudrate);
    // Default response was too low and retransmit time was too long by default.
    // They would cause errors
    vs.setResponseTimeoutMs(100);  // Wait for up to 1000 ms for response
    vs.setRetransmitDelayMs(50);    // Retransmit every 50 ms

    // Acceptable baud rates 9600, 19200, 38400, 57600, 128000, 115200, 230400, 460800, 921600
    // Data sheet says 128000 is a valid baud rate. It doesn't work with the VN100 so it is excluded.
    // All other values seem to work fine.
    try
    {
      // Connect to sensor at it's default rate
      if (defaultBaudrate != 128000 && SensorBaudrate != 128000)
      {
        vs.connect(SensorPort, defaultBaudrate);
        // Issues a change baudrate to the VectorNav sensor and then
        // reconnects the attached serial port at the new baudrate.
        vs.changeBaudRate(SensorBaudrate);
        // Only makes it here once we have the default correct
        ROS_INFO("Connected baud rate is %d", vs.baudrate());
        baudSet = true;
      }
    }
    // Catch all oddities
    catch (...)
    {
      // Disconnect if we had the wrong default and we were connected
      vs.disconnect();
      ros::Duration(0.2).sleep();
    }
    // Increment the default iterator
    i++;
    // There are only 9 available data rates, if no connection
    // made yet possibly a hardware malfunction?
    if (i > 8)
    {
      ROS_INFO("No baud rate found. Baud Rate Scan restarted");
      i = 0;
    }
  }

  // Now we verify connection (Should be good if we made it this far)
  if (vs.verifySensorConnectivity()) {
    ROS_INFO("Device connection established");
  } else {
    ROS_ERROR("No device communication");
    ROS_WARN("Please input a valid baud rate. Valid are:");
    ROS_WARN("9600, 19200, 38400, 57600, 115200, 128000, 230400, 460800, 921600");
    ROS_WARN("With the test IMU 128000 did not work, all others worked fine.");
  }
  // Query the sensor's model number.
  string mn = vs.readModelNumber();
  string fv = vs.readFirmwareVersion();
  uint32_t hv = vs.readHardwareRevision();
  uint32_t sn = vs.readSerialNumber();
  ROS_INFO("Model Number: %s, Firmware Version: %s", mn.c_str(), fv.c_str());
  ROS_INFO("Hardware Revision : %d, Serial Number : %d", hv, sn);

  // calculate the least common multiple of the two rate and assure it is a
  // valid package rate, also calculate the imu and output strides
  int package_rate = 0;
  for (int allowed_rate : {800, 1, 2, 4, 5, 10, 20, 25, 40, 50, 100, 200, 400, 0}) {
    package_rate = allowed_rate;
    if ((package_rate % async_output_rate) == 0 && (package_rate % imu_output_rate) == 0) break;
  }
  ROS_ASSERT_MSG(
    package_rate,
    "imu_output_rate (%d) or async_output_rate (%d) is not in 1, 2, 4, 5, 10, 20, 25, 40, 50, 100, "
    "200 Hz",
    imu_output_rate, async_output_rate);
  user_data.imu_stride = package_rate / imu_output_rate;
  user_data.output_stride = package_rate / async_output_rate;
  ROS_INFO("Package Receive Rate: %d Hz", package_rate);
  ROS_INFO("General Publish Rate: %d Hz", async_output_rate);
  ROS_INFO("IMU Publish Rate: %d Hz", imu_output_rate);

  // SyncIn SetUp
  vn::protocol::uart::SyncInMode uart_sync_in_mode =  syncInModeSetUp(sync_in_mode);
  vn::protocol::uart::SyncInEdge uart_sync_in_edge = syncInEdgeSetUp(sync_in_edge);
  uint16_t uart_sync_in_skip_factor = syncInSkipFactor(sync_in_skip_factor);
  vs.writeSynchronizationControl(uart_sync_in_mode, uart_sync_in_edge, uart_sync_in_skip_factor, SYNCOUTMODE_NONE, SYNCOUTPOLARITY_NEGATIVE, 1, 100000000, true);

  // Reset the SyncInCount, SyncInTime, and SyncOutCount to 0 (useful for the SyncInCount sync with MCU count)
  vs.writeSynchronizationStatus(0, 0, 0);


  // Binary Group SetUp
  vn::protocol::uart::CommonGroup commonGroupSetUp = getCommonGroupSetUp(pn);
  vn::protocol::uart::TimeGroup timeGroupSetUp = getTimeGroupSetUp(pn);
  vn::protocol::uart::ImuGroup imuGroupSetUp = getImuGroupSetUp(pn);
  vn::protocol::uart::GpsGroup gpsGroupSetUp = getGpsGroupSetUp(pn);
  vn::protocol::uart::GpsGroup gps2GroupSetUp = getGps2GroupSetUp(pn);
  vn::protocol::uart::AttitudeGroup attitudeGroupSetUp = getAttitudeGroupSetUp(pn);
  vn::protocol::uart::InsGroup insGroupSetUp = getInsGroupSetUp(pn);

  // Set the device info for passing to the packet callback function
  user_data.device_family = vs.determineDeviceFamily();

  // Declare subscriber
  subScannerState = n.subscribe("/scanner_state", 1000, callbackScannerState);

  // Declare publishers
  if (user_data.use_imu_with_syncincount_msg)   // Publisher declaration depending on wanted IMU msg
    pubIMU = n.advertise<vectornav::ImuWithCount>("vectornav/IMU", 1000);
  else
    pubIMU = n.advertise<sensor_msgs::Imu>("vectornav/IMU", 1000);

  if ((commonGroupSetUp & COMMONGROUP_MAGPRES) || (imuGroupSetUp & IMUGROUP_MAG))
    pubMag = n.advertise<sensor_msgs::MagneticField>("vectornav/Mag", 1000);
  if ((commonGroupSetUp & COMMONGROUP_MAGPRES) || (imuGroupSetUp & IMUGROUP_TEMP))
    pubTemp = n.advertise<sensor_msgs::Temperature>("vectornav/Temp", 1000);
  if ((commonGroupSetUp & COMMONGROUP_MAGPRES) || (imuGroupSetUp & IMUGROUP_PRES))
    pubPres = n.advertise<sensor_msgs::FluidPressure>("vectornav/Pres", 1000);

  if (user_data.device_family != VnSensor::Family::VnSensor_Family_Vn100)
    pubOdom = n.advertise<nav_msgs::Odometry>("vectornav/Odom", 1000);
  if (user_data.device_family != VnSensor::Family::VnSensor_Family_Vn100 && (gpsGroupSetUp != 0 || gps2GroupSetUp != 0))
    pubGPS = n.advertise<sensor_msgs::NavSatFix>("vectornav/GPS", 1000);
  if (user_data.device_family != VnSensor::Family::VnSensor_Family_Vn100 && insGroupSetUp != 0)
    pubIns = n.advertise<vectornav::Ins>("vectornav/INS", 1000);

  // Make sure no generic async output is registered
  vs.writeAsyncDataOutputType(VNOFF);

  // Configure binary output message
  /*BinaryOutputRegister bor(
    async_mode,
    SensorImuRate / 800,  // update rate [ms]
    COMMONGROUP_QUATERNION | COMMONGROUP_YAWPITCHROLL | COMMONGROUP_ANGULARRATE |
      COMMONGROUP_POSITION | COMMONGROUP_ACCEL | COMMONGROUP_MAGPRES |
      (user_data.adjust_ros_timestamp ? COMMONGROUP_TIMESTARTUP : 0),
    TIMEGROUP_NONE | TIMEGROUP_GPSTOW | TIMEGROUP_GPSWEEK | TIMEGROUP_TIMEUTC, IMUGROUP_NONE,
    GPSGROUP_NONE,
    ATTITUDEGROUP_YPRU,  //<-- returning yaw pitch roll uncertainties
    INSGROUP_INSSTATUS | INSGROUP_POSECEF | INSGROUP_VELBODY | INSGROUP_ACCELECEF |
      INSGROUP_VELNED | INSGROUP_POSU | INSGROUP_VELU,
    GPSGROUP_NONE); */


  BinaryOutputRegister bor(
    async_mode,
    SensorImuRate / package_rate,  // update rate [ms]
    commonGroupSetUp,
    timeGroupSetUp,
    imuGroupSetUp,
    gpsGroupSetUp,
    attitudeGroupSetUp,
    insGroupSetUp,
    gps2GroupSetUp);

  // An empty output register for disabling output 2 and 3 if previously set
  BinaryOutputRegister bor_none(
    0, 1, COMMONGROUP_NONE, TIMEGROUP_NONE, IMUGROUP_NONE, GPSGROUP_NONE, ATTITUDEGROUP_NONE,
    INSGROUP_NONE, GPSGROUP_NONE);

  vs.writeBinaryOutput1(bor);
  vs.writeBinaryOutput2(bor_none);
  vs.writeBinaryOutput3(bor_none);

  // Register async callback function
  vs.registerAsyncPacketReceivedHandler(&user_data, BinaryAsyncMessageReceived);

  // You spin me right round, baby
  // Right round like a record, baby
  // Right round round round
  while (ros::ok()) {
    ros::spin();  // Need to make sure we disconnect properly. Check if all ok.
  }

  // Node has been terminated
  vs.unregisterAsyncPacketReceivedHandler();
  ros::Duration(0.5).sleep();
  ROS_INFO("Unregisted the Packet Received Handler");
  vs.disconnect();
  ros::Duration(0.5).sleep();
  ROS_INFO("%s is disconnected successfully", mn.c_str());
  return 0;
}

//Helper function to create IMU message
void fill_imu_message(
  sensor_msgs::Imu & msgIMU, vn::sensors::CompositeData & cd, ros::Time & time,
  UserData * user_data)
{
  msgIMU.header.stamp = time;
  msgIMU.header.frame_id = user_data->frame_id;

  if (cd.hasQuaternion() && cd.hasAngularRate() && cd.hasAcceleration()) {
    vec4f q = cd.quaternion();
    vec3f ar = cd.angularRate();
    vec3f al = cd.acceleration();

    if (cd.hasAttitudeUncertainty()) {
      vec3f orientationStdDev = cd.attitudeUncertainty();
      msgIMU.orientation_covariance[0] =
        pow(orientationStdDev[2] * M_PI / 180, 2);  // Convert to radians Roll
      msgIMU.orientation_covariance[4] =
        pow(orientationStdDev[1] * M_PI / 180, 2);  // Convert to radians Pitch
      msgIMU.orientation_covariance[8] =
        pow(orientationStdDev[0] * M_PI / 180, 2);  // Convert to radians Yaw
    }

    //Quaternion message comes in as a Yaw (z) pitch (y) Roll (x) format
    if (user_data->tf_ned_to_enu) {
      // If we want the orientation to be based on the reference label on the imu
      tf2::Quaternion tf2_quat(q[0], q[1], q[2], q[3]);
      geometry_msgs::Quaternion quat_msg;

      if (user_data->frame_based_enu) {
        // Create a rotation from NED -> ENU
        tf2::Quaternion q_rotate;
        q_rotate.setRPY(M_PI, 0.0, M_PI / 2);
        // Apply the NED to ENU rotation such that the coordinate frame matches
        tf2_quat = q_rotate * tf2_quat;
        quat_msg = tf2::toMsg(tf2_quat);

        // Since everything is in the normal frame, no flipping required
        msgIMU.angular_velocity.x = ar[0];
        msgIMU.angular_velocity.y = ar[1];
        msgIMU.angular_velocity.z = ar[2];

        msgIMU.linear_acceleration.x = al[0];
        msgIMU.linear_acceleration.y = al[1];
        msgIMU.linear_acceleration.z = al[2];
      } else {
        // put into ENU - swap X/Y, invert Z
        quat_msg.x = q[1];
        quat_msg.y = q[0];
        quat_msg.z = -q[2];
        quat_msg.w = q[3];

        // Flip x and y then invert z
        msgIMU.angular_velocity.x = ar[1];
        msgIMU.angular_velocity.y = ar[0];
        msgIMU.angular_velocity.z = -ar[2];
        // Flip x and y then invert z
        msgIMU.linear_acceleration.x = al[1];
        msgIMU.linear_acceleration.y = al[0];
        msgIMU.linear_acceleration.z = -al[2];

        if (cd.hasAttitudeUncertainty()) {
          vec3f orientationStdDev = cd.attitudeUncertainty();
          msgIMU.orientation_covariance[0] =
            pow(orientationStdDev[1] * M_PI / 180, 2);  // Convert to radians pitch
          msgIMU.orientation_covariance[4] =
            pow(orientationStdDev[0] * M_PI / 180, 2);  // Convert to radians Roll
          msgIMU.orientation_covariance[8] =
            pow(orientationStdDev[2] * M_PI / 180, 2);  // Convert to radians Yaw
        }
      }

      msgIMU.orientation = quat_msg;
    } else {
      msgIMU.orientation.x = q[0];
      msgIMU.orientation.y = q[1];
      msgIMU.orientation.z = q[2];
      msgIMU.orientation.w = q[3];

      msgIMU.angular_velocity.x = ar[0];
      msgIMU.angular_velocity.y = ar[1];
      msgIMU.angular_velocity.z = ar[2];
      msgIMU.linear_acceleration.x = al[0];
      msgIMU.linear_acceleration.y = al[1];
      msgIMU.linear_acceleration.z = al[2];
    }
    // Covariances pulled from parameters
    msgIMU.angular_velocity_covariance = user_data->angular_vel_covariance;
    msgIMU.linear_acceleration_covariance = user_data->linear_accel_covariance;
  }
}

//Helper function to create magnetic field message
void fill_mag_message(
  sensor_msgs::MagneticField & msgMag, vn::sensors::CompositeData & cd, ros::Time & time,
  UserData * user_data)
{
  msgMag.header.stamp = time;
  msgMag.header.frame_id = user_data->frame_id;

  // Magnetic Field
  if (cd.hasMagnetic()) {
    vec3f mag = cd.magnetic();
    msgMag.magnetic_field.x = mag[0];
    msgMag.magnetic_field.y = mag[1];
    msgMag.magnetic_field.z = mag[2];
  }
}

//Helper function to create gps message
void fill_gps_message(
  sensor_msgs::NavSatFix & msgGPS, vn::sensors::CompositeData & cd, ros::Time & time,
  UserData * user_data)
{
  msgGPS.header.stamp = time;
  msgGPS.header.frame_id = user_data->frame_id;

  if (cd.hasPositionEstimatedLla()) {
    vec3d lla = cd.positionEstimatedLla();

    msgGPS.latitude = lla[0];
    msgGPS.longitude = lla[1];
    msgGPS.altitude = lla[2];

    // Read the estimation uncertainty (1 Sigma) from the sensor and write it to the covariance matrix.
    if (cd.hasPositionUncertaintyEstimated()) {
      double posVariance = pow(cd.positionUncertaintyEstimated(), 2);
      msgGPS.position_covariance[0] = posVariance;  // East position variance
      msgGPS.position_covariance[4] = posVariance;  // North position vaciance
      msgGPS.position_covariance[8] = posVariance;  // Up position variance

      // mark gps fix as not available if the outputted standard deviation is 0
      if (cd.positionUncertaintyEstimated() != 0.0) {
        // Position available
        msgGPS.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
      } else {
        // position not detected
        msgGPS.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
      }

      // add the type of covariance to the gps message
      msgGPS.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    } else {
      msgGPS.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    }
  }
}

//Helper function to create odometry message
void fill_odom_message(
  nav_msgs::Odometry & msgOdom, vn::sensors::CompositeData & cd, ros::Time & time,
  UserData * user_data)
{
  msgOdom.header.stamp = time;
  msgOdom.child_frame_id = user_data->frame_id;
  msgOdom.header.frame_id = user_data->map_frame_id;

  if (cd.hasPositionEstimatedEcef()) {
    // add position as earth fixed frame
    vec3d pos = cd.positionEstimatedEcef();

    if (!user_data->initial_position_set) {
      ROS_INFO("Set initial position to %f %f %f", pos[0], pos[1], pos[2]);
      user_data->initial_position_set = true;
      user_data->initial_position.x = pos[0];
      user_data->initial_position.y = pos[1];
      user_data->initial_position.z = pos[2];
    }

    msgOdom.pose.pose.position.x = pos[0] - user_data->initial_position[0];
    msgOdom.pose.pose.position.y = pos[1] - user_data->initial_position[1];
    msgOdom.pose.pose.position.z = pos[2] - user_data->initial_position[2];

    // Read the estimation uncertainty (1 Sigma) from the sensor and write it to the covariance matrix.
    if (cd.hasPositionUncertaintyEstimated()) {
      double posVariance = pow(cd.positionUncertaintyEstimated(), 2);
      msgOdom.pose.covariance[0] = posVariance;   // x-axis position variance
      msgOdom.pose.covariance[7] = posVariance;   // y-axis position vaciance
      msgOdom.pose.covariance[14] = posVariance;  // z-axis position variance
    }
  }

  if (cd.hasQuaternion()) {
    vec4f q = cd.quaternion();

    if (!user_data->tf_ned_to_enu) {
      // output in NED frame
      msgOdom.pose.pose.orientation.x = q[0];
      msgOdom.pose.pose.orientation.y = q[1];
      msgOdom.pose.pose.orientation.z = q[2];
      msgOdom.pose.pose.orientation.w = q[3];
    } else if (user_data->tf_ned_to_enu && user_data->frame_based_enu) {
      // standard conversion from NED to ENU frame
      tf2::Quaternion tf2_quat(q[0], q[1], q[2], q[3]);
      // Create a rotation from NED -> ENU
      tf2::Quaternion q_rotate;
      q_rotate.setRPY(M_PI, 0.0, M_PI / 2);
      // Apply the NED to ENU rotation such that the coordinate frame matches
      tf2_quat = q_rotate * tf2_quat;
      msgOdom.pose.pose.orientation = tf2::toMsg(tf2_quat);
    } else if (user_data->tf_ned_to_enu && !user_data->frame_based_enu) {
      // alternative method for conversion to ENU frame (leads to another result)
      // put into ENU - swap X/Y, invert Z
      msgOdom.pose.pose.orientation.x = q[1];
      msgOdom.pose.pose.orientation.y = q[0];
      msgOdom.pose.pose.orientation.z = -q[2];
      msgOdom.pose.pose.orientation.w = q[3];
    }

    // Read the estimation uncertainty (1 Sigma) from the sensor and write it to the covariance matrix.
    if (cd.hasAttitudeUncertainty()) {
      vec3f orientationStdDev = cd.attitudeUncertainty();
      // convert the standard deviation values from all three axis from degrees to radiant and calculate the variances from these (squared), which are assigned to the covariance matrix.
      if (!user_data->tf_ned_to_enu || user_data->frame_based_enu) {
        // standard assignment of variance values for NED frame and conversion to ENU frame by rotation
        msgOdom.pose.covariance[21] = pow(orientationStdDev[0] * M_PI / 180, 2);  // roll variance
        msgOdom.pose.covariance[28] = pow(orientationStdDev[1] * M_PI / 180, 2);  // pitch variance
        msgOdom.pose.covariance[35] = pow(orientationStdDev[2] * M_PI / 180, 2);  // yaw variance
      } else {
        // variance assignment for conversion by swapping and inverting (not frame_based_enu)

        // TODO not supported yet
      }
    }
  }

  // Add the velocity in the body frame (frame_id) to the message
  if (cd.hasVelocityEstimatedBody()) {
    vec3f vel = cd.velocityEstimatedBody();

    if (!user_data->tf_ned_to_enu || user_data->frame_based_enu) {
      // standard assignment of values for NED frame and conversion to ENU frame by rotation
      msgOdom.twist.twist.linear.x = vel[0];
      msgOdom.twist.twist.linear.y = vel[1];
      msgOdom.twist.twist.linear.z = vel[2];
    } else {
      // value assignment for conversion by swapping and inverting (not frame_based_enu)
      // Flip x and y then invert z
      msgOdom.twist.twist.linear.x = vel[1];
      msgOdom.twist.twist.linear.y = vel[0];
      msgOdom.twist.twist.linear.z = -vel[2];
    }

    // Read the estimation uncertainty (1 Sigma) from the sensor and write it to the covariance matrix.
    if (cd.hasVelocityUncertaintyEstimated()) {
      double velVariance = pow(cd.velocityUncertaintyEstimated(), 2);
      msgOdom.twist.covariance[0] = velVariance;   // x-axis velocity variance
      msgOdom.twist.covariance[7] = velVariance;   // y-axis velocity vaciance
      msgOdom.twist.covariance[14] = velVariance;  // z-axis velocity variance

      // set velocity variances to a high value if no data is available (this is the case at startup during INS is initializing)
      if (
        msgOdom.twist.twist.linear.x == 0 && msgOdom.twist.twist.linear.y == 0 &&
        msgOdom.twist.twist.linear.z == 0 && msgOdom.twist.covariance[0] == 0 &&
        msgOdom.twist.covariance[7] == 0 && msgOdom.twist.covariance[14] == 0) {
        msgOdom.twist.covariance[0] = 200;
        msgOdom.twist.covariance[7] = 200;
        msgOdom.twist.covariance[15] = 200;
      }
    }
  }

  if (cd.hasAngularRate()) {
    vec3f ar = cd.angularRate();

    if (!user_data->tf_ned_to_enu || user_data->frame_based_enu) {
      // standard assignment of values for NED frame and conversion to ENU frame by rotation
      msgOdom.twist.twist.angular.x = ar[0];
      msgOdom.twist.twist.angular.y = ar[1];
      msgOdom.twist.twist.angular.z = ar[2];
    } else {
      // value assignment for conversion by swapping and inverting (not frame_based_enu)
      // Flip x and y then invert z
      msgOdom.twist.twist.angular.x = ar[1];
      msgOdom.twist.twist.angular.y = ar[0];
      msgOdom.twist.twist.angular.z = -ar[2];
    }

    // add covariance matrix of the measured angular rate to odom message.
    // go through matrix rows
    for (int row = 0; row < 3; row++) {
      // go through matrix columns
      for (int col = 0; col < 3; col++) {
        // Target matrix has 6 rows and 6 columns, source matrix has 3 rows and 3 columns. The covariance values are put into the fields (3, 3) to (5, 5) of the destination matrix.
        msgOdom.twist.covariance[(row + 3) * 6 + (col + 3)] =
          user_data->angular_vel_covariance[row * 3 + col];
      }
    }
  }
}

//Helper function to create temperature message
void fill_temp_message(
  sensor_msgs::Temperature & msgTemp, vn::sensors::CompositeData & cd, ros::Time & time,
  UserData * user_data)
{
  msgTemp.header.stamp = time;
  msgTemp.header.frame_id = user_data->frame_id;
  if (cd.hasTemperature()) {
    float temp = cd.temperature();
    msgTemp.temperature = temp;
  }
}

//Helper function to create pressure message
void fill_pres_message(
  sensor_msgs::FluidPressure & msgPres, vn::sensors::CompositeData & cd, ros::Time & time,
  UserData * user_data)
{
  msgPres.header.stamp = time;
  msgPres.header.frame_id = user_data->frame_id;
  if (cd.hasPressure()) {
    float pres = cd.pressure();
    msgPres.fluid_pressure = pres;
  }
}

//Helper function to create ins message
void fill_ins_message(
  vectornav::Ins & msgINS, vn::sensors::CompositeData & cd, ros::Time & time, UserData * user_data)
{
  msgINS.header.stamp = time;
  msgINS.header.frame_id = user_data->frame_id;

  if (cd.hasInsStatus()) {
    InsStatus insStatus = cd.insStatus();
    msgINS.insStatus = static_cast<uint16_t>(insStatus);
  }

  if (cd.hasTow()) {
    msgINS.time = cd.tow();
  }

  if (cd.hasWeek()) {
    msgINS.week = cd.week();
  }

  if (cd.hasTimeUtc()) {
    TimeUtc utcTime = cd.timeUtc();
    char * utcTimeBytes = reinterpret_cast<char *>(&utcTime);
    //msgINS.utcTime bytes are in Little Endian Byte Order
    std::memcpy(&msgINS.utcTime, utcTimeBytes, 8);
  }

  if (cd.hasYawPitchRoll()) {
    vec3f rpy = cd.yawPitchRoll();
    msgINS.yaw = rpy[0];
    msgINS.pitch = rpy[1];
    msgINS.roll = rpy[2];
  }

  if (cd.hasPositionEstimatedLla()) {
    vec3d lla = cd.positionEstimatedLla();
    msgINS.latitude = lla[0];
    msgINS.longitude = lla[1];
    msgINS.altitude = lla[2];
  }

  if (cd.hasVelocityEstimatedNed()) {
    vec3f nedVel = cd.velocityEstimatedNed();
    msgINS.nedVelX = nedVel[0];
    msgINS.nedVelY = nedVel[1];
    msgINS.nedVelZ = nedVel[2];
  }

  if (cd.hasAttitudeUncertainty()) {
    vec3f attUncertainty = cd.attitudeUncertainty();
    msgINS.attUncertainty[0] = attUncertainty[0];
    msgINS.attUncertainty[1] = attUncertainty[1];
    msgINS.attUncertainty[2] = attUncertainty[2];
  }

  if (cd.hasPositionUncertaintyEstimated()) {
    msgINS.posUncertainty = cd.positionUncertaintyEstimated();
  }

  if (cd.hasVelocityUncertaintyEstimated()) {
    msgINS.velUncertainty = cd.velocityUncertaintyEstimated();
  }
}

static ros::Time get_time_stamp(vn::sensors::CompositeData & cd, UserData * user_data, const ros::Time & ros_time)
{
  // Conditions to use TimeSyncIn as timestamp
  if (user_data->timestamp_type == 2 && cd.hasTimeSyncIn())
  {
    return ros::Time(cd.timeSyncIn() * 1e-9);
  }

  // Conditions to use TimeStartup not adjusted to ROS Time as timestamp
  else if (user_data->timestamp_type == 1 && cd.hasTimeStartup() && !user_data->adjust_ros_timestamp)
  {
    return ros::Time(cd.timeStartup() * 1e-9);
  }

  // Conditions to use TimeStartup adjusted to ROS Time as timestamp
  else if (user_data->timestamp_type == 1 && cd.hasTimeStartup() && user_data->adjust_ros_timestamp)
  {
    const double sensor_time = cd.timeStartup() * 1e-9;  // time in seconds
    if (user_data->average_time_difference == 0) {       // first call
      user_data->ros_start_time = ros_time;
      user_data->average_time_difference = static_cast<double>(-sensor_time);
    }
    // difference between node startup and current ROS time
    const double ros_dt = (ros_time - user_data->ros_start_time).toSec();
    // difference between elapsed ROS time and time since sensor startup
    const double dt = ros_dt - sensor_time;
    // compute exponential moving average
    const double alpha = 0.001;  // average over rougly 1000 samples
    user_data->average_time_difference =
      user_data->average_time_difference * (1.0 - alpha) + alpha * dt;

    // adjust sensor time by average difference to ROS time
    const ros::Time adj_time =
      user_data->ros_start_time + ros::Duration(user_data->average_time_difference + sensor_time);
    return (adj_time);
  }

  // If no previous conditions are satisfied, use ROS Time as timestamp
  else
    return (ros_time);
}

//
// Callback function to process data packet from sensor
//
void BinaryAsyncMessageReceived(void * userData, Packet & p, size_t index)
{
  // package counter to calculate strides
  static unsigned long long pkg_count = 0;

  // evaluate time first, to have it as close to the measurement time as possible
  const ros::Time ros_time = ros::Time::now();

  vn::sensors::CompositeData cd = vn::sensors::CompositeData::parse(p);
  UserData * user_data = static_cast<UserData *>(userData);
  ros::Time time = get_time_stamp(cd, user_data, ros_time);

  // IMU
  if ((pkg_count % user_data->imu_stride) == 0) {
    sensor_msgs::Imu msgIMU;
    fill_imu_message(msgIMU, cd, time, user_data);
    if(user_data->use_imu_with_syncincount_msg)
    {
      vectornav::ImuWithCount msgIMUWithCount;
      msgIMUWithCount.imu = msgIMU;

      // Check if IMU msg contains a SyncInCount. If not, set count to NULL
      if(cd.hasSyncInCnt())
      {
        msgIMUWithCount.count = cd.syncInCnt();
        pubIMU.publish(msgIMUWithCount);
      }
      else
      {
        ROS_WARN_STREAM("IMU message does not contain SyncInCount.");
        ROS_WARN_STREAM("The message is: " << msgIMU);
      }
    }
    else
      pubIMU.publish(msgIMU);
  }

  if ((pkg_count % user_data->output_stride) == 0) {
    // Magnetic Field
    if (pubMag.getNumSubscribers() > 0) {
      sensor_msgs::MagneticField msgMag;
      fill_mag_message(msgMag, cd, time, user_data);
      pubMag.publish(msgMag);
    }

    // Temperature
    if (pubTemp.getNumSubscribers() > 0) {
      sensor_msgs::Temperature msgTemp;
      fill_temp_message(msgTemp, cd, time, user_data);
      pubTemp.publish(msgTemp);
    }

    // Barometer
    if (pubPres.getNumSubscribers() > 0) {
      sensor_msgs::FluidPressure msgPres;
      fill_pres_message(msgPres, cd, time, user_data);
      pubPres.publish(msgPres);
    }

    // GPS
    if (
      user_data->device_family != VnSensor::Family::VnSensor_Family_Vn100 &&
      pubGPS.getNumSubscribers() > 0) {
      sensor_msgs::NavSatFix msgGPS;
      fill_gps_message(msgGPS, cd, time, user_data);
      pubGPS.publish(msgGPS);
    }

    // Odometry
    if (
      user_data->device_family != VnSensor::Family::VnSensor_Family_Vn100 &&
      pubOdom.getNumSubscribers() > 0) {
      nav_msgs::Odometry msgOdom;
      fill_odom_message(msgOdom, cd, time, user_data);
      pubOdom.publish(msgOdom);
    }

    // INS
    if (
      user_data->device_family != VnSensor::Family::VnSensor_Family_Vn100 &&
      pubIns.getNumSubscribers() > 0) {
      vectornav::Ins msgINS;
      fill_ins_message(msgINS, cd, time, user_data);
      pubIns.publish(msgINS);
    }
  }
  pkg_count += 1;
}

vn::protocol::uart::CommonGroup getCommonGroupSetUp(ros::NodeHandle pn)
{
  vn::protocol::uart::CommonGroup commonGroupSetUp;
  bool use_group;
  pn.param<bool>("common_group/use_group", use_group, false);
  if(use_group)
  {
    bool get_timestartup, get_timegps, get_timesyncin, get_yawpitchroll, get_quaternion,
      get_angularrate, get_position, get_velocity, get_accel, get_imu, get_magpres,
      get_deltatheta, get_insstatus, get_syncincnt, get_timegpspps;

    // Get activation parameters
    pn.param<bool>("common_group/get_timestartup", get_timestartup, false);
    pn.param<bool>("common_group/get_timegps", get_timegps, false);
    pn.param<bool>("common_group/get_timesyncin", get_timesyncin, false);
    pn.param<bool>("common_group/get_yawpitchroll", get_yawpitchroll, false);
    pn.param<bool>("common_group/get_quaternion", get_quaternion, false);
    pn.param<bool>("common_group/get_angularrate", get_angularrate, false);
    pn.param<bool>("common_group/get_position", get_position, false);
    pn.param<bool>("common_group/get_velocity", get_velocity, false);
    pn.param<bool>("common_group/get_accel", get_accel, false);
    pn.param<bool>("common_group/get_imu", get_imu, false);
    pn.param<bool>("common_group/get_magpres", get_magpres, false);
    pn.param<bool>("common_group/get_deltatheta", get_deltatheta, false);
    pn.param<bool>("common_group/get_insstatus", get_insstatus, false);
    pn.param<bool>("common_group/get_syncincnt", get_syncincnt, false);
    pn.param<bool>("common_group/get_timegpspps", get_timegpspps, false);

    // Set up the Common Group wanted fields depending on parameters
    commonGroupSetUp = (get_timestartup ? COMMONGROUP_TIMESTARTUP : COMMONGROUP_NONE) |
                       (get_timegps ? COMMONGROUP_TIMEGPS : COMMONGROUP_NONE) |
                       (get_timesyncin ? COMMONGROUP_TIMESYNCIN : COMMONGROUP_NONE) |
                       (get_yawpitchroll ? COMMONGROUP_YAWPITCHROLL : COMMONGROUP_NONE) |
                       (get_quaternion ? COMMONGROUP_QUATERNION : COMMONGROUP_NONE) |
                       (get_angularrate ? COMMONGROUP_ANGULARRATE : COMMONGROUP_NONE) |
                       (get_position ? COMMONGROUP_POSITION : COMMONGROUP_NONE) |
                       (get_velocity ? COMMONGROUP_VELOCITY : COMMONGROUP_NONE) |
                       (get_accel ? COMMONGROUP_ACCEL : COMMONGROUP_NONE) |
                       (get_imu ? COMMONGROUP_IMU : COMMONGROUP_NONE) |
                       (get_magpres ? COMMONGROUP_MAGPRES : COMMONGROUP_NONE) |
                       (get_deltatheta ? COMMONGROUP_DELTATHETA : COMMONGROUP_NONE) |
                       (get_insstatus ? COMMONGROUP_INSSTATUS : COMMONGROUP_NONE) |
                       (get_syncincnt ? COMMONGROUP_SYNCINCNT : COMMONGROUP_NONE) |
                       (get_timegpspps ? COMMONGROUP_TIMEGPSPPS : COMMONGROUP_NONE);
  }
  else
  {
    commonGroupSetUp = COMMONGROUP_NONE;
  }
  return commonGroupSetUp;
}


vn::protocol::uart::TimeGroup getTimeGroupSetUp(ros::NodeHandle pn)
{
  vn::protocol::uart::TimeGroup timeGroupSetUp;
  bool use_group;
  pn.param<bool>("time_group/use_group", use_group, false);
  if(use_group)
  {
    bool get_timestartup, get_timegps, get_gpstow, get_gpsweek, get_timesyncin,
      get_timegpspps, get_timeutc, get_syncincnt, get_syncoutcnt, get_timestatus;

    // Get activation parameters
    pn.param<bool>("time_group/get_timestartup", get_timestartup, false);
    pn.param<bool>("time_group/get_timegps", get_timegps, false);
    pn.param<bool>("time_group/get_gpstow", get_gpstow, false);
    pn.param<bool>("time_group/get_gpsweek", get_gpsweek, false);
    pn.param<bool>("time_group/get_timesyncin", get_timesyncin, false);
    pn.param<bool>("time_group/get_timegpspps", get_timegpspps, false);
    pn.param<bool>("time_group/get_timeutc", get_timeutc, false);
    pn.param<bool>("time_group/get_syncincnt", get_syncincnt, false);
    pn.param<bool>("time_group/get_syncoutcnt", get_syncoutcnt, false);
    pn.param<bool>("time_group/get_timestatus", get_timestatus, false);

    // Set up the Common Group wanted fields depending on parameters
    timeGroupSetUp = (get_timestartup ? TIMEGROUP_TIMESTARTUP : TIMEGROUP_NONE) |
                       (get_timegps ? TIMEGROUP_TIMEGPS : TIMEGROUP_NONE) |
                       (get_gpstow ? TIMEGROUP_GPSTOW : TIMEGROUP_NONE) |
                       (get_gpsweek ? TIMEGROUP_GPSWEEK : TIMEGROUP_NONE) |
                       (get_timesyncin ? TIMEGROUP_TIMESYNCIN : TIMEGROUP_NONE) |
                       (get_timegpspps ? TIMEGROUP_TIMEGPSPPS : TIMEGROUP_NONE) |
                       (get_timeutc ? TIMEGROUP_TIMEUTC : TIMEGROUP_NONE) |
                       (get_syncincnt ? TIMEGROUP_SYNCINCNT : TIMEGROUP_NONE) |
                       (get_syncoutcnt ? TIMEGROUP_SYNCOUTCNT : TIMEGROUP_NONE) |
                       (get_timestatus ? TIMEGROUP_TIMESTATUS : TIMEGROUP_NONE);
  }
  else
  {
    timeGroupSetUp = TIMEGROUP_NONE;
  }
  return timeGroupSetUp;
}

vn::protocol::uart::ImuGroup getImuGroupSetUp(ros::NodeHandle pn)
{
  vn::protocol::uart::ImuGroup imuGroupSetUp;
  bool use_group;
  pn.param<bool>("imu_group/use_group", use_group, false);
  if(use_group)
  {
    bool get_imustatus, get_uncompmag, get_uncompaccel, get_uncompgyro, get_temp, get_pres,
      get_deltatheta, get_deltavel, get_mag, get_accel, get_angularrate, get_senssat;

    // Get activation parameters
    pn.param<bool>("imu_group/get_imustatus", get_imustatus, false);
    pn.param<bool>("imu_group/get_uncompmag", get_uncompmag, false);
    pn.param<bool>("imu_group/get_uncompaccel", get_uncompaccel, false);
    pn.param<bool>("imu_group/get_uncompgyro", get_uncompgyro, false);
    pn.param<bool>("imu_group/get_temp", get_temp, false);
    pn.param<bool>("imu_group/get_pres", get_pres, false);
    pn.param<bool>("imu_group/get_deltatheta", get_deltatheta, false);
    pn.param<bool>("imu_group/get_deltavel", get_deltavel, false);
    pn.param<bool>("imu_group/get_mag", get_mag, false);
    pn.param<bool>("imu_group/get_accel", get_accel, false);
    pn.param<bool>("imu_group/get_angularrate", get_angularrate, false);
    pn.param<bool>("imu_group/get_senssat", get_senssat, false);

    // Set up the Common Group wanted fields depending on parameters
    imuGroupSetUp = (get_imustatus ? IMUGROUP_IMUSTATUS : IMUGROUP_NONE) |
                     (get_uncompmag ? IMUGROUP_UNCOMPMAG : IMUGROUP_NONE) |
                     (get_uncompaccel ? IMUGROUP_UNCOMPACCEL : IMUGROUP_NONE) |
                     (get_uncompgyro ? IMUGROUP_UNCOMPGYRO : IMUGROUP_NONE) |
                     (get_temp ? IMUGROUP_TEMP : IMUGROUP_NONE) |
                     (get_pres ? IMUGROUP_PRES : IMUGROUP_NONE) |
                     (get_deltatheta ? IMUGROUP_DELTATHETA : IMUGROUP_NONE) |
                     (get_deltavel ? IMUGROUP_DELTAVEL : IMUGROUP_NONE) |
                     (get_mag ? IMUGROUP_MAG : IMUGROUP_NONE) |
                     (get_accel ? IMUGROUP_ACCEL : IMUGROUP_NONE) |
                     (get_angularrate ? IMUGROUP_ANGULARRATE : IMUGROUP_NONE) |
                     (get_senssat ? IMUGROUP_SENSSAT : IMUGROUP_NONE);
  }
  else
  {
    imuGroupSetUp = IMUGROUP_NONE;
  }
  return imuGroupSetUp;
}
vn::protocol::uart::GpsGroup getGpsGroupSetUp(ros::NodeHandle pn)
{
  vn::protocol::uart::GpsGroup gpsGroupSetUp;
  bool use_group;
  pn.param<bool>("gps_group/use_group", use_group, false);
  if(use_group)
  {
    bool get_utc, get_tow, get_week, get_numsats, get_fix, get_poslla, get_posecef, get_velned,
      get_velecef, get_posu, get_velu, get_timeu, get_timeinfo, get_dop;

    // Get activation parameters
    pn.param<bool>("gps_group/get_utc", get_utc, false);
    pn.param<bool>("gps_group/get_tow", get_tow, false);
    pn.param<bool>("gps_group/get_week", get_week, false);
    pn.param<bool>("gps_group/get_numsats", get_numsats, false);
    pn.param<bool>("gps_group/get_fix", get_fix, false);
    pn.param<bool>("gps_group/get_poslla", get_poslla, false);
    pn.param<bool>("gps_group/get_posecef", get_posecef, false);
    pn.param<bool>("gps_group/get_velned", get_velned, false);
    pn.param<bool>("gps_group/get_velecef", get_velecef, false);
    pn.param<bool>("gps_group/get_posu", get_posu, false);
    pn.param<bool>("gps_group/get_velu", get_velu, false);
    pn.param<bool>("gps_group/get_timeu", get_timeu, false);
    pn.param<bool>("gps_group/get_timeinfo", get_timeinfo, false);
    pn.param<bool>("gps_group/get_dop", get_dop, false);

    // Set up the Common Group wanted fields depending on parameters
    gpsGroupSetUp = (get_utc ? GPSGROUP_UTC : GPSGROUP_NONE) |
                    (get_tow ? GPSGROUP_TOW : GPSGROUP_NONE) |
                    (get_week ? GPSGROUP_WEEK : GPSGROUP_NONE) |
                    (get_numsats ? GPSGROUP_NUMSATS : GPSGROUP_NONE) |
                    (get_fix ? GPSGROUP_FIX : GPSGROUP_NONE) |
                    (get_poslla ? GPSGROUP_POSLLA : GPSGROUP_NONE) |
                    (get_posecef ? GPSGROUP_POSECEF : GPSGROUP_NONE) |
                    (get_velned ? GPSGROUP_VELNED : GPSGROUP_NONE) |
                    (get_velecef ? GPSGROUP_VELECEF : GPSGROUP_NONE) |
                    (get_posu ? GPSGROUP_POSU : GPSGROUP_NONE) |
                    (get_velu ? GPSGROUP_VELU : GPSGROUP_NONE) |
                    (get_timeu ? GPSGROUP_TIMEU : GPSGROUP_NONE) |
                    (get_timeinfo ? GPSGROUP_TIMEINFO : GPSGROUP_NONE) |
                    (get_dop ? GPSGROUP_DOP : GPSGROUP_NONE);
  }
  else
  {
    gpsGroupSetUp = GPSGROUP_NONE;
  }
  return gpsGroupSetUp;
}

vn::protocol::uart::GpsGroup getGps2GroupSetUp(ros::NodeHandle pn)
{
  vn::protocol::uart::GpsGroup gps2GroupSetUp;
  bool use_group;
  pn.param<bool>("gps2_group/use_group", use_group, false);
  if(use_group)
  {
    bool get_utc, get_tow, get_week, get_numsats, get_fix, get_poslla, get_posecef, get_velned,
      get_velecef, get_posu, get_velu, get_timeu, get_timeinfo, get_dop;

    // Get activation parameters
    pn.param<bool>("gps2_group/get_utc", get_utc, false);
    pn.param<bool>("gps2_group/get_tow", get_tow, false);
    pn.param<bool>("gps2_group/get_week", get_week, false);
    pn.param<bool>("gps2_group/get_numsats", get_numsats, false);
    pn.param<bool>("gps2_group/get_fix", get_fix, false);
    pn.param<bool>("gps2_group/get_poslla", get_poslla, false);
    pn.param<bool>("gps2_group/get_posecef", get_posecef, false);
    pn.param<bool>("gps2_group/get_velned", get_velned, false);
    pn.param<bool>("gps2_group/get_velecef", get_velecef, false);
    pn.param<bool>("gps2_group/get_posu", get_posu, false);
    pn.param<bool>("gps2_group/get_velu", get_velu, false);
    pn.param<bool>("gps2_group/get_timeu", get_timeu, false);
    pn.param<bool>("gps2_group/get_timeinfo", get_timeinfo, false);
    pn.param<bool>("gps2_group/get_dop", get_dop, false);

    // Set up the Common Group wanted fields depending on parameters
    gps2GroupSetUp = (get_utc ? GPSGROUP_UTC : GPSGROUP_NONE) |
                    (get_tow ? GPSGROUP_TOW : GPSGROUP_NONE) |
                    (get_week ? GPSGROUP_WEEK : GPSGROUP_NONE) |
                    (get_numsats ? GPSGROUP_NUMSATS : GPSGROUP_NONE) |
                    (get_fix ? GPSGROUP_FIX : GPSGROUP_NONE) |
                    (get_poslla ? GPSGROUP_POSLLA : GPSGROUP_NONE) |
                    (get_posecef ? GPSGROUP_POSECEF : GPSGROUP_NONE) |
                    (get_velned ? GPSGROUP_VELNED : GPSGROUP_NONE) |
                    (get_velecef ? GPSGROUP_VELECEF : GPSGROUP_NONE) |
                    (get_posu ? GPSGROUP_POSU : GPSGROUP_NONE) |
                    (get_velu ? GPSGROUP_VELU : GPSGROUP_NONE) |
                    (get_timeu ? GPSGROUP_TIMEU : GPSGROUP_NONE) |
                    (get_timeinfo ? GPSGROUP_TIMEINFO : GPSGROUP_NONE) |
                    (get_dop ? GPSGROUP_DOP : GPSGROUP_NONE);
  }
  else
  {
    gps2GroupSetUp = GPSGROUP_NONE;
  }
  return gps2GroupSetUp;
}

vn::protocol::uart::AttitudeGroup getAttitudeGroupSetUp(ros::NodeHandle pn)
{
  vn::protocol::uart::AttitudeGroup attitudeGroupSetUp;
  bool use_group;
  pn.param<bool>("attitude_group/use_group", use_group, false);
  if(use_group)
  {
    bool get_vpestatus, get_yawpitchroll, get_quaternion, get_dcm, get_magned, get_accelned,
      get_linearaccelbody, get_linearaccelned, get_ypru, get_heave;

    // Get activation parameters
    pn.param<bool>("attitude_group/get_vpestatus", get_vpestatus, false);
    pn.param<bool>("attitude_group/get_yawpitchroll", get_yawpitchroll, false);
    pn.param<bool>("attitude_group/get_quaternion", get_quaternion, false);
    pn.param<bool>("attitude_group/get_dcm", get_dcm, false);
    pn.param<bool>("attitude_group/get_magned", get_magned, false);
    pn.param<bool>("attitude_group/get_accelned", get_accelned, false);
    pn.param<bool>("attitude_group/get_linearaccelbody", get_linearaccelbody, false);
    pn.param<bool>("attitude_group/get_linearaccelned", get_linearaccelned, false);
    pn.param<bool>("attitude_group/get_ypru", get_ypru, false);
    pn.param<bool>("attitude_group/get_heave", get_heave, false);

    // Set up the Common Group wanted fields depending on parameters
    attitudeGroupSetUp = (get_vpestatus ? ATTITUDEGROUP_VPESTATUS : ATTITUDEGROUP_NONE) |
                     (get_yawpitchroll ? ATTITUDEGROUP_YAWPITCHROLL : ATTITUDEGROUP_NONE) |
                     (get_quaternion ? ATTITUDEGROUP_QUATERNION : ATTITUDEGROUP_NONE) |
                     (get_dcm ? ATTITUDEGROUP_DCM : ATTITUDEGROUP_NONE) |
                     (get_magned ? ATTITUDEGROUP_MAGNED : ATTITUDEGROUP_NONE) |
                     (get_accelned ? ATTITUDEGROUP_ACCELNED : ATTITUDEGROUP_NONE) |
                     (get_linearaccelbody ? ATTITUDEGROUP_LINEARACCELBODY : ATTITUDEGROUP_NONE) |
                     (get_linearaccelned ? ATTITUDEGROUP_LINEARACCELNED : ATTITUDEGROUP_NONE) |
                     (get_ypru ? ATTITUDEGROUP_YPRU : ATTITUDEGROUP_NONE) |
                     (get_heave ? ATTITUDEGROUP_HEAVE : ATTITUDEGROUP_NONE);
  }
  else
  {
    attitudeGroupSetUp = ATTITUDEGROUP_NONE;
  }
  return attitudeGroupSetUp;
}

vn::protocol::uart::InsGroup getInsGroupSetUp(ros::NodeHandle pn)
{
  vn::protocol::uart::InsGroup insGroupSetUp;
  bool use_group;
  pn.param<bool>("ins_group/use_group", use_group, false);
  if(use_group)
  {
    bool get_insstatus, get_poslla, get_posecef, get_velbody, get_velned, get_velecef, get_magecef,
      get_accelecef, get_linearaccelecef, get_posu, get_velu;

    // Get activation parameters
    pn.param<bool>("ins_group/get_insstatus", get_insstatus, false);
    pn.param<bool>("ins_group/get_poslla", get_poslla, false);
    pn.param<bool>("ins_group/get_posecef", get_posecef, false);
    pn.param<bool>("ins_group/get_velbody", get_velbody, false);
    pn.param<bool>("ins_group/get_velned", get_velned, false);
    pn.param<bool>("ins_group/get_velecef", get_velecef, false);
    pn.param<bool>("ins_group/get_magecef", get_magecef, false);
    pn.param<bool>("ins_group/get_accelecef", get_accelecef, false);
    pn.param<bool>("ins_group/get_linearaccelecef", get_linearaccelecef, false);
    pn.param<bool>("ins_group/get_posu", get_posu, false);
    pn.param<bool>("ins_group/get_velu", get_velu, false);

    // Set up the Common Group wanted fields depending on parameters
    insGroupSetUp = (get_insstatus ? INSGROUP_INSSTATUS : INSGROUP_NONE) |
                         (get_poslla ? INSGROUP_POSLLA : INSGROUP_NONE) |
                         (get_posecef ? INSGROUP_POSECEF : INSGROUP_NONE) |
                         (get_velbody ? INSGROUP_VELBODY : INSGROUP_NONE) |
                         (get_velned ? INSGROUP_VELNED : INSGROUP_NONE) |
                         (get_velecef ? INSGROUP_VELECEF : INSGROUP_NONE) |
                         (get_magecef ? INSGROUP_MAGECEF : INSGROUP_NONE) |
                         (get_accelecef ? INSGROUP_ACCELECEF : INSGROUP_NONE) |
                         (get_linearaccelecef ? INSGROUP_LINEARACCELECEF : INSGROUP_NONE) |
                         (get_posu ? INSGROUP_POSU : INSGROUP_NONE) |
                         (get_velu ? INSGROUP_VELU : INSGROUP_NONE);
  }
  else
  {
    insGroupSetUp = INSGROUP_NONE;
  }
  return insGroupSetUp;
}

vn::protocol::uart::SyncInMode syncInModeSetUp(int sync_in_mode)
{
  vn::protocol::uart::SyncInMode uart_sync_in_mode;
  if(sync_in_mode == 3)
    uart_sync_in_mode = SYNCINMODE_COUNT;
  else if(sync_in_mode == 4)
    uart_sync_in_mode = SYNCINMODE_IMU;
  else if(sync_in_mode == 5)
    uart_sync_in_mode = SYNCINMODE_ASYNC;
  else if(sync_in_mode == 6)
    uart_sync_in_mode = SYNCINMODE_ASYNC3;
  else
  {
    uart_sync_in_mode = SYNCINMODE_COUNT;
    ROS_WARN("The SyncInMode value you positioned does not exist. SyncInMode positioned to the default value.");
  }
  ROS_INFO("SyncInMode = %d", uart_sync_in_mode);
  return uart_sync_in_mode;
}

vn::protocol::uart::SyncInEdge syncInEdgeSetUp(int sync_in_edge)
{
  vn::protocol::uart::SyncInEdge uart_sync_in_edge;
  if(sync_in_edge == 0)
    uart_sync_in_edge = SYNCINEDGE_RISING;
  else if(sync_in_edge == 1)
    uart_sync_in_edge = SYNCINEDGE_FALLING;
  else
  {
    uart_sync_in_edge = SYNCINEDGE_RISING;
    ROS_WARN("The SyncInEdge value you positioned does not exist. SyncInEdge positioned to the default value.");
  }
  ROS_INFO("SyncInEdge = %d", uart_sync_in_edge);
  return uart_sync_in_edge;
}

uint16_t syncInSkipFactor(int sync_in_skip_factor)
{
  uint16_t uart_sync_in_skip_factor;
  if(sync_in_skip_factor < 0 || sync_in_skip_factor > UINT16_MAX)
  {
    uart_sync_in_skip_factor = 0;
    ROS_WARN("The SyncInSkipFactor value you positioned does not exist. SyncInFactor positioned to the default value.");
  }
  else
    uart_sync_in_skip_factor = sync_in_skip_factor;
  ROS_INFO("SyncInSkipFactor = %d", uart_sync_in_skip_factor);

  return uart_sync_in_skip_factor;
}
