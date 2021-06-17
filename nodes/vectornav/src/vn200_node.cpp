/* * MIT License (MIT)
 *
 * Copyright (c) 2013 Dereck Wonnacott <dereck@gmail.com>
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


#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <stdlib.h>     /* exit, EXIT_FAILURE */
#include <cmath>
#include <inttypes.h>
#include "vn/sensors.h"
#include "vn/compositedata.h"
#include "vn/util.h"

#include <ros/ros.h>
#include <tf/tf.h>
#include <signal.h>
#include <time.h>
// Message Types
#include <vectornav/utc_time.h>
#include <vectornav/imugps.h>

#include <ros/xmlrpc_manager.h>

#include <iostream>
#include <fstream>

using namespace vn::protocol::uart;
using namespace vn::sensors;
using namespace vn::math;

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

const char ledpipe[] = "/tmp/ledpipe0";
const char insrecording[] = "/tmp/insrecording";
const char on[] = "ON\n";
bool fix = false;
unsigned int ledcount = 0;

// Params
std::string imu_frame_id, gps_frame_id;

// Publishers
ros::Publisher pub_ins;

// Device
vn::sensors::VnSensor vn200;

int ins_seq           = 0;
int msg_cnt           = 0;
int last_group_number = 0;


std::string port;
char vn_error_msg[100];

struct utc_time_struct
{
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t millisecond;
};

struct ins_binary_data_struct
{
    uint64_t gps_time;
    vn::math::vec4f orientation;
    vn::math::vec3f angular_rate;
    vn::math::vec3d lla;
    vn::math::vec3f vel_ned;
    vn::math::vec3f accel;
    vn::math::vec4f dtheta;
    vn::math::vec3f dvel;
    uint8_t         fix;
    vn::math::vec3d gpslla;
    vn::math::vec3f gpsvel_ned;
};

ins_binary_data_struct ins_binary_data;

int ins_msg_count = 0;

const unsigned raw_imu_max_rate = 800;

const unsigned ins_group_signature = BINARYGROUP_COMMON | BINARYGROUP_GPS;


void publish_ins_data()
{
    ins_seq++;
    ros::Time timestamp =  ros::Time::now();

    if (pub_ins.getNumSubscribers() > 0)
    {
        vectornav::imugps msg_ins;
        msg_ins.header.stamp    = timestamp;
        msg_ins.header.frame_id = "ins";

        msg_ins.time    = ins_binary_data.gps_time;

        msg_ins.orientation.x = ins_binary_data.orientation[0];
        msg_ins.orientation.y = ins_binary_data.orientation[1];
        msg_ins.orientation.z = ins_binary_data.orientation[2];
        msg_ins.orientation.w = ins_binary_data.orientation[3];

        msg_ins.angular_velocity.x = ins_binary_data.angular_rate[0];
        msg_ins.angular_velocity.y = ins_binary_data.angular_rate[1];
        msg_ins.angular_velocity.z = ins_binary_data.angular_rate[2];

        msg_ins.LLA.x = ins_binary_data.lla[0];
        msg_ins.LLA.y = ins_binary_data.lla[1];
        msg_ins.LLA.z = ins_binary_data.lla[2];

        msg_ins.nedvel.x = ins_binary_data.vel_ned[0];
        msg_ins.nedvel.y = ins_binary_data.vel_ned[1];
        msg_ins.nedvel.z = ins_binary_data.vel_ned[2];

        msg_ins.linear_acceleration.x = ins_binary_data.accel[0];
        msg_ins.linear_acceleration.y = ins_binary_data.accel[1];
        msg_ins.linear_acceleration.z = ins_binary_data.accel[2];

        msg_ins.dtime = ins_binary_data.dtheta[0];

        msg_ins.dtheta[0] = ins_binary_data.dtheta[1];
        msg_ins.dtheta[1] = ins_binary_data.dtheta[2];
        msg_ins.dtheta[2] = ins_binary_data.dtheta[3];

        msg_ins.dvel[0] = ins_binary_data.dvel[0];
        msg_ins.dvel[1] = ins_binary_data.dvel[1];
        msg_ins.dvel[2] = ins_binary_data.dvel[2];

        msg_ins.fix = ins_binary_data.fix;
	if (ins_binary_data.fix > 2) {
            std::fstream fs;
            ledcount++;
            std::ifstream rec;
            try {
                rec.open(insrecording, std::ios::in);
                std::string line;
                getline(rec, line);
                if (line == "ON" && (!fix || ledcount > 3000)) {
                    ledcount = 0;
                    try {
                        fs.open(ledpipe, std::fstream::out|std::fstream::app);
                        fs << on;
                        fix = true;
                    } catch (...) {
                        //
                    }
                }
            } catch (...) {}
        }

        msg_ins.gpsLLA.x = ins_binary_data.gpslla[0];
        msg_ins.gpsLLA.y = ins_binary_data.gpslla[1];
        msg_ins.gpsLLA.z = ins_binary_data.gpslla[2];

        msg_ins.gpsnedvel.x = ins_binary_data.gpsvel_ned[0];
        msg_ins.gpsnedvel.y = ins_binary_data.gpsvel_ned[1];
        msg_ins.gpsnedvel.z = ins_binary_data.gpsvel_ned[2];

        pub_ins.publish(msg_ins);
    }
}

/*
void binaryMessageReceived(void * user_data, Packet & p, size_t index)
{
    std::string raw_data;
    if (p.type() == Packet::TYPE_HEXADECIMAL && p.isValid()) {
        switch (p.groups()) {
        case ins_group_signature:
            ++ins_msg_count;
            ins_binary_data.gps_time =      p.extractUint64();
            ins_binary_data.orientation =   p.extractVec4f();
            ins_binary_data.angular_rate =  p.extractVec3f();
            ins_binary_data.lla =           p.extractVec3d();
            ins_binary_data.vel_ned =       p.extractVec3f();
            ins_binary_data.accel =         p.extractVec3f();
            ins_binary_data.dtheta =        p.extractVec4f();
            ins_binary_data.dvel =          p.extractVec3f();
            ins_binary_data.fix =           p.extractUint8();
            ins_binary_data.gpslla =        p.extractVec3d();
            ins_binary_data.gpsvel_ned =    p.extractVec3f();

            publish_ins_data();

            /*if (remainder(ins_msg_count, 100) == 0) {
                ins_msg_count = 0;
                ROS_INFO_STREAM("INS_Time: " << ins_binary_data.gps_time*1E-9 << " Yaw: " <<
                        ins_binary_data.ypr[0] << " Pitch: " << ins_binary_data.ypr[1] <<
                        " Roll: " << ins_binary_data.ypr[2] << " INS_Status: " <<
                        ins_binary_data.ins_status << " latitude: " << ins_binary_data.lla[0] <<
                        " longitude: " << ins_binary_data.lla[1] << " altitude: " << ins_binary_data.lla[2]);
            }
            
            break;
        default:
            ROS_WARN("Received unknown group signature from vectornav");
        }
    } else {
        ROS_WARN("Received invalid packet from vectornav.");
	ROS_INFO("%s", p.datastr().c_str());
        // Ignore non-binary packets for now.
    }
}*/

void binaryMessageReceived(void * user_data, Packet & p, size_t index)
{
  ROS_INFO("=================Measure===============");
    std::string raw_data;
    /*
    //[Draft]
    //ROS_INFO("packet.type() == ",p.type());
    //ROS_INFO("packet.isValid() == ",p.isValid());
    if (p.isValid()){
      //if (p.type() == Packet::TYPE_BINARY && p.isValid())
        switch (p.groups()) {
        case ins_group_signature:
            ++ins_msg_count;
            ins_binary_data.gps_time =      p.extractUint64();
            ins_binary_data.orientation =   p.extractVec4f();
            ins_binary_data.angular_rate =  p.extractVec3f();
            ins_binary_data.lla =           p.extractVec3d();
            ins_binary_data.vel_ned =       p.extractVec3f();
            ins_binary_data.accel =         p.extractVec3f();
            ins_binary_data.dtheta =        p.extractVec4f();
            ins_binary_data.dvel =          p.extractVec3f();
            ins_binary_data.fix =           p.extractUint8();
            ins_binary_data.gpslla =        p.extractVec3d();
            ins_binary_data.gpsvel_ned =    p.extractVec3f();

            publish_ins_data();

            //*-----------if (remainder(ins_msg_count, 100) == 0) {
                ins_msg_count = 0;
                ROS_INFO_STREAM("INS_Time: " << ins_binary_data.gps_time*1E-9 << " Yaw: " <<
                        ins_binary_data.ypr[0] << " Pitch: " << ins_binary_data.ypr[1] <<
                        " Roll: " << ins_binary_data.ypr[2] << " INS_Status: " <<
                        ins_binary_data.ins_status << " latitude: " << ins_binary_data.lla[0] <<
                        " longitude: " << ins_binary_data.lla[1] << " altitude: " << ins_binary_data.lla[2]);
            }
	//----------------------------------NOT THIS-------------------------------
            break;
        default:
            ROS_WARN("Received unknown group signature from vectornav");
        }
    } else {
        ROS_WARN("Received invalid packet from vectornav.");
	ROS_INFO("%s", p.datastr().c_str());
        // Ignore non-binary packets for now.
	}
    
    */

    /*-------------------------2nd implementation--------------
      //Not print finish extracting packet, ins count increase
    if(p.isValid()){
      ROS_INFO("Start extracting packet");
      ++ins_msg_count;
      ROS_INFO("ins msg count: %d",ins_msg_count);
      ins_binary_data.gps_time =      p.extractUint64();
      ins_binary_data.orientation =   p.extractVec4f();
      ins_binary_data.angular_rate =  p.extractVec3f();
      ins_binary_data.lla =           p.extractVec3d();
      ins_binary_data.vel_ned =       p.extractVec3f();
      ins_binary_data.accel =         p.extractVec3f();
      ins_binary_data.dtheta =        p.extractVec4f();
      ins_binary_data.dvel =          p.extractVec3f();
      ins_binary_data.fix =           p.extractUint8();
      ins_binary_data.gpslla =        p.extractVec3d();
      ins_binary_data.gpsvel_ned =    p.extractVec3f();
      ROS_INFO("Finish extracting packet");
      publish_ins_data();
      }*/
	    
    if(p.isValid()){
    vn::sensors::CompositeData cd = vn::sensors::CompositeData::parse(p);
    vec4f q = cd.quaternion();
    vec3f ar = cd.angularRate();
    vec3f al = cd.acceleration();
    //ROS_INFO("q=%0.2f %f %f %f", q[0],q[1],q[2],q[3]);
    //ROS_INFO("q: %f %f %f %f",q.getX(),q.getY(),q.getZ(),q.getW());
    //ROS_INFO(q.x);
    //ROS_INFO(q[0]);
    vectornav::imugps msg_ins;
    msg_ins.header.stamp = ros::Time::now();
    msg_ins.header.frame_id = "ins";

    //timeGps: uint64_t
    msg_ins.time = cd.timeGps();
    ROS_INFO("msg_ins time %" PRIu64,msg_ins.time);
    
    msg_ins.orientation.x = q[0];
    msg_ins.orientation.y = q[1];
    msg_ins.orientation.z = q[2];
    msg_ins.orientation.w = q[3];
    ROS_INFO("Orientation=(%0.2f,%0.2f,%0.2f,%0.2f))",msg_ins.orientation.x,msg_ins.orientation.y,msg_ins.orientation.z,msg_ins.orientation.w);
    
    msg_ins.angular_velocity.x = ar[0];
    msg_ins.angular_velocity.y = ar[1];
    msg_ins.angular_velocity.z = ar[2];
    ROS_INFO("Angular velocity=(%0.2f,%0.2f,%0.2f)",msg_ins.angular_velocity.x,msg_ins.angular_velocity.y,msg_ins.angular_velocity.z);
      
    msg_ins.linear_acceleration.x = al[0];
    msg_ins.linear_acceleration.y = al[1];
    msg_ins.linear_acceleration.z = al[2];
    ROS_INFO("Linear acceleration=(%0.2f,%0.2f,%0.2f)",msg_ins.linear_acceleration.x,msg_ins.linear_acceleration.y,msg_ins.linear_acceleration.z);

    msg_ins.fix = cd.fix();
    ROS_INFO("Fix: %" PRIu8,msg_ins.fix);
    if (cd.fix() > 2) {
            std::fstream fs;
            ledcount++;
            std::ifstream rec;
            try {
                rec.open(insrecording, std::ios::in);
                std::string line;
                getline(rec, line);
                if (line == "ON" && (!fix || ledcount > 3000)) {
                    ledcount = 0;
                    try {
                        fs.open(ledpipe, std::fstream::out|std::fstream::app);
                        fs << on;
                        fix = true;
                    } catch (...) {
                        //
                    }
                }
            } catch (...) {}
    }
    
    //positionEstimatedLla : math:vec3d
    vec3d lla = cd.positionEstimatedLla();
    msg_ins.LLA.x = lla[0];
    msg_ins.LLA.y = lla[1];
    msg_ins.LLA.z = lla[2];
    ROS_INFO("LLA=(%lf,%lf,%lf)",msg_ins.LLA.x,msg_ins.LLA.y,msg_ins.LLA.z);
    
    //positionGpsLla : math:vec3d
    vec3d gpslla = cd.positionGpsLla();
    msg_ins.gpsLLA.x = gpslla[0];
    msg_ins.gpsLLA.y = gpslla[1];
    msg_ins.gpsLLA.z = gpslla[2];
    ROS_INFO("gps_LLA=(%lf,%lf,%lf)",msg_ins.gpsLLA.x,msg_ins.gpsLLA.y,msg_ins.gpsLLA.z);
      
    vec3f nedVel = cd.velocityEstimatedNed();
    msg_ins.nedvel.x = nedVel[0];
    msg_ins.nedvel.y = nedVel[1];
    msg_ins.nedvel.z = nedVel[2];
    ROS_INFO("velocity_ned=(%0.2f,%0.2f,%0.2f)",msg_ins.nedvel.x,msg_ins.nedvel.y,msg_ins.nedvel.z);
    
    float dtime = cd.deltaTime();
    vec3f dtheta = cd.deltaTheta();
    msg_ins.dtime = dtime;
    ROS_INFO("deltaTime=(%0.2f)",msg_ins.dtime);
    msg_ins.dtheta[0] = dtheta[0];
    msg_ins.dtheta[1] = dtheta[1];
    msg_ins.dtheta[2] = dtheta[2];
    ROS_INFO("deltaTheta=(%0.2f,%0.2f,%0.2f)",msg_ins.dtheta[0],msg_ins.dtheta[1],msg_ins.dtheta[2]);
      
    vec3f dvel = cd.deltaVelocity();
    msg_ins.dvel[0] = dvel[0];
    msg_ins.dvel[1] = dvel[1];
    msg_ins.dvel[2] = dvel[2];
    ROS_INFO("deltaVelocity=(%0.2f,%0.2f,%0.2f)",msg_ins.dvel[0],msg_ins.dvel[1],msg_ins.dvel[2]);
    
    vec3f gpsvel_ned = cd.velocityGpsNed();
    msg_ins.gpsnedvel.x = gpsvel_ned[0];
    msg_ins.gpsnedvel.y = gpsvel_ned[1];
    msg_ins.gpsnedvel.z = gpsvel_ned[2];
    ROS_INFO("gpsvel_ned=(%0.2f,%0.2f,%0.2f)",msg_ins.gpsnedvel.x,msg_ins.gpsnedvel.y,msg_ins.gpsnedvel.z);
    //ROS_INFO("(dvel-%0.2f,%0.2f,%0.2f\n"),msg_ins.dvel, msg_ins.dvel[1],msg_ins.dvel[2]);
    pub_ins.publish(msg_ins);
    }
}

bool dividesEvenly(unsigned numerator, unsigned denominator)
{
    unsigned dividend = numerator / denominator;
    return numerator == denominator * dividend;
}

void mySigintHandler(int sig)
{
    g_request_shutdown = 1;
}

// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
    int num_params = 0;

    if (params.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        num_params = params.size();
    }
    if (num_params > 1) {
        std::string reason = params[1];
        ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
        g_request_shutdown = 1; // Set flag
    }

    result = ros::xmlrpc::responseInt(1, "", 0);
}

/////////////////////////////////////////////////
int main(int argc, char* argv[])
{
    // Initialize ROS;
    ros::init(argc, argv, "vectornav", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    ros::NodeHandle n_("~");

    // Read Parameters
    int baud, poll_rate_ins, poll_rate_gps, poll_rate_imu, async_output_type, async_output_rate,
        binary_data_output_port, binary_gps_data_rate, binary_ins_data_rate,
        binary_imu_data_rate, target_baud;

    int retry_cnt = 0;

    // Override XMLRPC shutdown
    ros::XMLRPCManager::instance()->unbind("shutdown");
    ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);
    signal(SIGINT, mySigintHandler);

    n_.param<std::string>("serial_port" , port     , "/dev/ttyAMA0");
    n_.param<int>(        "serial_baud" , baud     , 115200);
    n_.param<int>(        "poll_rate_gps"   , poll_rate_gps, 5);
    n_.param<int>(        "poll_rate_ins"   , poll_rate_ins, 20);
    n_.param<int>(        "poll_rate_imu"   , poll_rate_imu, 100);

    n_.param<std::string>("imu/frame_id", imu_frame_id, "LLA");
    n_.param<std::string>("gps/frame_id", gps_frame_id, "LLA");

    // Type: 0 None, 19 IMU, 20 GPS, 22 INS
    n_.param<int>(        "async_output_type"  , async_output_type, 0);
    n_.param<int>(        "async_output_rate"  , async_output_rate, 50);

    n_.param<int>(        "binary_data_output_port"  , binary_data_output_port, 1);
    n_.param<int>(        "binary_gps_data_output_rate"  , binary_gps_data_rate, 4);
    n_.param<int>(        "binary_ins_data_output_rate"  , binary_ins_data_rate, 20);
    n_.param<int>(        "binary_imu_data_output_rate"  , binary_imu_data_rate, 100);
    n_.param<int>(        "target_baud"  , target_baud, 921600);

    // Validate the rate inputs.
    if (binary_gps_data_rate < 1 || binary_gps_data_rate > raw_imu_max_rate) {
        ROS_FATAL_STREAM("binary_gps_data_output_rate of "
          << binary_gps_data_rate << " is out of range (1 - "
          << raw_imu_max_rate << ")");
        exit(EXIT_FAILURE);
    } else if (!dividesEvenly(raw_imu_max_rate, binary_gps_data_rate)) {
        ROS_FATAL_STREAM("binary_gps_data_rate does not evenly divide "
          << raw_imu_max_rate);
        exit(EXIT_FAILURE);
    }

    if (binary_ins_data_rate < 1 || binary_ins_data_rate > raw_imu_max_rate) {
        ROS_FATAL_STREAM("binary_ins_data_rate of "
          << binary_ins_data_rate << " is out of range (1 - "
          << raw_imu_max_rate << ")");
        exit(EXIT_FAILURE);
    } else if (!dividesEvenly(raw_imu_max_rate, binary_ins_data_rate)) {
        ROS_FATAL_STREAM("binary_ins_data_rate does not evenly divide "
          << raw_imu_max_rate);
        exit(EXIT_FAILURE);
    }

    if (binary_imu_data_rate < 1 || binary_imu_data_rate > raw_imu_max_rate) {
        ROS_FATAL_STREAM("binary_imu_data_rate of "
          << binary_imu_data_rate << " is out of range (1 - "
          << raw_imu_max_rate << ")");
        exit(EXIT_FAILURE);
    } else if (!dividesEvenly(raw_imu_max_rate, binary_imu_data_rate)) {
        ROS_FATAL_STREAM("binary_imu_data_rate does not evenly divide "
          << raw_imu_max_rate);
        exit(EXIT_FAILURE);
    }

    AsyncMode binary_data_output_mode =
      static_cast<AsyncMode>(binary_data_output_port);

    // Initialize Publishers
    //pub_ins     = n_.advertise<vectornav::ins>    ("ins", 1000);
    //pub_gps     = n_.advertise<vectornav::gps>    ("gps", 1000);
    //pub_sensors = n_.advertise<vectornav::sensors>("imu", 1000);
    pub_ins       = n_.advertise<vectornav::imugps> ("imugps", 1000);

    // Initialize VectorNav
    //VN_ERROR_CODE vn_retval;
    ROS_INFO("Initializing vn200. Port:%s Baud:%d\n", port.c_str(), baud);

    std::vector<uint32_t> bauds = vn200.supportedBaudrates();
    bool connected = false;
    uint32_t connbaud = 0;

    for (std::vector<uint32_t>::reverse_iterator it = bauds.rbegin(); it != bauds.rend(); ++it) {
        try {
            ROS_INFO("Trying baud %d\n", *it);
            vn200.connect(port, *it);
        } catch (...) {
            continue;
        }
        if (vn200.isConnected()) {
            ROS_INFO("Connected?: %d", *it);
            //if (*it == target_baud) {
            //    connected = true;
            //    ROS_INFO("Target baud reached");
            //    break;
            //}
            try {
                vn200.writeSerialBaudRate(target_baud, binary_data_output_port, true);
                ROS_INFO("Connecting with target baud %d\n", target_baud);
                vn200.disconnect();
                vn200.connect(port, target_baud);
            } catch (...) {
                continue;
            }
            connbaud = *it;
            if (vn200.isConnected()) {
                ROS_INFO("Should be connected now");
                connected = true;
            }
            break;
        }
    }

    /*bool connected = false;

    try {
        vn200.connect(port, baud);
        connected = true;
    } catch (...) {
        connected = false;
    }

    if (baud != target_baud && connected) {
        ROS_INFO("Changing baud to %d", target_baud);
        try {
            vn200.writeSerialBaudRate(target_baud, binary_data_output_port, true);
            vn200.disconnect();
        } catch (...) {
            ROS_WARN("Couldn't change baud");
            throw;
        }
	//ros::shutdown();
	try {
            vn200.connect(port, target_baud);
            connected = true;
        } catch (...) {
            connected = false;
        }
    }*/

    if (!connected) {
        ROS_FATAL("Could not conenct to vn200 on port:%s @ Baud:%d;"
                "Did you add your user to the 'dialout' group in /etc/group?",
                port.c_str(),
                baud);
        exit(EXIT_FAILURE);
    }

    //vn200.writeAsyncDataOutputFrequency(0);
    vn200.writeAsyncDataOutputFrequency(async_output_rate);
    
    CommonGroup ins_common_group = COMMONGROUP_TIMEGPS | COMMONGROUP_QUATERNION
        | COMMONGROUP_ANGULARRATE | COMMONGROUP_POSITION
        | COMMONGROUP_VELOCITY | COMMONGROUP_ACCEL | COMMONGROUP_DELTATHETA;

    GpsGroup ins_gps_group = GPSGROUP_FIX | GPSGROUP_POSLLA | GPSGROUP_VELNED;

    BinaryOutputRegister ins_log_reg(
        binary_data_output_mode,
        raw_imu_max_rate / binary_ins_data_rate,
        ins_common_group,
        TIMEGROUP_NONE,
        IMUGROUP_NONE,
        ins_gps_group,
        ATTITUDEGROUP_NONE,
        INSGROUP_NONE);

    ROS_INFO("Writing binary output 1");
    vn200.writeBinaryOutput1(ins_log_reg);
    ROS_INFO("Written binary output 1");
    //vn200.writeBinaryOutput2(ins_log_reg);
    //vn200.writeBinaryOutput3(imu_log_reg);

//    ROS_INFO("About to set SynchronizationControl");

/*    vn::sensors::SynchronizationControlRegister sync_control(
        SYNCINMODE_COUNT,
        SYNCINEDGE_RISING,
        0, // sync in skip factor
        SYNCOUTMODE_GPSPPS,
        SYNCOUTPOLARITY_NEGATIVE,
        0, // sync out skip factor
        1000000); // a millisecond should be fine.

    vn200.writeSynchronizationControl(sync_control);
*/
    vn::math::vec3f position;
    // for original
    position[0] = -0.08;
    position[1] = 0.0;
    position[2] = -0.05;

    /* for jyu
    position[0] = -0.004;
    position[1] = 0.0;
    position[2] = 0.008;
*/
    vn200.writeGpsAntennaOffset(position);

    // Lower magnetometer velocity threshold for walking speeds
    InsAdvancedConfigurationRegister inssettings(
      true, ///< The useMag field.
      true, ///< The usePres field.
      true, ///< The posAtt field.
      true, ///< The velAtt field.
      true, ///< The velBias field.
      FOAMINIT_FOAMINITHEADINGPITCHROLLCOVARIANCE, ///< The useFoam field.
      0, ///< The gpsCovType field.
      5, ///< The velCount field.
      1.0, ///< The velInit field.
      1000.0, ///< The moveOrigin field.
      30.0, ///< The gpsTimeout field.
      1000.0, ///< The deltaLimitPos field.
      100.0, ///< The deltaLimitVel field.
      1.0, ///< The minPosUncertainty field.
      1.0 ///< The minVelUncertainty field.
    );
    vn200.writeInsAdvancedConfiguration(inssettings, false);

    ROS_INFO("Registering handler");
    vn200.registerAsyncPacketReceivedHandler(NULL, binaryMessageReceived);
    ROS_INFO("Registered");

    ros::Rate r(100);
    while (!g_request_shutdown) {
        ros::spinOnce();
        //usleep(500);
        r.sleep();
    }

    vn200.unregisterAsyncPacketReceivedHandler();
    ros::shutdown();
}
