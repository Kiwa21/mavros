/**
 * @brief iLink
 * @file iLink.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */
/*
 * Copyright 2014,2016 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <ros/ros.h>
#include <ros/console.h>

#include <mavros/utils.h>
#include <mavros/mavlink_diag.h>
#include <mavconn/interface.h>

using namespace mavros;
using namespace mavconn;

ros::Publisher mavlink_from_mavros_pub,mavlink_from_master_pub,mavlink_from_slave_pub;
ros::Subscriber mavlink_sub;
MAVConnInterface::Ptr master_link;
MAVConnInterface::Ptr slave_link;
MAVConnInterface::Ptr mavros_link;

ros::Timer mavros_hb_timeout_timer;
ros::Timer master_hb_timeout_timer;
ros::Timer slave_hb_timeout_timer; 


bool master_connected = false;




void mavlink_from_mavros_pub_cb(const mavlink::mavlink_message_t *mmsg, const mavconn::Framing framing)
{
	auto rmsg = boost::make_shared<mavros_msgs::Mavlink>();

	rmsg->header.stamp = ros::Time::now();
	mavros_msgs::mavlink::convert(*mmsg, *rmsg, mavros::utils::enum_value(framing));
	mavlink_from_mavros_pub.publish(rmsg);

    if(mmsg->msgid == 0){
    //mavros_link_diag.set_connection_status(true);
    mavros_hb_timeout_timer.stop();
    mavros_hb_timeout_timer.start();
    }

    if (master_link){
        master_link->send_message_ignore_drop(mmsg);
        //ROS_INFO("Msg sent to Master Link");
    }
    if (slave_link){
        if(!master_connected || mmsg->msgid == 0){
            //Passtrough HEARTBEAT only if master is connected
            slave_link->send_message_ignore_drop(mmsg);
            //ROS_INFO("Msg sent to Slave Link");
            }
    }
}
void mavlink_from_master_pub_cb(const mavlink::mavlink_message_t *mmsg, const mavconn::Framing framing)
{
	auto rmsg = boost::make_shared<mavros_msgs::Mavlink>();

	rmsg->header.stamp = ros::Time::now();
	mavros_msgs::mavlink::convert(*mmsg, *rmsg, mavros::utils::enum_value(framing));
	mavlink_from_master_pub.publish(rmsg);

    mavros_link->send_message_ignore_drop(mmsg);
	
    if(mmsg->msgid == 0){
        //master_link_diag.set_connection_status(true);
        master_connected = true;
        master_hb_timeout_timer.stop();
        master_hb_timeout_timer.start();
    }
}

void mavlink_from_slave_pub_cb(const mavlink::mavlink_message_t *mmsg, const mavconn::Framing framing)
{
	auto rmsg = boost::make_shared<mavros_msgs::Mavlink>();

	rmsg->header.stamp = ros::Time::now();
	mavros_msgs::mavlink::convert(*mmsg, *rmsg, mavros::utils::enum_value(framing));
	mavlink_from_slave_pub.publish(rmsg);

    if(!master_connected){
            ROS_INFO("Master Not Connected : slave sending");
            mavros_link->send_message_ignore_drop(mmsg);
        }
    
    if(mmsg->msgid == 0){
        //slave_link_diag.set_connection_status(true);
        slave_hb_timeout_timer.stop();
        slave_hb_timeout_timer.start();
    }
}

void timeout_hb_mavros_cb(const ros::TimerEvent &event)
{
    //mavros_link_diag.set_connection_status(false);
    ROS_WARN("MAVROS LINK HB TIMEOUT");
}
void timeout_hb_master_cb(const ros::TimerEvent &event)
{
    //master_link_diag.set_connection_status(false);
    master_connected = false;
    ROS_WARN("MASTER LINK HB TIMEOUT");
}
void timeout_hb_slave_cb(const ros::TimerEvent &event)
{
    //slave_link_diag.set_connection_status(false);
    ROS_WARN("SLAVE LINK HB TIMEOUT");
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "iLink");
	ros::NodeHandle priv_nh("~");
	ros::NodeHandle mavlink_nh("mavlink");
    diagnostic_updater::Updater updater_mavros, updater_master, updater_slave;

    mavros::MavlinkDiag mavros_link_diag("iLink_mavros");
    mavros::MavlinkDiag master_link_diag("iLink_master");
    mavros::MavlinkDiag slave_link_diag("iLink_slave");

    mavros_hb_timeout_timer = priv_nh.createTimer(ros::Duration(10),
				timeout_hb_mavros_cb, true);
    master_hb_timeout_timer = priv_nh.createTimer(ros::Duration(10),
                timeout_hb_master_cb, true);
    slave_hb_timeout_timer = priv_nh.createTimer(ros::Duration(10),
                timeout_hb_slave_cb, true); 

    std::string mavros_url,master_url,slave_url;
    priv_nh.param<std::string>("mavros_url", mavros_url, "udp://@");
	priv_nh.param<std::string>("master_url", master_url, "udp://@");
    priv_nh.param<std::string>("slave_url", slave_url, "serial:///dev/ttyACM0");
    bool master_link_error = false;
    bool slave_link_error = false;

    
/* //SYSID and COMPID not needed, we don't change the values inside the msg, just passthrough
    int system_id, component_id;
	int tgt_system_id, tgt_component_id;

    priv_nh.param("system_id", system_id, 1);
	priv_nh.param<int>("component_id", component_id, mavconn::MAV_COMP_ID_UDP_BRIDGE);
	priv_nh.param("target_system_id", tgt_system_id, 1);
	priv_nh.param("target_component_id", tgt_component_id, 1);
*/
	try {
		mavros_link = MAVConnInterface::open_url(mavros_url); //, tgt_system_id, tgt_component_id);
        // may be overridden by URL
		//tgt_system_id = mavros_link->get_system_id();
		//tgt_component_id = mavros_link->get_component_id();

		mavros_link_diag.set_mavconn(mavros_link);
        updater_mavros.setHardwareID(mavros_url);
        updater_mavros.add(mavros_link_diag);
		mavros_link_diag.set_connection_status(true);
	}
	catch (mavconn::DeviceError &ex) {
		ROS_FATAL("iLink MAVROS LINK : %s", ex.what());
        ros::shutdown();
		return 1;
	}
    
    try {
        if (master_url != "") {
        master_link = MAVConnInterface::open_url(master_url); //,system_id, component_id);
        // may be overridden by URL
        //system_id = master_link->get_system_id();
		//component_id = master_link->get_component_id();

		master_link_diag.set_mavconn(master_link);
        updater_master.setHardwareID(master_url);
        updater_master.add(master_link_diag);
        master_link_diag.set_connection_status(true);
        }
	}
	catch (mavconn::DeviceError &ex) {
		ROS_FATAL("iLink MASTER LINK: %s", ex.what());
		master_link_error = true;
	}

    try {
        if (slave_url != "") {
        slave_link = MAVConnInterface::open_url(slave_url); //,system_id, component_id);
        // may be overridden by URL
        //system_id = slave_link->get_system_id();
		//component_id = slave_link->get_component_id();

        slave_link_diag.set_mavconn(slave_link);
        updater_slave.setHardwareID(slave_url);
        updater_slave.add(slave_link_diag);
        slave_link_diag.set_connection_status(true);
        }
	}
	catch (mavconn::DeviceError &ex) {
		ROS_FATAL("iLink SLAVE LINK: %s", ex.what());
		slave_link_error = true;
	}

    if(master_link_error && slave_link_error){
        ROS_FATAL("iLink MASTER AND SLAVE LINK ERROR");
        return 1;
    }

    std::string fcu_protocol;
    priv_nh.param<std::string>("fcu_protocol", fcu_protocol, "v2.0");

    //setup mavlink protocol
    if (fcu_protocol == "v1.0") {
		mavros_link->set_protocol_version(mavconn::Protocol::V10);
        master_link->set_protocol_version(mavconn::Protocol::V10);
        slave_link->set_protocol_version(mavconn::Protocol::V10);
	}
	else if (fcu_protocol == "v2.0") {
        mavros_link->set_protocol_version(mavconn::Protocol::V20);
        master_link->set_protocol_version(mavconn::Protocol::V20);
        slave_link->set_protocol_version(mavconn::Protocol::V20);
	}
    else {
		ROS_WARN("Unknown FCU protocol: \"%s\", should be: \"v1.0\" or \"v2.0\". Used default v1.0.", fcu_protocol.c_str());
		mavros_link->set_protocol_version(mavconn::Protocol::V10);
        master_link->set_protocol_version(mavconn::Protocol::V10);
        slave_link->set_protocol_version(mavconn::Protocol::V10);
	}

    // setup publisher to monitor activity only
	mavlink_from_mavros_pub = mavlink_nh.advertise<mavros_msgs::Mavlink>("from_mavros", 10);
    mavlink_from_master_pub = mavlink_nh.advertise<mavros_msgs::Mavlink>("from_master", 10);
    mavlink_from_slave_pub = mavlink_nh.advertise<mavros_msgs::Mavlink>("from_slave", 10);

    //Receive and send mavlink

    mavros_link->message_received_cb = mavlink_from_mavros_pub_cb;
        
	

    mavros_link->port_closed_cb = []() {
		ROS_ERROR("MAVROS LINK connection closed, mavros will be terminated.");
		ros::requestShutdown();
	};

    if (master_link) {
		master_link->message_received_cb = mavlink_from_master_pub_cb;
           
	}

    if (slave_link) {
		slave_link->message_received_cb = mavlink_from_slave_pub_cb;
            
	}

	

	// updater spinner
	auto diag_timer_mavros = priv_nh.createTimer(ros::Duration(0.5),
			[&](const ros::TimerEvent &evt) {
				updater_mavros.update();
			});
	diag_timer_mavros.start();

    auto diag_timer_master = priv_nh.createTimer(ros::Duration(0.5),
			[&](const ros::TimerEvent &evt) {
				updater_master.update();
			});
	diag_timer_master.start();

    auto diag_timer_slave = priv_nh.createTimer(ros::Duration(0.5),
			[&](const ros::TimerEvent &evt) {
				updater_slave.update();
			});
	diag_timer_slave.start();

	ros::spin();
	return 0;
}
