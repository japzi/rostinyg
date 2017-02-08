/*
  Author: Ruwan J Egodagamage
*/ 

#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>

#include "ros/ros.h"
#include "rostinyg/position.h"
#include <sstream>
#include <iostream>
#include "json/json.hpp"
#include <vector>

typedef websocketpp::client<websocketpp::config::asio_client> client;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;
// pull out the type of messages sent by our config
typedef websocketpp::config::asio_client::message_type::ptr message_ptr;

// for convenience
using json = nlohmann::json;
using namespace std;

bool initialized = false;
std::vector<string> initialStatusRequests;
float posX = 0.0f;
float posY = 0.0f;
float posZ = 0.0f;

ros::Publisher positionPublisher;

void sendPositionMessage() {
    rostinyg::position msg;
    msg.posX = posX;
    msg.posY = posY;
    msg.posZ = posZ;
    msg.timestamp = ros::Time::now();
    positionPublisher.publish(msg);
    ros::spinOnce();
}

void onOpen(client* c, websocketpp::connection_hdl hdl) {
    if (!initialized) {
        initialized = true;
        for (int i = 0; i < initialStatusRequests.size(); i++) {
            websocketpp::lib::error_code ec;
            c->send(hdl, initialStatusRequests[i].c_str(), websocketpp::frame::opcode::text, ec);
            if (ec) {
                cout << "Status reports requesting failed because : " << ec.message() << endl;
            }
        }
    }
}

// This message handler will be invoked once for each incoming message. It
// prints the message and then sends a copy of the message back to the server.
void onMessage(client* c, websocketpp::connection_hdl hdl, message_ptr msg) {
    try {
        json result = json::parse(msg->get_payload());
        if (result.find("D") != result.end()) {
            json data = json::parse(result["D"].get<string>());
            json sr;
            bool srFound = false;
            if (data.find("r") != data.end()) {
                json r = data["r"];
                cout << r << endl;
                if (r.find("sr") != r.end()) {
                    sr = r["sr"];
                    cout << sr << endl;
                    srFound = true;
                }
            }
            else if (data.find("sr") != data.end()) {
                sr = data["sr"];
                srFound = true;
            }

            if (srFound) {
                if (sr.find("posx") != sr.end())
                    posX = sr["posx"];
                if (sr.find("posy") != sr.end())
                    posY = sr["posy"];
                if (sr.find("posz") != sr.end())
                    posZ = sr["posz"];
                sendPositionMessage();
            }
        }
    }
    catch (...) {
        // non JSON payloads like "sendjson {}" are captured here. 
        // cout << msg->get_payload() << endl;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rostinyg");
    ROS_INFO("rostinyg started");

    string uri = "http://localhost:8989/ws";
    if (ros::param::get("~serial_port_json_server_url", uri))
	{
		ros::param::del("~serial_port_json_server_url");
	}

    string usbPort = "/dev/ttyUSB0";
    if (ros::param::get("~usb_port", usbPort))
	{
		ros::param::del("~usb_port");
	}
    
    int timeBetweenStatusReports = 100;
    if (ros::param::get("~report_delay", timeBetweenStatusReports))
	{
		ros::param::del("~report_delay");
	}

    string cmdBegin = "sendjson {\"P\":\"";
    cmdBegin += usbPort;
    cmdBegin += "\",\"Data\":[{\"D\":\"";
    string cmdEnd = "}]}";

    initialStatusRequests.push_back(cmdBegin + "\"{\\\"sr\\\":n}\\n\"" + cmdEnd);
    initialStatusRequests.push_back(cmdBegin + "\"{\\\"si\\\":" + to_string(timeBetweenStatusReports) + "}\\n\"" + cmdEnd);

    ros::NodeHandle n;
    positionPublisher = n.advertise<rostinyg::position>("position", 1000); // params: topic name, size of the queue  
    
    // Create a client endpoint
    client c;

    try {
        // Set logging to be pretty verbose (everything except message payloads)
        c.set_access_channels(websocketpp::log::alevel::all);
        c.clear_access_channels(websocketpp::log::alevel::frame_payload);

        // Initialize ASIO
        c.init_asio();

        // Register our message handler
        c.set_open_handler(bind(&onOpen,&c,::_1));
        c.set_message_handler(bind(&onMessage,&c,::_1,::_2));

        websocketpp::lib::error_code ec;
        client::connection_ptr con = c.get_connection(uri, ec);
        if (ec) {
            cout << "could not create connection because: " << ec.message() << endl;
            return 0;
        }

        // Note that connect here only requests a connection. No network messages are
        // exchanged until the event loop starts running in the next line.
        c.connect(con);

        // Start the ASIO io_service run loop
        // this will cause a single connection to be made to the server. c.run()
        // will exit when this connection is closed.
        c.run();
    } 
    catch (websocketpp::exception const & e) {
        cout << e.what() << endl;
    }

    return 0;
}
