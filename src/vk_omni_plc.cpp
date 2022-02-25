#include "vk_omni_plc_serial.h"
#include "vk_omni_plc.h"
#include <std_msgs/UInt8.h>
#include <vector>
#include <ros/ros.h>

using namespace mn::CppLinuxSerial;
using namespace safety;

/*check sum*/
int CheckSum(std::vector<uint8_t> &data)
{
	uint16_t i;

	uint8_t csum;

	csum = 0;

	for(i = 0; i < data.size(); i++) {
		csum += data[i];
	}

	if(csum == 0) return 0;

	return -1;
}

/*add checksum value to packet*/
void AddCheckSum(std::vector<uint8_t> &data) {
	uint8_t chksum = 0;

	uint16_t i;

	for(i = 0; i < data.size(); i++) chksum += data[i];

	chksum = ~chksum + 1;

	data.push_back(chksum);
}

/*constructors*/
safetyPLC::safetyPLC(ros::NodeHandle *nh) : 
		id(156)
{
	statusPublisher = nh->advertise<std_msgs::UInt8>("robot_status", 1);

	commandSubscriber = nh->subscribe("safety_command", 10, &safetyPLC::commandSend, this);

	port = std::make_unique<SerialPort>(DEFAULT_DEVICE, DEFAULT_BAUDRATE);

	statusPLC = std::unique_ptr<safetyPLCStatus>(new safetyPLCStatus());

	port->Open();
}

safetyPLC::safetyPLC(ros::NodeHandle *nh, const std::string& device, int addr) :
		id(addr)
{
	statusPublisher = nh->advertise<std_msgs::UInt8>("robot_status", 1);

	commandSubscriber = nh->subscribe("safety_command", 10, &safetyPLC::commandSend, this);

	port = std::make_unique<SerialPort>(device, DEFAULT_BAUDRATE);

	statusPLC = std::unique_ptr<safetyPLCStatus>(new safetyPLCStatus());

	port->Open();
}

/*de-constructor*/
safetyPLC::~safetyPLC() {}

/*send commands*/
void safetyPLC::commandSend(std_msgs::UInt8 msg)
{
	/*save the command for later use*/
	statusPLC->lastCommand = msg;

	std::vector<uint8_t> buff;
	
	buff.push_back(ADDRESS);

	buff.push_back(CONTROL);

	buff.push_back(msg.data);

	AddCheckSum(buff);

	/*send message*/
	port->WriteBinary(buff);

	usleep(DEFAULT_WAIT_MICROS);

	while (responseRead() && !(statusPLC->flag))
	{
		resendHandler();
	}
}

/*handle response data*/
int safetyPLC::responseRead()
{
	std::vector<uint8_t> buff;

	port->ReadBinary(buff);

	if (CheckSum) return -1;

	if (buff[0] != ADDRESS || buff[1] != ACKNOWLEDGE || buff[2] > 13 || buff[2] < 8) return 1;
	
	switch(buff[3])
	{
		case RES_CODE:
		{
			return 2;
		}

		default:
		{
			if(statusPLC->resendCount != 0) statusPLC->resetResendCount();
			statusUpdate(buff[3]);
		}
	}
	return 0;
}

/*update current status*/
void safetyPLC::statusUpdate(uint8_t data)
{
	status.data = data;

	statusPublisher.publish(status);
}

/*handle resend request*/
void safetyPLC::resendHandler()
{
	if(statusPLC->resendCount > 3)
	{
		/*publish warning message*/
		statusPLC->raiseFlag();
		statusPLC->resetResendCount();
	}

	/*resend the lastest command*/
	commandSend(statusPLC->lastCommand);

	statusPLC->increaseResendCount();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vk_omni_plc");

	ros::NodeHandle nh;

	safetyPLC plc(&nh);

	ROS_INFO("Start vk_omni_plc node.\n");

	/*send ready message to plc*/
	plc.commandSend(msg_(RDY_CODE));

	ROS_INFO("Ready state.\n");

	ros::Duration(0.5).sleep();

	plc.commandSend(msg_(RST_CODE));

	ROS_INFO("Normal state.\n");

	ros::spin();
}
