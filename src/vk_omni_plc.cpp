#include "vk_omni_plc_serial.h"
#include "vk_omni_plc.h"

#include <chrono>

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
		id(156),
		threadAlive(true),
		flag(false)
{
	statusPublisher = nh->advertise<std_msgs::UInt8>("robot_status", 1);

	commandSubscriber = nh->subscribe("plc_commands", 10, &safetyPLC::commandSend, this);

	port = std::make_unique<SerialPort>(DEFAULT_DEVICE, DEFAULT_BAUDRATE);

	statusPLC = std::unique_ptr<safetyPLCStatus>(new safetyPLCStatus());

	/*open the serial port*/
	port->Open();

	/*start reading thread*/
	readThread = std::thread (&safetyPLC::responseHandler, this);
}

safetyPLC::safetyPLC(ros::NodeHandle *nh, const std::string& device, int addr) :
		id(addr),
		threadAlive(true),
		flag(false)
{
	statusPublisher = nh->advertise<std_msgs::UInt8>("robot_status", 1);

	commandSubscriber = nh->subscribe("plc_commands", 10, &safetyPLC::commandSend, this);

	port = std::make_unique<SerialPort>(device, DEFAULT_BAUDRATE);

	statusPLC = std::unique_ptr<safetyPLCStatus>(new safetyPLCStatus());

	/*open the serial port*/
	port->Open();

	/*start reading thread*/
	readThread = std::thread (&safetyPLC::responseHandler, this);
}

/*de-constructor*/
safetyPLC::~safetyPLC()
{
	threadAlive = false;

	readThread.join();

	port->Close();
}

/*send commands*/
void safetyPLC::commandSend(std_msgs::UInt8 msg)
{
	/*save the command for later use*/
	statusPLC->lastCommand = msg;

	std::vector<uint8_t> buff;

	buff.reserve(4);
	
	buff.emplace_back(ADDRESS);

	buff.emplace_back(CONTROL);

	buff.emplace_back(msg.data);

	AddCheckSum(buff);

	/*send message*/
	port->WriteBinary(buff);
}

void safetyPLC::responseHandler()
{
	while(threadAlive)
	{
		if(port->Available()) responseRead();
	}
}

/*handle response data*/
int safetyPLC::responseRead()
{
	std::vector<uint8_t> buff;

	while (port->Available())
	{
		port->ReadBinary(buff); /*return after timeout*/
	}

	if (CheckSum(buff)) return -1;

	if (buff[0] != ADDRESS || buff[1] != ACKNOWLEDGE || buff[2] > 13 || buff[2] < 8) return 1;
	
	switch(buff[2])
	{
		case RES_CODE:
		{
			if(flag)
			{
				flagDown();

				return -2;
			}
			resendHandler();

			return 2;
		}

		default:
		{
			if(statusPLC->resendCount != 0) statusPLC->resetResendCount();

			statusUpdate(buff[2]);

			return 0;
		}
	}
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
	statusPLC->increaseResendCount();

	if (statusPLC->resendCount > 3) flagUp();

	/*resend the lastest command*/
	commandSend(statusPLC->lastCommand);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vk_omni_plc");

	ros::NodeHandle nh;

	safetyPLC plc(&nh);

	ROS_INFO("vk_omni_plc started.\n");

	/*send ready message to plc*/
	plc.commandSend(msg_(RDY_CODE));

	ROS_INFO("Ready state.\n");

	ros::spin();
}
