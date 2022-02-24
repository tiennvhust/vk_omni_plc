#ifndef VK_OMNI_PLC_H
#define VK_OMNI_PLC_H

#include "CppLinuxSerial/SerialPort.hpp"
#include "vk_omni_plc_serial.h"
#include <memory>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>

using namespace mn::CppLinuxSerial;

namespace safety 
{
	std_msgs::UInt8 msg_(uint8_t data)
	{
		std_msgs::UInt8 msg;

		msg.data = data;

		return msg;
	}
	
	struct safetyPLCStatus
	{
		int resendCount; /*resend counter*/
	
		std_msgs::UInt8 lastCommand; /*store the lastest command*/
	
		bool flag; /*flag for communication error warning*/
	
		safetyPLCStatus() : resendCount(0),	flag(false)	{}

		void increaseResendCount() { resendCount++; }

		void resetResendCount() { resendCount = 0; }

		void raiseFlag() { flag = true; }

		void resetFlag() { flag = false; }
	};
	
	class safetyPLC 
	{
		private:
			ros::Publisher statusPublisher;

			// ros::Publisher statusPLCPublisher;
	
			ros::Subscriber commandSubscriber;
	
			std::unique_ptr<SerialPort> port; /*serial port*/
		
			int id; /*machine ID*/
			
			std_msgs::UInt8 status; /*status*/

			std::unique_ptr<safetyPLCStatus> statusPLC;
		
		public:
			safetyPLC(ros::NodeHandle *nh); /*default constructor*/
		
			safetyPLC(ros::NodeHandle *nh, const std::string& device, int addr);
		
			~safetyPLC(); /*de-constructor*/
		
			void commandSend(std_msgs::UInt8 msg); /*send commands*/
		
			int responseRead(); /*handle response data*/
		
			void statusUpdate(uint8_t data); /*update status*/
		
			void resendHandler(); /*handle resend request*/
	};
}

#endif //VK_OMNI_PLC_H
