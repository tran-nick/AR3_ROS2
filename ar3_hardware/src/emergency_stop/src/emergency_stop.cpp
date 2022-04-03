#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>		// write(), read, close()
#include <fcntl.h>		// contains file controls
#include <errno.h>		// contains error int and strerror func
#include <termios.h>	// posix terminal control funs
#include "emergency_stop/emergency_stop.hpp"


namespace emergency_stop{

float EmergencyStop::sensor_read(void)
{
    fd = open("/dev/hcsr04", O_RDWR | O_NOCTTY | O_NDELAY);
	if(fd == -1)
	{
		printf("Error writing!");
        RCLCPP_INFO(rclcpp::get_logger("EmergencyStop"),"Failed to connect driver port /dev/hcsr04\n");
    }
	else
    {
		fcntl(fd, F_SETFL, 0);	// reading blocking behavior
    }
    
    int ret;
    char buff[10];
    ret = read(fd,buff,sizeof(buff));

    if(ret<0)
    {   
         RCLCPP_INFO(rclcpp::get_logger("EmergencyStop"),"Error with reading /dev/hcsr04\n");
        return EXIT_FAILURE;
    }
 
    close(fd);          // close serial port 
    return atof(buff);  //string to float
}

int EmergencyStop::button_read(void)
{
    fd1 = open("/dev/estop_clr", O_RDWR | O_NOCTTY | O_NDELAY);
    	if(fd1 == -1)
	{
		printf("Error writing!");
        RCLCPP_INFO(rclcpp::get_logger("EmergencyStop"),"Failed to connect driver port /dev/estop_clr\n");
    }
	else
    {
		fcntl(fd1, F_SETFL, 0);	// reading blocking behavior
    }

    int ret;
    char buff[10];
    ret = read(fd1,buff,sizeof(buff));
    
    if(ret<0)
    {   
         RCLCPP_INFO(rclcpp::get_logger("EmergencyStop"),"Error with reading /dev/estop_clr\n");
        return EXIT_FAILURE;
    }
 
    close(fd1);          // close serial port 
    return std::stoi(buff);  //string to float
}



} // namespace estop_driver