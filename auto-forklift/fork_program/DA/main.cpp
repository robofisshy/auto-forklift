#include "motor_control.h"
#include "jetsonGPIO.h"
#include <unistd.h>

int main()
{
	MotorControl motor;
	int i=0;
	while(i<1000)
	{
		motor.Turn(1.1513);
//		gpioSetValue(motor.Area_set1,high);
//		sleep(1);
//		gpioSetValue(motor.Area_set1,low);
//		sleep(1);
		i++;
	}
	motor.close();
}
