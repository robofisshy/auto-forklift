---fork 为叉车定位、运动控制等代码
        
主要由detect、location、motor_control、multi_thread等几个类组成
        
detect：障碍检测线程
 	
location:定位线程
	
multi_thread:线程类
motor_control：运动跟踪
、驱动程序。使用Can总线控制叉车驱动电机，使用DA模拟编码器信号控制叉车转向电机。

其中DA驱动部分为自己编码实现，详见src/motor_control/jetsonGPIO.cpp。Can总线使用Can驱动设备提供驱动，位于lib/libcontrolcan.so
PS:该驱动是针对TX1平台，若要换置其它平台，若不兼容的话需要联系厂家重新编译库。
		
main函数使用fork.start()开启各线程，工作开始

---DA   为叉车转向电机初始化程序
 	
叉车打开开关后，若转向电机A通道和B通道初始值无效会报出3154错误
	
该程序配合DA_Init脚本实现转向电机初始化

---radar_detect.ino为arduino程序，负责拉线编码器和激光雷达数据读取
	 
