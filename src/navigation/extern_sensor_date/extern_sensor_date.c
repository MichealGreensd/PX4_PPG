/*
 * extern_sensor_date.c
 *
 *  Created on: May 8, 2017
 *      Author: mrfish
 */

/*******************头文件区************************************/
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_defines.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>  //read函数
#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <termios.h>
#include <stdbool.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <poll.h>
#include <float.h>
#include <fcntl.h> //open函数的头文件
#include<uORB/uORB.h>
#include<uORB/topics/extern_sensor.h>

/***********************功能说明*****************************************/
//通过串口读取外部传感器数据,获得俯仰,偏航以及翻滚角度再把他们发布到topic中
//其他线程可以订阅该topic获取俯仰,偏航以及翻滚的角度信息
/************************说明结束****************************************/

/***************使用函数总览***********************/
 static int uart_init(const char * uart_name);  //关联串口
 static int set_uart_baudrate(const int fd, unsigned int baud); //设置波特率
__EXPORT int extern_sensor_date_main(int argc, char *argv[]);//主函数,按数据格式自行解包即可

/****************函数使用说明**************************/
//波特率设定函数
//传入参数两个:成功驱动串口后的返回值fd,设定的波特率baud
//传回参数一个:成功设定波特率返回值大于0,失败返回值小于等于0
int set_uart_baudrate(const int fd, unsigned int baud)
{
    int speed;

    switch (baud) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:
            warnx("ERR: baudrate: %d\n", baud);
            return -EINVAL;
  }

    struct termios uart_config;

    int termios_state;

    /* fill the struct for the new configuration */
    tcgetattr(fd, &uart_config);
    /* clear ONLCR flag (which appends a CR for every LF) */
    uart_config.c_oflag &= ~ONLCR;
    /* no parity, one stop bit */
    uart_config.c_cflag &= ~(CSTOPB | PARENB);
    /* set baud rate */
    if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetispeed)\n", termios_state);
        return false;
    }

    if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetospeed)\n", termios_state);
        return false;
    }

    if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
        warnx("ERR: %d (tcsetattr)\n", termios_state);
        return false;
    }

    return true;
}

//串口驱动函数
//传入参数一个:指定串口对应字符串
//传回参数一个:失败返回-1,成功返回非负整数
     /*
     * TELEM1 : /dev/ttyS1
     * TELEM2 : /dev/ttyS2
     * GPS    : /dev/ttyS3
     * NSH    : /dev/ttyS5
     * SERIAL4: /dev/ttyS6
     * N/A    : /dev/ttyS4
     * IO DEBUG (RX only):/dev/ttyS0
     */
int uart_init(const char * uart_name)
{
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

    if (serial_fd < 0) {
        PX4_INFO("failed to open port!");
        return -1;
    }
    return serial_fd;
}
int extern_sensor_date_main(int argc, char *argv[])
{
	int f1=uart_init("/dev/ttyS2");
	if(f1<0)//说明打开失败
	{
		PX4_INFO("port failed to open!");
		return 0;
	}
	int f2=set_uart_baudrate(f1,115200);
	if(f2<=0)
	{
		PX4_INFO("baudrate set failed!");
		return 0;
	}
	PX4_INFO("S2 port set successed!");
	char ch='0';
	char buffer[33];
	buffer[0]=0x55;
	buffer[1]=0x51;
	float roll,pitch,yaw;
	bool cc=false;
	struct extern_sensor_s coco;
    void*para_handle=orb_advertise(ORB_ID(extern_sensor),&coco);
	while(1)
    {
       read(f1,&ch,1);
       if((ch>=0x55)&&(ch<=0x55))
       {
    	   read(f1,&ch,1);
    	   if((ch>=0x51)&&(ch<=0x51))//说明为帧数据
    	   {
    		   for(int i=2;i<33;i++)
    		   {
    			   read(f1,&ch,1);
    			   buffer[i]=ch;
    		   }
    		   cc=true;
    	   }
       }
      //buffer[24],buffer[25]决定roll
       //buffer[26],buffer[27]决定pitch
       //buffer[28],buffer[29]决定yaw
       while(cc)
       {
    	   cc=false;
    	   if(((buffer[12]>=0x52)&&(buffer[12]<=0x52))&&((buffer[23]>=0x53)&&(buffer[23]<=0x53)))  //确认的确是帧数据
    	   {
    		   /***************处理角度转换*****************************/
    		      	         roll=((buffer[25]<<8)|buffer[24])*1.0/32768*180;
    		      	         pitch=((buffer[27]<<8)|buffer[26])*1.0/32768*180;
    		      	         yaw=((buffer[29]<<8)|buffer[28])*1.0/32768*180;
    		      	         /******************处理结束**********************/
    		      	         /**************角度规定,超过180为负角度******************/
    		      	         if(roll>180)
    		      	        	 roll=roll-360;
    		      	         if(pitch>180)
    		      	        	 pitch=pitch-360;
    		      	         if(yaw>180)
    		      	        	 yaw=yaw-360;
    		      	         /************处理结束**********************************/
    		      	     //       PX4_INFO("roll=%.2f,pitch=%.2f,yaw=%.2f",roll,pitch,yaw);
    		      	         coco.parafoil_roll_angle=roll;
    		      	         coco.parafoil_pitch_angle=pitch;
    		      	         coco.parafoil_yaw_angle=yaw;
    		      	        orb_publish(ORB_ID(extern_sensor),para_handle,&coco);
    		      	         usleep(2000);//每间隔2ms更新一次topic数据
    	   }
       }

    }
	return 0;
}





