#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_defines.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h> //strcasecmp
#include <unistd.h>  //read
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
#include<math.h>
#include<px4_tasks.h>
#include<uORB/uORB.h>
#include<uORB/topics/TSC.h>
#include<uORB/topics/PM25.h>

/************************函数声明区********************************/
static int uart_init(const char * uart_name);  //关联串口
static int uart_init1(const char * uart_name);  //关联串口
static int set_uart_baudrate(const int fd, unsigned int baud); //设置波特率
__EXPORT int airsensor_main(int argc, char *argv[]);//主函数
int airsensor_main1(int argc, char *argv[]); //daemon

/**********************************************************/
static bool start=false;
/*********************************************************/
/*******************实现模块********************************/

int airsensor_main(int argc, char *argv[])
{
  if(!strcasecmp(argv[1],"start"))
  {
   if(start)
   {
	   PX4_INFO("already running!");
	   return 0;
   }
   start=true;
   px4_task_spawn_cmd("faemon1",SCHED_DEFAULT,SCHED_PRIORITY_DEFAULT,3600,airsensor_main1,(char*const*)NULL);
   return 0;
  }
  else if(!strcasecmp(argv[1],"stop"))
    {
      start=false;
      return 0;
    }
  else if(!strcasecmp(argv[1],"check"))
    {
      if(start)
    	  PX4_INFO("already running!");
      else
    	  PX4_INFO("Never run!");
    }
  else
	  PX4_INFO("Enter Error!");
  return 0;
}
int airsensor_main1(int argc, char *argv[])  //start为true时始终执行
{
	int f1=uart_init("/dev/ttyS1");
	if(f1<0)//说明打开失败
	{
		PX4_INFO("port failed to open!");
		return -1;
	}
	int f2=set_uart_baudrate(f1,9600);  //树莓派硬件串口默认波特率9600
	if(f2<=0)
	{
		PX4_INFO("baudrate set failed!");
		return -1;
	}
	int f4=uart_init1("/dev/ttyS6");
	if(f4<0)
	{
		PX4_INFO("port failed to open!");
		return -1;
	}

	int f5=set_uart_baudrate(f4,9600);
	if(f5<=0)
	{
		PX4_INFO("baudrate set failed!");
		return -1;
	}
	PX4_INFO("wait struct data!");
    char rec,cc[9];
    double temp,shi;
    unsigned ppm;
    /*****PM2.5*******/
    char pp[8];
    long sumlowtime;
    double per,concentration;
    /***********double temp,shi,concentration+unsigned ppm***************/
    struct TSC_s T1;
    struct PM25_s P1;
    /****************************************/
    void* Thandle=orb_advertise(ORB_ID(TSC),&T1);
    void* Phandle=orb_advertise(ORB_ID(PM25),&P1);
    while(start)
    {
       if(read(f1,&rec,1)>0)
       {
    	   /********************CO2,温湿度包*************************************/
    	           if(rec=='a')
    	           {
    	           	for(int i=0;i<9;i++)
    	           	{
    	           		read(f1,&cc[i],1);
    	           	}
    	           	ppm=10*((cc[4]-48)+(cc[3]-48)*10+(cc[2]-48)*100+(cc[1]-48)*1000+(cc[0]-48)*10000);
    	           	temp=cc[7]+cc[8]/256.0;
    	           	shi=cc[5]+cc[6]/256.0;
    	           	T1.ppm=ppm;
    	           	T1.shi=shi;
    	           	T1.temp=temp;
    	           	orb_publish(ORB_ID(TSC),Thandle,&T1);
    	           //	printf("CO2:%d ppm temp:%.1f 'C shi:%.1f RH\n",ppm,temp,shi);
    	           }
       }
        /************************PM2.5包***************************/
       if(read(f4,&rec,1)>0)
       {
    	   if(rec=='b')
    	   {
    		   for(int i=0;i<8;i++)
    		   {
    			   read(f4,&pp[i],1);
    		   }
    		   sumlowtime=(pp[0]-48)+(pp[1]-48)*10+(pp[2]-48)*100+(pp[3]-48)*1000+
    				   (pp[4]-48)*1000*10+(pp[5]-48)*1000*100+(pp[6]-48)*1000*1000+(pp[7]-48)*1000*10000;
               per=sumlowtime/(30000*10.0);
               concentration=1.1*pow(per,3)-3.8*pow(per,2)+520*per+0.62;
               P1.pm25=concentration;
               orb_publish(ORB_ID(PM25),Phandle,&P1);
              // printf("PM2.5: %.1f pcs/0.01cf\n",concentration);
    	   }
       }
       usleep(50);
    }
	return 0;
}
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
int uart_init(const char * uart_name)
{
    int serial_fd = open(uart_name, O_RDONLY);

    if (serial_fd < 0) {
        PX4_INFO("failed to open port!");
        return -1;
    }
    return serial_fd;
}
int uart_init1(const char * uart_name)
{
    int serial_fd = open(uart_name, O_RDONLY|O_NONBLOCK);

    if (serial_fd < 0) {
        PX4_INFO("failed to open port!");
        return -1;
    }
    return serial_fd;
}



