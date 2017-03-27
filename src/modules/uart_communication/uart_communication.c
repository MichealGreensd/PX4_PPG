/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Example User <mail@example.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file uart_communication.c
 * @Author: libn 2017-3-20
 * @uart_communication:
 * parafoil attitude sensor driver for PX4 autopilot
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <termios.h>
#include <stdbool.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <fcntl.h>
//#include <modules/uart_communication/data_process.h>
#include <uORB/topics/parafoil_attitude_sensor.h>
#include <px4_tasks.h>

//#include <poll.h>	/* to add poll request -libn Mar 21, 2017 */
//#include <uORB/topics/sensor_combined.h>

static bool thread_should_exit = false;		/**< uart_communication exit flag */
static bool thread_running = false;		/**< uart_communication status flag */
static int daemon_task;				/**< Handle of uart_communication task / thread */

/**
 * management function.
 */
__EXPORT int uart_communication_main(int argc, char *argv[]);

/**
 * Mainloop
 */
int uart_communication_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

/**
 * uart initial.
 */
static int uart_init(char * uart_name);


static int set_uart_baudrate(const int fd, unsigned int baud);


static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	errx(1, "usage: daemon {start|stop|status} [-p <additional params>]\n\n");
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


int uart_init(char * uart_name)
{
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

    if (serial_fd < 0) {
        err(1, "failed to open port: %s", uart_name);
        return false;
    }
    return serial_fd;
}

int uart_communication_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("uart_communication already running\n");
			/* this is not an error */
			return 1;
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("uart_communication",
							   SCHED_DEFAULT,
							   SCHED_PRIORITY_MAX - 5,
							   2500,
							   uart_communication_thread_main,
							   (argv) ? (char * const *)&argv[2] : (char * const *)NULL);   //从第2个地址(agrv[2])开始，进行指针的传递!!!argc也会对应发生变化
		return 1;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");

		} else {
			warnx("\tnot started\n");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

int uart_communication_thread_main(int argc, char *argv[])
{
	thread_running = true;
    printf("Hello uart_communication!\n");

//    /* subscribe to sensor_combined topic */
//    int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));     // 订阅sensor_combined
//    orb_set_interval(sensor_sub_fd, 1000);
//    /* one could wait for multiple topics with this technique, just using one here */
//	struct pollfd fds[] = {
//		{ .fd = sensor_sub_fd,   .events = POLLIN },
//		/* there could be more file descriptors here, in the form like:
//		 * { .fd = other_sub_fd,   .events = POLLIN },
//		 */
//	};

//    char out_buffer[50];
    char byte_data = '0';
    char in_buffer[64] = "";
    int index=0;
    int index2=0;

    /*
     * TELEM1 : /dev/ttyS1
     * TELEM2 : /dev/ttyS2
     * GPS    : /dev/ttyS3
     * NSH    : /dev/ttyS5
     * SERIAL4: /dev/ttyS6
     * N/A    : /dev/ttyS4
     * IO DEBUG (RX only):/dev/ttyS0
     */
    int uart_read = uart_init("/dev/ttyS6");

    if(false == uart_read)return -1;
    if(false == set_uart_baudrate(uart_read,115200))
    {
        printf("[YCM]set_uart_baudrate is failed\n");
        return -1;
    }

//    /* write to uart. -libn Mar 21, 2017 */
//    sleep(10);
//    sprintf(out_buffer,"AT+SMPR=200\r\n");
//    usleep(100);
//    write(uart_read,&out_buffer,strlen(out_buffer));
//    sprintf(out_buffer,"AT+GSD\r\n");
//    write(uart_read,&out_buffer,strlen(out_buffer));
//    usleep(10);
//    printf("[coupling force]uart init is successful\n");
////    usleep(100000);	/* one second. -libn */
//    printf("usleep is over\n");

    struct parafoil_attitude_sensor_s parafoil_attitude_sensor_data;
    memset(&parafoil_attitude_sensor_data, 0, sizeof(parafoil_attitude_sensor_data));
    orb_advert_t parafoil_attitude_sensor_pub_fd = orb_advertise(ORB_ID(parafoil_attitude_sensor), &parafoil_attitude_sensor_data);

    while(!thread_should_exit){

//        /* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
//        int poll_ret = poll(fds, 1, 1000);                          // 线程等待数据更新, timeout为1s,
//                                                                    // 如果数据没更新, 那么此时系统会进行任务切换.
//        /* handle the poll result */
//        if (poll_ret == 0) {
//            /* this means none of our providers is giving us data */
//            printf("[px4_simple_app] Got no data within a second\n");
//        } else if (poll_ret < 0) {
//            /* this is seriously bad - should be an emergency */
//        } else {
//            if (fds[0].revents & POLLIN) {
//
//            }
//            /* there could be more file descriptors here, in the form like:
//             * if (fds[1..n].revents & POLLIN) {}
//             */
//        }


    	read(uart_read,&byte_data,1);
		in_buffer[index] = byte_data;
		printf("message got: %x\n",byte_data);

		/* buffer title: -libn Mar 21, 2017 */
		/*
		 * comment:				title:
		 * acceleration			0x5551
		 * angle acceleration	0x5552
		 * angle(z-y-x rotation)	0x5553
		 */

		/* angles. -libn Mar 23, 2017 */
		if ((index == 0)&&(in_buffer[index] == 0x55))
				index =1;
		else if ((index == 1)&&(in_buffer[index] == 0x53)){
				index =2;
				printf("Attitude message is coming!\n");
		}
		else if ((index >= 2)&&(index < 11))
				index++;
		else if (index == 11)
		{
			int title;
			title = in_buffer[0]<<8|in_buffer[1];
			printf("Title confirmed(att_message -> 0x5553): title_x = %x\n",title);

			index = 0;
			printf("I get the attitude message!\n");

			char RL = in_buffer[2],RH = in_buffer[3];
			printf("RL = %x\tRH = %x\n",RL,RH);
			char PL = in_buffer[4],PH = in_buffer[5];
			printf("PL = %x\tPH = %x\n",PL,PH);
			char YL = in_buffer[6],YH = in_buffer[7];
			printf("YL = %x\tYH = %x\n",YL,YH);
			char TL = in_buffer[8],TH = in_buffer[9];
			printf("TL = %x\tTH = %x\n",TL,TH);

//        	int temp = (RH<<8|RL);
//        	float roll_angle_2 = (float)temp/32768.0f*180.0f;
//        	printf("roll_angle_2 = %8.3f degree\n",(double)roll_angle_2);

			float roll_angle = (float)(RH<<8|RL)/32768.0f*180.0f;
			printf("roll angle = %8.3f degree\n",(double)roll_angle);
			float pitch_angle = (float)(PH<<8|PL)/32768.0f*180.0f;
			printf("pitch angle = %8.3f degree\n",(double)pitch_angle);
			float yaw_angle = (float)(YH<<8|YL)/32768.0f*180.0f;
			printf("yaw angle = %8.3f degree\n",(double)yaw_angle);
			float T_sensor = (float)(TH<<8|TL)/340.0f+36.53f;
			printf("Temperature = %8.3f degree\n",(double)T_sensor);


			//force_sensor_data_decode(coupling_force_buff,in_buffer);
			//usleep(1000);
			//printf("%8.4f\n",(double)coupling_force_buff[2]);

			parafoil_attitude_sensor_data.parafoil_roll_angle = roll_angle;
			parafoil_attitude_sensor_data.parafoil_pitch_angle = pitch_angle;
			parafoil_attitude_sensor_data.parafoil_yaw_angle = yaw_angle;
//        	parafoil_attitude_sensor.force_x = coupling_force_buff[0];
//        	parafoil_attitude_sensor.force_y = coupling_force_buff[1];
//        	parafoil_attitude_sensor.force_z = coupling_force_buff[2];
//        	parafoil_attitude_sensor.moment_x = coupling_force_buff[3];
//        	parafoil_attitude_sensor.moment_y = coupling_force_buff[4];
//        	parafoil_attitude_sensor.moment_z = coupling_force_buff[5];
			orb_publish(ORB_ID(parafoil_attitude_sensor),parafoil_attitude_sensor_pub_fd, &parafoil_attitude_sensor_data);
		}
		else
			index = 0;

		/* angle_rates -libn Mar 23, 2017 */
		if ((index2 == 0)&&(in_buffer[index2] == 0x55))
				index2 =1;
		else if ((index2 == 1)&&(in_buffer[index2] == 0x52)){
				index2 =2;
				printf("Attitude_rate message is coming!\n");
		}
		else if ((index2 >= 2)&&(index2 < 11))
				index2++;
		else if (index2 == 11)
		{
			int title;
			title = in_buffer[0]<<8|in_buffer[1];
			printf("Title confirmed(att_rate_message -> 0x5552): title_x = %x\n",title);

			index2 = 0;
			printf("I get the attitude rate message!\n");

			char RL = in_buffer[2],RH = in_buffer[3];
			printf("RL = %x\tRH = %x\n",RL,RH);
			char PL = in_buffer[4],PH = in_buffer[5];
			printf("PL = %x\tPH = %x\n",PL,PH);
			char YL = in_buffer[6],YH = in_buffer[7];
			printf("YL = %x\tYH = %x\n",YL,YH);
			char TL = in_buffer[8],TH = in_buffer[9];
			printf("TL = %x\tTH = %x\n",TL,TH);

//        	int temp = (RH<<8|RL);
//        	float roll_angle_2 = (float)temp/32768.0f*180.0f;
//        	printf("roll_angle_2 = %8.3f degree\n",(double)roll_angle_2);

			float roll_angle_rate = (float)(RH<<8|RL)/32768.0f*2000.0f;
			printf("roll angle rate = %8.3f degree/s\n",(double)roll_angle_rate);
			float pitch_angle_rate = (float)(PH<<8|PL)/32768.0f*2000.0f;
			printf("pitch angle rate = %8.3f degree/s\n",(double)pitch_angle_rate);
			float yaw_angle_rate = (float)(YH<<8|YL)/32768.0f*2000.0f;
			printf("yaw angle rate = %8.3f degree/s\n",(double)yaw_angle_rate);
			float T_sensor = (float)(TH<<8|TL)/340.0f+36.53f;
			printf("Temperature = %8.3f degree\n",(double)T_sensor);


			//force_sensor_data_decode(coupling_force_buff,in_buffer);
			//usleep(1000);
			//printf("%8.4f\n",(double)coupling_force_buff[2]);

			parafoil_attitude_sensor_data.parafoil_roll_rate = roll_angle_rate;
			parafoil_attitude_sensor_data.parafoil_pitch_rate = pitch_angle_rate;
			parafoil_attitude_sensor_data.parafoil_yaw_rate = yaw_angle_rate;
//        	parafoil_attitude_sensor.force_x = coupling_force_buff[0];
//        	parafoil_attitude_sensor.force_y = coupling_force_buff[1];
//        	parafoil_attitude_sensor.force_z = coupling_force_buff[2];
//        	parafoil_attitude_sensor.moment_x = coupling_force_buff[3];
//        	parafoil_attitude_sensor.moment_y = coupling_force_buff[4];
//        	parafoil_attitude_sensor.moment_z = coupling_force_buff[5];
//					orb_publish(ORB_ID(parafoil_attitude_sensor),parafoil_attitude_sensor_pub_fd, &parafoil_attitude_sensor_data);
		}
		else
			index2 = 0;
		/**/

//		orb_publish(ORB_ID(parafoil_attitude_sensor),parafoil_attitude_sensor_pub_fd, &parafoil_attitude_sensor_data);

    }
    thread_running = false;
    return 0;
}
