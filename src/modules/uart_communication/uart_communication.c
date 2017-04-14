/**
* @file     : uart_communication.c
* @brief    : To get parafoil_attitude.
* @author   : libn
* @time     : Mar 28, 2017 4:49:26 PM
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <px4_config.h>
#include <nuttx/sched.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/parafoil_attitude.h>

#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <poll.h>

#include <fcntl.h>
#include <drivers/drv_hrt.h>
#include <termios.h>

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

/**
 * daemon management function.
 */
__EXPORT int uart_communication_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int uart_communication_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);
int set_uart_baudrate(const int fd, unsigned int baud);
int uart_init(char * uart_name);
int set_uart_baudrate(const int fd, unsigned int baud);

static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: daemon {start|stop|status} [-p <additional params>]\n\n");
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


/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int uart_communication_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("daemon already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("uart_communication",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 2000,
						 uart_communication_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");

		} else {
			warnx("\tnot started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

int uart_communication_thread_main(int argc, char *argv[])
{

	warnx("[daemon] starting\n");

	thread_running = true;

	/* subscribing. */
	int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
	/* limit the update rate to 50 Hz */
	orb_set_interval(sensor_sub_fd, 20);

//	/* subscribing. */
//	struct parafoil_attitude_s parafoil_attitude_data;
//	int parafoil_attitude_sub_fd = orb_subscribe(ORB_ID(parafoil_attitude));
//	/* limit the update rate to 5 Hz */
//	orb_set_interval(parafoil_attitude_sub_fd, 200);

	/* publishing. */
	struct parafoil_attitude_s parafoil_attitude_data;
	memset(&parafoil_attitude_data, 0, sizeof(parafoil_attitude_data));
	orb_advert_t parafoil_attitude_pub_fd = orb_advertise(ORB_ID(parafoil_attitude), &parafoil_attitude_data);


	/* uart init. -libn Mar 29, 2017 */
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
	/* buffer for parafoil_attitude message decoding. */
	char byte_data = '0';
	char in_buffer[64] = "";
	int index=0;

	/* wake up. */
	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	while (!thread_should_exit)
	{
		/* wait for up to 10ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 10);
		/* timed out - periodic check for _task_should_exit */
		if (pret == 0) {
			continue;
		}
		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("px4_daemon_app: poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(100000);
			continue;
		}

		if (fds[0].revents & POLLIN) {

			/* main loop. -libn Mar 28, 2017 */

			/* obtained data for the first file descriptor */
			struct sensor_combined_s raw;
			/* copy sensors raw data into local buffer */
			orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
//			PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
//				 (double)raw.accelerometer_m_s2[0],
//				 (double)raw.accelerometer_m_s2[1],
//				 (double)raw.accelerometer_m_s2[2]);


//			/* subscribe to parafoil_attitude topic. */
//			bool updated;
//			/* Check if data have changed */
//			orb_check(parafoil_attitude_sub_fd, &updated);
//			if(updated) {
//				/* copy sensors raw data into local buffer */
//				orb_copy(ORB_ID(parafoil_attitude), parafoil_attitude_sub_fd, &parafoil_attitude_data);
//				PX4_INFO("parafoil_attitude_data:\t%8.4f\t%8.4f\t%8.4f",
//					 (double)parafoil_attitude_data.parafoil_roll_angle,
//					 (double)parafoil_attitude_data.parafoil_pitch_angle,
//					 (double)parafoil_attitude_data.parafoil_yaw_angle);
//			}
//			else
//				PX4_INFO("No parafoil attitude update!");

			/* get parafoil_attitude message. */
			read(uart_read,&byte_data,1);
			printf("uart message = %x   index = %d\n",byte_data,index);
			in_buffer[index] = byte_data;

			/* buffer title. */
			/*
			 * comment:					title:
			 * acceleration				0x5551
			 * angle acceleration		0x5552
			 * angle(z-y-x rotation)	0x5553
			 */

			/* message length = 55 byte, so time consuming = 55 * 0.01s = 0.55s -> */

			/* message read & publish -begin. -libn Mar 29, 2017 */
			if ((index == 0)&&(in_buffer[index] == 0x55))	index =1;
			else if ((index == 1)&&(in_buffer[index] == 0x53)){
					index =2;
//					printf("Attitude message is coming!\n");
			}
			else if ((index >=2)&&(index <11))
					index++;
			else if (index ==11)
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

//				int temp = (RH<<8|RL);
//				float roll_angle_2 = (float)temp/32768.0f*180.0f;
//				printf("roll_angle_2 = %8.3f degree\n",(double)roll_angle_2);

				float roll_angle = (float)(RH<<8|RL)/32768.0f*180.0f;
				printf("roll angle = %8.3f degree\n",(double)roll_angle);
				float pitch_angle = (float)(PH<<8|PL)/32768.0f*180.0f;
				printf("pitch angle = %8.3f degree\n",(double)pitch_angle);
				float yaw_angle = (float)(YH<<8|YL)/32768.0f*180.0f;
				printf("yaw angle = %8.3f degree\n",(double)yaw_angle);
				float T_sensor = (float)(TH<<8|TL)/340.0f+36.53f;
				printf("Temperature = %8.3f degree\n",(double)T_sensor);

				/* publish to parafoil_attitude topic. */
				parafoil_attitude_data.parafoil_roll_angle = roll_angle;
				parafoil_attitude_data.parafoil_pitch_angle = pitch_angle;
				parafoil_attitude_data.parafoil_yaw_angle = yaw_angle;
				orb_publish(ORB_ID(parafoil_attitude),parafoil_attitude_pub_fd, &parafoil_attitude_data);
			}
			else
				index = 0;

			/* message read & publish -end. -libn Mar 29, 2017 */





		}





//		warnx("Hello daemon!\n");
//		sleep(10);
	}

	warnx("[daemon] exiting.\n");

	thread_running = false;

	return 0;
}
