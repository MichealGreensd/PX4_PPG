/**
 * @file px4_simple_app.c
 * examples for adding app to pxihawk -- huangling
 * 2016/3/12
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <nuttx/config.h>
#include <stdlib.h>
#include <nuttx/sched.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>

/* custom message */
#include <uORB/topics/px4_test.h>
#include <uORB/topics/parafoil_attitude_sensor.h>

static bool thread_should_exit = false;     /**< px4_simple_app exit flag */
static bool thread_running = false;         /**< px4_simple_app status flag */
static int  px4_simple_task;                /**< Handle of px4_simple_app task / thread */
// 线程管理程序
__EXPORT int px4_simple_app_main(int argc, char *argv[]);
/**
 * Mainloop of px4_simple_appp
 * 用户线程, 执行用户代码
 */
int px4_simple_thread_main(int argc, char *argv[]);
/**
 * print the correct usage for px4_simple_app operating
 */
static void usage(const char *reason);
static void
usage(const char *reason)
{
    if(reason)
        warnx("%s\n", reason);
    errx(1,"usage: px4_simple_app {start|stop|status} [-p <additional params>]\n\n");
}
/**
 * px4_simple_app模块的前台管理程序, 由shell启动, 可以控制模块的线程的启动和停止.
 */
int px4_simple_app_main(int argc, char *argv[])
{
    if(argc < 2)	{
    	warnx("usage: px_simple_app {start|stop|status}");
		return 1;
    }
    if(!strcmp(argv[1],"start"))	{
    	if(thread_running){
    		warnx("px_simple_app already running\n");
    		return 1;
    	}
    	thread_should_exit = false;     // 将线程状态位设置为false
    	// 创建线程, ucos非常相同.
		/* start the task */
		px4_simple_task = px4_task_spawn_cmd("px4_simple_app",
							   SCHED_DEFAULT,
							   SCHED_PRIORITY_MAX - 5,
							   1700,
							   px4_simple_thread_main,
							   (argv) ? (char * const *)&argv[2] : (char * const *)NULL);   //从第2个地址(agrv[2])开始，进行指针的传递!!!argc也会对应发生变化
		return 1;                      // 正常退出
    }
    if (!strcmp(argv[1], "stop")) {     // shell停止命令
		thread_should_exit = true;
		exit(0);
	}
	if (!strcmp(argv[1], "status")) {   // shell查询命令, 用于查询线程的状态.
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
/**
 * px4_simple_app的后台线程, 用于执行用户任务
 */
int px4_simple_thread_main(int argc, char *argv[])
{
    printf("Hello Sky!\n");

    printf("argc(thread_main) = %d\n",argc);
    //第1个字符串对应的指针为argv[1]!
    //px4_task_spawn_cmd命令中，从第2个地址(agrv[2])开始，进行指针的传递，所以进程中的程序，识别到的argv[1]即为原程序(px4_task_spawn_cmd)中的argv[2]!! argc也会对应发生变化！
    if(argc > 1)
    {
    	int num1 = atoi(argv[1]);
		printf("parameter_num1 = %d\n",num1);
    	if(argc > 2)
    	{
    		int num2 = atoi(argv[2]);
			printf("parameter_num2 = %d\n",num2);
    		if(argc > 3)
    		{
    		    int num3 = atoi(argv[3]);
				printf("parameter_num3 = %d\n",num3);
    		}
    	}
    }

    if (argc < 1)
        printf("missing command");
    /* subscribe to sensor_combined topic */
    int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));     // 订阅sensor_combined
    orb_set_interval(sensor_sub_fd, 1000);                          // 设置topic(bus)数据读取最小时间间隔.
                                                                    // 即如果没有1s, 那么认为数据还是旧的.
    /* subscribe to sensor_combined topic */
	int parafoil_attitude_sensor_sub_fd = orb_subscribe(ORB_ID(parafoil_attitude_sensor));     // 订阅sensor_combined
	orb_set_interval(parafoil_attitude_sensor_sub_fd, 1000);                          // 设置topic(bus)数据读取最小时间间隔.
																	// 即如果没有1s, 那么认为数据还是旧的.
    /* advertise attitude topic */
    struct vehicle_attitude_s att;
    memset(&att, 0, sizeof(att));                                   // 初始化, 清零
    orb_advert_t att_pub_fd = orb_advertise(ORB_ID(vehicle_attitude), &att);

    /* advertise attitude topic */
	struct px4_test_s px4_test;
	memset(&px4_test, 0, sizeof(px4_test));                                   // 初始化, 清零
	orb_advert_t px4_test_pub_fd = orb_advertise(ORB_ID(px4_test), &px4_test);

    /* one could wait for multiple topics with this technique, just using one here */
//    struct pollfd fds[] = {
//        { .fd = parafoil_attitude_sensor_sub_fd,   .events = POLLIN },
//        /* there could be more file descriptors here, in the form like:
//         * { .fd = other_sub_fd,   .events = POLLIN },
//         */
//    };
//    int error_counter = 0;
    thread_running = true;

    /* check parafoil attitude message update. -libn Mar 28, 2017 */
	bool updated = false;
	int parafoil_att_count = 0;
    struct parafoil_attitude_sensor_s parafoil_attitude_sensor_data;
	/* obtained data for the first file descriptor */
	struct sensor_combined_s raw;

	/* poll request. -libn Mar 28, 2017 */
	/* wakeup source */
	px4_pollfd_struct_t fds[1];
	fds[0].fd = parafoil_attitude_sensor_sub_fd;
	fds[0].events = POLLIN;

    while(!thread_should_exit)              // 如果线程没有被停止
    {
        /* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
//        int poll_ret = poll(fds, 1, 100);                          // 线程等待数据更新, timeout为1s,
                                                                    // 如果数据没更新, 那么此时系统会进行任务切换.
//        /* handle the poll result */
//        if (poll_ret == 0) {
//            /* this means none of our providers is giving us data */
//            printf("[px4_simple_app] Got no data within a second\n");
//        } else if (poll_ret < 0) {
//            /* this is seriously bad - should be an emergency */
//            if (error_counter < 10 || error_counter % 50 == 0) {
//                /* use a counter to prevent flooding (and slowing us down) */
//                printf("[px4_simple_app] ERROR return value from poll(): %d\n"
//                    , poll_ret);
//            }
//            error_counter++;
//        } else {
//            if (fds[0].revents & POLLIN) {
//                /* obtained data for the first file descriptor */
//                struct sensor_combined_s raw;
//                /* copy sensors raw data into local buffer */
//                orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
//                // 将更新的数据输出到shell上
////                printf("[px4_simple_app] Accelerometer:\t%8.4f\t%8.4f\t%8.4f\n",
////                    (double)raw.accelerometer_m_s2[0],
////                    (double)raw.accelerometer_m_s2[1],
////                    (double)raw.accelerometer_m_s2[2]);
//
//                struct parafoil_attitude_sensor_s parafoil_attitude_sensor_data;
//				/* copy sensors raw data into local buffer */
//				orb_copy(ORB_ID(parafoil_attitude_sensor), parafoil_attitude_sensor_sub_fd, &parafoil_attitude_sensor_data);
//				printf("parafoil_attitude_sensor_data.parafoil_roll_angle = %8.3f\n",(double)parafoil_attitude_sensor_data.parafoil_roll_angle);
//
//
//                /* set att and publish this information for other apps */
//                att.rollspeed = raw.accelerometer_m_s2[0];
//                att.pitchspeed = raw.accelerometer_m_s2[1];
//                att.yawspeed = raw.accelerometer_m_s2[2];
//                orb_publish(ORB_ID(vehicle_attitude), att_pub_fd, &att);    // 发送vehicle_attitude数据
//
//                /* set att and publish this information for other apps */
//				px4_test.a = 2;
//				px4_test.b = 2;
//				orb_publish(ORB_ID(px4_test), px4_test_pub_fd, &px4_test);    // 发送vehicle_attitude数据
//            }
//            /* there could be more file descriptors here, in the form like:
//             * if (fds[1..n].revents & POLLIN) {}
//             */
//        }

    	/* wait for up to 20ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 20);

		/* timed out - periodic check for _task_should_exit */
		if (pret == 0) {
			// Go through the loop anyway to copy manual input at 50 Hz.
		}

		/* this is undesirable but not much we can do */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		printf("time out!\n");

//            /* copy sensors raw data into local buffer */
//            orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
            // 将更新的数据输出到shell上
//                printf("[px4_simple_app] Accelerometer:\t%8.4f\t%8.4f\t%8.4f\n",
//                    (double)raw.accelerometer_m_s2[0],
//                    (double)raw.accelerometer_m_s2[1],
//                    (double)raw.accelerometer_m_s2[2]);


		/* Check if vehicle control mode has changed */
		orb_check(parafoil_attitude_sensor_sub_fd, &updated);

		if (updated) {
			parafoil_att_count++;
			if(parafoil_att_count > 10) {
				parafoil_att_count = 0;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(parafoil_attitude_sensor), parafoil_attitude_sensor_sub_fd, &parafoil_attitude_sensor_data);
				printf("parafoil_attitude_sensor_data.parafoil_roll_angle = %8.3f\n",(double)parafoil_attitude_sensor_data.parafoil_roll_angle);

				/* set att and publish this information for other apps */
				att.rollspeed = raw.accelerometer_m_s2[0];
				att.pitchspeed = raw.accelerometer_m_s2[1];
				att.yawspeed = raw.accelerometer_m_s2[2];
				orb_publish(ORB_ID(vehicle_attitude), att_pub_fd, &att);    // 发送vehicle_attitude数据

				/* set att and publish this information for other apps */
				px4_test.a = 2;
				px4_test.b = 2;
				orb_publish(ORB_ID(px4_test), px4_test_pub_fd, &px4_test);    // 发送vehicle_attitude数据

			}

		}

        /* there could be more file descriptors here, in the form like:
         * if (fds[1..n].revents & POLLIN) {}
         */

    }
    thread_running = false;
    return 0;
}
