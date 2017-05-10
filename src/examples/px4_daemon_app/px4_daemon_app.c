/********************头文件区****************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>//strcmp strcasecmp
#include <unistd.h>
#include <px4_config.h>
#include <px4_tasks.h>   //产生deamon
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include<poll.h>   //poll函数所在头文件
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include<uORB/topics/extern_sensor.h>

/*******************全局变量声明区*******************************/
static bool thread_should_exit = false;
static bool thread_running = false;

/***********************功能说明*****************************************/
//通过订阅parafoil_attitude这个topic,获得俯仰,偏航以及翻滚角度

/************************说明结束****************************************/

/************************函数声明区********************************/
int daemon(int argc, char *argv[]);  //deamon函数
__EXPORT int px4_daemon_app_main(int argc, char *argv[]);//主函数

/*******************实现模块********************************/
int px4_daemon_app_main(int argc, char *argv[])
{


	if (argc < 2) {
		PX4_INFO("missing command");
		return 1;
	}

	if (!strcasecmp(argv[1], "start")) {

		if (thread_running) {
			PX4_INFO("daemon is already running\n");
			return 0;
		}

		thread_should_exit = false;
        px4_task_spawn_cmd("daemon 1", SCHED_DEFAULT,SCHED_PRIORITY_DEFAULT,2000, daemon,(char *const *)NULL);
		return 0;
	}

	if (!strcasecmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcasecmp(argv[1], "check")) {
		if (thread_running) {
			warnx("daemon is running \n");

		} else {
			warnx("daemon doesn't exit \n");
		}

		return 0;
	}
	PX4_INFO("unrecognized command");
	return 0;
}

int daemon(int argc, char *argv[])
{

	thread_running = true;


	int para_handle=orb_subscribe(ORB_ID(extern_sensor));
    orb_set_interval(para_handle, 200);

    bool updated;

	while (!thread_should_exit)
	{

		updated=false;
	    orb_check(para_handle, &updated);
		if(updated)
	   {
        struct extern_sensor_s extern_sensor_data;
	    orb_copy(ORB_ID(extern_sensor), para_handle, &extern_sensor_data);
		PX4_INFO("extern_sensor_data:%8.4f %8.4f %8.4f",(double)extern_sensor_data.parafoil_roll_angle,
										 (double)extern_sensor_data.parafoil_pitch_angle,
										 (double)extern_sensor_data.parafoil_yaw_angle);

	   }
		 else
		    PX4_INFO("No extern_sensor update!");

		if(thread_should_exit)
					break;

        sleep(1);//隔1s订阅一次topic数据
	}

	warnx("[daemon] exiting.\n");

	thread_running = false;

	return 0;
}

