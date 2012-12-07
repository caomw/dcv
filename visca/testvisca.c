#include <err.h>
#include <unistd.h>

#include "visca.h"

int fd;
struct visca_packet pack;
struct visca_packet *p = &pack;
#define SLEEP_INTERVAL 1

char *dev_name = "/dev/ttyS0";

void test_zoom() 
{	
	visca_zoom_wide(fd, p);
	sleep(5);
	visca_zoom_tele(fd, p);
	sleep(3);
	visca_zoom_stop(fd, p);
	visca_zoom_wide(fd, p);
	sleep(5);
	
	visca_zoom_tele_speed(fd, p, 0);
	sleep(5);
	visca_zoom_wide_speed(fd, p, 7);
	sleep(5);	
}

void test_check_para()
{
	visca_zoom_wide_speed(fd, p, 20000);
	visca_zoom_direct(fd, p, 20000);
	visca_pantilt_dir(fd, p, 100, 23, 1);
	visca_pantilt_absolute_pos(fd, p, 5, 5, -400, -500);
}

void test_pantilt()
{
	visca_pantilt_dir(fd, p, 24, 23, VISCA_PT_UP);
	sleep(SLEEP_INTERVAL);
	visca_pantilt_stop(fd, p);

	visca_pantilt_dir(fd, p, 5, 5, VISCA_PT_DOWN);
	sleep(SLEEP_INTERVAL);
	visca_pantilt_stop(fd, p);

	visca_pantilt_absolute_pos(fd, p, 5, 5, -200, -200);
	visca_pantilt_stop(fd, p);
	sleep(SLEEP_INTERVAL);
	visca_pantilt_relative_pos(fd, p, 5, 5, 0, -200);
	sleep(SLEEP_INTERVAL);
	visca_pantilt_home(fd, p);
	sleep(SLEEP_INTERVAL);
	visca_pantilt_relative_pos(fd, p, 5, 5, 0, 200);
	sleep(SLEEP_INTERVAL);
	visca_pantilt_reset(fd, p);
	sleep(SLEEP_INTERVAL);
}

void test_inq() 
{
	struct visca_version ver;
	int zoom_val;
	visca_zoom_direct(fd, p, 200);
	sleep(SLEEP_INTERVAL);
	visca_inq_zoom_pos(fd, p, &zoom_val);
	warnx("zoom inquired is %d\n", zoom_val);
	
	visca_inq_version(fd, p, &ver);

	debug("vendor: 0x%04x\nmodel: 0x%04x\nrom version 0x%04x\n", ver.vendor, ver.model, ver.rom_version);
}

int main(int argc, char **argv)
{
/* struct visca_camera_info cam_info; */
/* just example ignore error handling for simplicity*/
	fd = visca_open_serial(dev_name);
	visca_set_address(fd, p);
	visca_clear_if(fd, p);

/* test_zoom(fd, &pack); */
/* test_check_para(fd, &pack); */
	/* test_pantilt(fd, &pack); */
	test_inq();
/* visca_inq_version(fd, cam_addr, &cam_info); */
/* warn("Some camera info:\n-------------------\n"); */

	visca_close_serial(fd);
	return 0;
}
