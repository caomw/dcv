#include <unistd.h>

#include "visca.h"

int fd;
struct visca_interface iface =VISCA_IF_INIT;
struct visca_interface *pif = &iface;

#define SLEEP_INTERVAL 1

char *dev_name = "/dev/ttyS0";

void test_zoom()
{
	visca_zoom_wide(pif);
	sleep(5);
	visca_zoom_tele(pif);
	sleep(3);
	visca_zoom_stop(pif);
	visca_zoom_wide(pif);
	sleep(5);
	
	visca_zoom_tele_speed(pif, 0);
	sleep(5);
	visca_zoom_wide_speed(pif, 7);
	sleep(5);
}

void test_check_arg()
{
	visca_zoom_wide_speed(pif, 20000);
	visca_zoom_direct(pif, 20000);
	visca_pantilt_dir(pif, 100, 23, 1);
	visca_pantilt_absolute_pos(pif, 5, 5, -400, -500);
}

void test_pantilt()
{
	visca_pantilt_dir(pif, 24, 23, VISCA_PT_UP);
	sleep(SLEEP_INTERVAL);
	visca_pantilt_stop(pif);
	
	visca_pantilt_dir(pif, 5, 5, VISCA_PT_DOWN);
	sleep(SLEEP_INTERVAL);
	visca_pantilt_stop(pif);

	visca_pantilt_absolute_pos(pif, 5, 5, -200, -200);
	visca_pantilt_stop(pif);
	sleep(SLEEP_INTERVAL);
	visca_pantilt_relative_pos(pif, 5, 5, 0, -200);
	sleep(SLEEP_INTERVAL);
	visca_pantilt_home(pif);
	sleep(SLEEP_INTERVAL);
	visca_pantilt_relative_pos(pif, 5, 5, 0, 200);
	sleep(SLEEP_INTERVAL);
	visca_pantilt_reset(pif);
	sleep(SLEEP_INTERVAL);
}

void test_inq()
{
	int vendor, model, rom_version;
	int zoom_val;
	visca_zoom_direct(pif, 200);
	sleep(SLEEP_INTERVAL);
	visca_inq_zoom_pos(pif, &zoom_val);
	dbg("zoom inquired is %d\n", zoom_val);
	
	visca_inq_version(pif, &vendor, &model, &rom_version);

	dbg("vendor: 0x%04x\nmodel: 0x%04x\nrom version 0x%04x\n", vendor, model, rom_version);
}

int main(int argc, char **argv)
{
	/* struct visca_camera_info cam_info; */
	/* just example ignore error handling for simplicity*/
	visca_open_serial(pif, dev_name);
	visca_set_address(pif);
	visca_clear_if(pif);

	test_zoom();
	test_check_arg();
	test_pantilt();
	test_inq();

	visca_close_serial(pif);
	return 0;
}
