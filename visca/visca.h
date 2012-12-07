#ifndef __VISCA_H__
#define __VISCA_H__

#include <stdint.h>
#include <err.h>

#ifdef DEBUG
#include <stdio.h>

#define ldebug(fmt, ...) do { fprintf(stderr, "%s,%d: "fmt, __FILE__, __LINE__, ##__VA_ARGS__); } while (0)
#define debug(fmt, ...)do { fprintf(stderr, fmt, ##__VA_ARGS__); } while(0)
#else
#define ldebug
#define debug
#endif

#define BUG_ON(cond) do {					\
	if ((cond))						\
		BUG();						\
} while(0) 

#define BUG() do {	    \
		errx(-2, "%s(%d) BUG", __FILE__, __LINE__);	\
	  } while(0)

#ifndef VISCA_IMAGE_FILP
#define VISCA_IMAGE_FILP 0
#endif

#define PACKET_BUF_SIZE		 16
typedef unsigned char byte_t;

#define VISCA_ZOOM_SPEED_MIN	0x00
#define VISCA_ZOOM_SPEED_MAX	0x07
#define VISCA_ZOOM_POS_MIN	0x00
#define VISCA_ZOOM_POS_MAX	0x4000
#define VISCA_PAN_SPEED_MIN	0x01
#define VISCA_PAN_SPEED_MAX	0x18
#define VISCA_TILT_SPEED_MIN    0x01
#define VISCA_TILT_SPEED_MAX	0x17

#define VISCA_PAN_POS_MIN       ((int16_t)0xf725)
#define VISCA_PAN_POS_MAX       ((int16_t)0x08db)

#if !VISCA_IMAGE_FILP
#define VISCA_TILT_POS_MIN	((int16_t)0xfe70)
#define VISCA_TILT_POS_MAX	((int16_t)0x04b0)
#else
#define VISCA_TILT_POS_MIN	((int16_t)0xfb50)
#define VISCA_TILT_POS_MAX	((int16_t)0x0190)
#endif

#define PT_P            0x01
#define PT_N            0x02

#define PT_LEFT	        PT_P
#define PT_RIGHT     	PT_N
#define PT_UP        	PT_P
#define PT_DOWN	        PT_N
#define PT_STOP		0x03

#define PT_DIR(byte0, byte1)    ((byte0) << 8 | (byte1))

#define VISCA_PT_UP    	PT_DIR(PT_STOP, PT_UP)
#define VISCA_PT_DOWN    	PT_DIR(PT_STOP, PT_DOWN)
#define VISCA_PT_LEFT	PT_DIR(PT_LEFT, PT_STOP)
#define VISCA_PT_RIGHT	PT_DIR(PT_RIGHT, PT_STOP)
#define VISCA_PT_UPLEFT	PT_DIR(PT_LEFT, PT_UP)
#define VISCA_PT_UPRIGHT	PT_DIR(PT_RIGHT, PT_UP)
#define VISCA_PT_DOWNLEFT	PT_DIR(PT_LEFT, PT_DOWN)
#define VISCA_PT_DOWNRIGHT	PT_DIR(PT_RIGHT, PT_DOWN)
#define VISCA_PT_STOP	PT_DIR(PT_STOP, PT_STOP)


enum {
	VISCA_CMD_SET_ADDRESS,
	VISCA_CMD_CLEAR_IF,
	VISCA_CMD_ZOOM_STOP,
	VISCA_CMD_ZOOM_TELE,
	VISCA_CMD_ZOOM_WIDE,
	VISCA_CMD_ZOOM_TELE_SPEED,
	VISCA_CMD_ZOOM_WIDE_SPEED,
	VISCA_CMD_ZOOM_DIRECT,
	VISCA_CMD_PANTILT_DIR,
	VISCA_CMD_PANTILT_STOP,
	VISCA_CMD_PANTILT_ABSOLUTE_POS,
	VISCA_CMD_PANTILT_RELATIVE_POS,
	VISCA_CMD_PANTILT_HOME,
	VISCA_CMD_PANTILT_RESET,
		
	VISCA_CMD_MAX
};

enum {
	VISCA_INQ_ZOOM_POS,
	VISCA_INQ_VERSION,
	VISCA_INQ_PANTILT_STATUS,

	VISCA_INQ_MAX
};

struct visca_pantilt_dir {
	int pan_speed;
	int tilt_speed;
	int dir;
};

struct visca_pantilt_pos {
	int pan_speed;
	int tilt_speed;
	int pan_pos;
	int tilt_pos;
};

struct visca_version {
	int vendor;
	int model;
	int rom_version;
};

struct visca_packet {
	byte_t buf[PACKET_BUF_SIZE];
	int len;

	/* for debug*/
	int dir; 
};

void visca_init_pack(struct visca_packet *pack);

int visca_open_serial(char *dev_name);
void visca_close_serial(int fd);

int __visca_command(int fd, struct visca_packet *pack, int cmd_idx, int cam_addr, long para);
#define __visca_command0(fd, pack, cmd_idx, cam_addr) __visca_command(fd, pack, cmd_idx, cam_addr, 0);

int __visca_inquiry(int fd, struct visca_packet *pack, int cmd_idx, int cam_addr, void *ret);

int __visca_pantilt_dir(int fd, struct visca_packet *pack, int cam_addr, int pan_speed, int tilt_speed, int dir) 
{
	struct visca_pantilt_dir para = {
		.pan_speed = pan_speed,
		.tilt_speed = tilt_speed,
		.dir = dir,
	};
	return __visca_command(fd, pack, VISCA_CMD_PANTILT_DIR, cam_addr, (long) &para);
}

int __visca_pantilt_pos(int fd, struct visca_packet *pack, int cmd_idx, int cam_addr, int pan_speed, int tilt_speed, int pan_pos, int tilt_pos) 
{
	struct visca_pantilt_pos para = {
		.pan_speed = pan_speed,
		.tilt_speed = tilt_speed,
		.pan_pos = pan_pos,
		.tilt_pos = tilt_pos
	};
	return __visca_command(fd, pack, cmd_idx, cam_addr, (long) &para);
}

#define DEF_CAM_ADDR  1
#define visca_command(fd, pack, cmd_idx, para)	__visca_command(fd, pack, cmd_idx, DEF_CAM_ADDR, para)
#define visca_command0(fd, pack, cmd_idx)      	__visca_command(fd, pack, cmd_idx, DEF_CAM_ADDR, 0)

#define visca_set_address(fd, pack)  visca_command0(fd, pack, VISCA_CMD_SET_ADDRESS)
#define visca_clear_if(fd, pack)     visca_command0(fd, pack, VISCA_CMD_CLEAR_IF)
#define visca_zoom_stop(fd, pack)    visca_command0(fd, pack, VISCA_CMD_ZOOM_STOP)
#define visca_zoom_tele(fd, pack)    visca_command0(fd, pack, VISCA_CMD_ZOOM_TELE)
#define visca_zoom_wide(fd, pack)    visca_command0(fd, pack, VISCA_CMD_ZOOM_WIDE)
#define visca_zoom_wide_speed(fd, pack, para)  visca_command(fd, pack, VISCA_CMD_ZOOM_WIDE_SPEED, para)
#define visca_zoom_tele_speed(fd, pack, para)  visca_command(fd, pack, VISCA_CMD_ZOOM_TELE_SPEED, para)
#define visca_zoom_direct(fd, pack, para)	visca_command(fd, pack, VISCA_CMD_ZOOM_DIRECT, para)
#define visca_pantilt_home(fd, pack) visca_command0(fd, pack, VISCA_CMD_PANTILT_HOME)
#define visca_pantilt_reset(fd, pack) visca_command0(fd, pack, VISCA_CMD_PANTILT_RESET)
#define visca_pantilt_stop(fd, pack) visca_command0(fd, pack, VISCA_CMD_PANTILT_STOP)

#define visca_pantilt_dir(fd, pack, ...) __visca_pantilt_dir(fd, pack, DEF_CAM_ADDR, ##__VA_ARGS__)
#define visca_pantilt_absolute_pos(fd, pack, ...) __visca_pantilt_pos(fd, pack, VISCA_CMD_PANTILT_ABSOLUTE_POS, DEF_CAM_ADDR, ##__VA_ARGS__)
#define visca_pantilt_relative_pos(fd, pack, ...) __visca_pantilt_pos(fd, pack, VISCA_CMD_PANTILT_RELATIVE_POS, DEF_CAM_ADDR, ##__VA_ARGS__)

#define visca_inquiry(fd, pack, cmd_idx, ret)	__visca_inquiry(fd, pack, cmd_idx, DEF_CAM_ADDR, ret)

int visca_inq_zoom_pos(int fd, struct visca_packet *pack, int *ret)
{
	return visca_inquiry(fd, pack, VISCA_INQ_ZOOM_POS, ret);
}

int visca_inq_version(int fd, struct visca_packet *pack, struct visca_version *ret)
{
	return visca_inquiry(fd, pack, VISCA_INQ_VERSION, ret);
}

#endif







