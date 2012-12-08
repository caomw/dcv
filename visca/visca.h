#ifndef __VISCA_H__
#define __VISCA_H__

#include <stdio.h>
#include <stdint.h>
#include <pthread.h>
#include <stdlib.h>
#include <err.h>

#define ARRAY_SIZE(x) (sizeof((x))/sizeof((x)[0]))

#ifdef DEBUG
/* debug with line position*/
#define dbgl(fmt, ...) do { fprintf(stderr, "%s,%d: "fmt, __FILE__, __LINE__, ##__VA_ARGS__); } while (0)
#define dbg(fmt, ...)do { fprintf(stderr, fmt, ##__VA_ARGS__); } while(0)
#else
#define dbgl
#define dbg
#endif

#define __pr(level, fmt, ...) do { fprintf(stderr, level#fmt, ##__VA_ARGS__); } while(0)
#define pr_warn(fmt, ...) __pr("WARN: ", fmt, ##__VA_ARGS__)
#define pr_err(fmt, ...) __pr("ERROR: ", fmt, ##__VA_ARGS__)

#define likely(x)      __builtin_expect(!!(x), 1)
#define unlikely(x)    __builtin_expect(!!(x), 0)

#define BUG() do {						\
	fprintf(stderr, "BUG failure at %s:%d/%s()!\n",	\
		__FILE__, __LINE__, __func__);			\
	err(-2, " ");						\
} while(0)

#define BUG_ON(cond) do { if(unlikely(cond)) BUG(); } while(0)

#ifndef VISCA_IMAGE_FILP
#define VISCA_IMAGE_FILP 0
#endif

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

#define __PT_P            	0x01
#define __PT_N            	0x02

#define __PT_LEFT	        __PT_P
#define __PT_RIGHT     		__PT_N
#define __PT_UP        		__PT_P
#define __PT_DOWN	        __PT_N
#define __PT_STOP		0x03

#define __PT_DIR(byte0, byte1)    ((byte0) << 8 | (byte1))

#define VISCA_PT_UP    		__PT_DIR(__PT_STOP, __PT_UP)
#define VISCA_PT_DOWN    	__PT_DIR(__PT_STOP, __PT_DOWN)
#define VISCA_PT_LEFT		__PT_DIR(__PT_LEFT, __PT_STOP)
#define VISCA_PT_RIGHT		__PT_DIR(__PT_RIGHT, __PT_STOP)
#define VISCA_PT_UPLEFT		__PT_DIR(__PT_LEFT, __PT_UP)
#define VISCA_PT_UPRIGHT	__PT_DIR(__PT_RIGHT, __PT_UP)
#define VISCA_PT_DOWNLEFT	__PT_DIR(__PT_LEFT, __PT_DOWN)
#define VISCA_PT_DOWNRIGHT	__PT_DIR(__PT_RIGHT, __PT_DOWN)
#define VISCA_PT_STOP		__PT_DIR(__PT_STOP, __PT_STOP)


enum {
	visca_nr_set_address,
	visca_nr_clear_if,
	visca_nr_zoom_stop,
	visca_nr_zoom_tele,
	visca_nr_zoom_wide,
	visca_nr_zoom_tele_speed,
	visca_nr_zoom_wide_speed,
	visca_nr_zoom_direct,
	visca_nr_pantilt_dir,
	visca_nr_pantilt_stop,
	visca_nr_pantilt_absolute_pos,
	visca_nr_pantilt_relative_pos,
	visca_nr_pantilt_home,
	visca_nr_pantilt_reset,
	visca_nr_inq_zoom_pos,
	visca_nr_inq_version,
	visca_nr_inq_pantilt_status,

	visca_nr_max,
};

#define VISCA_IF_BUF_SIZE 16
struct visca_interface {	
	pthread_mutex_t lock; /* protect the whole structure */
	int opened;
	int fd;

	byte_t send_buf[VISCA_IF_BUF_SIZE];
	int send_pos;
	byte_t recv_buf[VISCA_IF_BUF_SIZE];
	int recv_pos;
};

#define VISCA_IF_INIT { \
	.lock = PTHREAD_MUTEX_INITIALIZER, \
	.opened = 0, \
}

int visca_open_serial(struct visca_interface *iface, char *dev_name);
int visca_close_serial(struct visca_interface *iface);

/* Unless you know what you do to use the function directly 
 * It doesnt have argument check
 */
int __visca_command(struct visca_interface *iface, int cmd_idx, int cam_addr, long arg);

#define __CMD_DECL1(t1, a1)		t1 a1
#define __CMD_DECL2(t2, a2, ...)	t2 a2, __CMD_DECL1(__VA_ARGS__)
#define __CMD_DECL3(t3, a3, ...)	t3 a3, __CMD_DECL2(__VA_ARGS__)
#define __CMD_DECL4(t4, a4, ...)	t4 a4, __CMD_DECL3(__VA_ARGS__)
#define __CMD_DECL5(t5, a5, ...)	t5 a5, __CMD_DECL4(__VA_ARGS__)
#define __CMD_DECL6(t6, a6, ...)	t6 a6, __CMD_DECL5(__VA_ARGS__)

#define __CMD_CAST1(t1, a1)		(t1) a1
#define __CMD_CAST2(t2, a2, ...)	(t2) a2, __CMD_CAST1(__VA_ARGS__)
#define __CMD_CAST3(t3, a3, ...)	(t3) a3, __CMD_CAST2(__VA_ARGS__)
#define __CMD_CAST4(t4, a4, ...)	(t4) a4, __CMD_CAST3(__VA_ARGS__)
#define __CMD_CAST5(t5, a5, ...)	(t5) a5, __CMD_CAST4(__VA_ARGS__)
#define __CMD_CAST6(t6, a6, ...)	(t6) a6, __CMD_CAST5(__VA_ARGS__)

#define _VISCA_DECL_CMDx(x, name, ...) \
	int _visca_##name(struct visca_interface *iface, int cam_addr, __CMD_DECL##x(__VA_ARGS__))

#define _VISCA_DECL_CMD1(name, ...)	_VISCA_DECL_CMDx(1, name, __VA_ARGS__)
#define _VISCA_DECL_CMD2(name, ...)	_VISCA_DECL_CMDx(2, name, __VA_ARGS__)
#define _VISCA_DECL_CMD3(name, ...)	_VISCA_DECL_CMDx(3, name, __VA_ARGS__)
#define _VISCA_DECL_CMD4(name, ...)	_VISCA_DECL_CMDx(4, name, __VA_ARGS__)
#define _VISCA_DECL_CMD5(name, ...)	_VISCA_DECL_CMDx(5, name, __VA_ARGS__)
#define _VISCA_DECL_CMD6(name, ...)	_VISCA_DECL_CMDx(6, name, __VA_ARGS__)

#define _VISCA_DECL_INQ1(name, ...)	_VISCA_DECL_CMDx(1, inq_##name, __VA_ARGS__)
#define _VISCA_DECL_INQ2(name, ...)	_VISCA_DECL_CMDx(2, inq_##name, __VA_ARGS__)
#define _VISCA_DECL_INQ3(name, ...)	_VISCA_DECL_CMDx(3, inq_##name, __VA_ARGS__)
#define _VISCA_DECL_INQ4(name, ...)	_VISCA_DECL_CMDx(4, inq_##name, __VA_ARGS__)
#define _VISCA_DECL_INQ5(name, ...)	_VISCA_DECL_CMDx(5, inq_##name, __VA_ARGS__)
#define _VISCA_DECL_INQ6(name, ...)	_VISCA_DECL_CMDx(6, inq_##name, __VA_ARGS__)

#define _VISCA_CMD0(name)	\
	int _visca_##name(struct visca_interface *iface, int cam_addr)	\
	{								\
		return __visca_command(iface, visca_nr_##name, cam_addr, 0);		\
	}	


#define _VISCA_CMD1(name, type1, name1)	\
	_VISCA_DECL_CMD1(name, type1, name1) \
	{								\
		return __visca_command(iface, visca_nr_##name, cam_addr, (long) name1);	\
	}


#define VISCA_DEFAULT_CAM_ADDR 1

#define VISCA_CMD0(name) \
	inline int visca_##name(struct visca_interface *iface) \
	{ \
		return _visca_##name(iface, VISCA_DEFAULT_CAM_ADDR);	\
	}

#define VISCA_CMDx(x, name, ...)					\
	inline int visca_##name(struct visca_interface *iface, __CMD_DECL##x(__VA_ARGS__)) \
	{						\
		return _visca_##name(iface, VISCA_DEFAULT_CAM_ADDR, __CMD_CAST##x(__VA_ARGS__)); \
	}

#define VISCA_CMD1(name, ...) VISCA_CMDx(1, name, __VA_ARGS__)
#define VISCA_CMD2(name, ...) VISCA_CMDx(2, name, __VA_ARGS__)
#define VISCA_CMD3(name, ...) VISCA_CMDx(3, name, __VA_ARGS__)
#define VISCA_CMD4(name, ...) VISCA_CMDx(4, name, __VA_ARGS__)
#define VISCA_CMD5(name, ...) VISCA_CMDx(5, name, __VA_ARGS__)
#define VISCA_CMD6(name, ...) VISCA_CMDx(6, name, __VA_ARGS__)

#define VISCA_INQ1(name, ...) VISCA_CMD1(inq_##name, __VA_ARGS__)
#define VISCA_INQ2(name, ...) VISCA_CMD2(inq_##name, __VA_ARGS__)
#define VISCA_INQ3(name, ...) VISCA_CMD3(inq_##name, __VA_ARGS__)
#define VISCA_INQ4(name, ...) VISCA_CMD4(inq_##name, __VA_ARGS__)
#define VISCA_INQ5(name, ...) VISCA_CMD5(inq_##name, __VA_ARGS__)
#define VISCA_INQ6(name, ...) VISCA_CMD6(inq_##name, __VA_ARGS__)

#define VISCA_DEFINE_CMD0(name)  \
	_VISCA_CMD0(name) \
	VISCA_CMD0(name)

#define VISCA_DEFINE_CMD1(name, ...) \
	_VISCA_CMD1(name, __VA_ARGS__) \
	VISCA_CMD1(name, __VA_ARGS__)

#define VISCA_DEFINE_INQ1(name, ...) VISCA_DEFINE_CMD1(inq_##name, __VA_ARGS__)

VISCA_DEFINE_CMD0(set_address)
VISCA_DEFINE_CMD0(clear_if)
VISCA_DEFINE_CMD0(zoom_stop)
VISCA_DEFINE_CMD0(zoom_tele)
VISCA_DEFINE_CMD0(zoom_wide)
VISCA_DEFINE_CMD0(pantilt_home)
VISCA_DEFINE_CMD0(pantilt_reset)
VISCA_DEFINE_CMD0(pantilt_stop)

VISCA_DEFINE_CMD1(zoom_tele_speed, int, speed)
VISCA_DEFINE_CMD1(zoom_wide_speed, int, speed)
VISCA_DEFINE_CMD1(zoom_direct, int, pos)

struct visca_pantilt_dir {
	int pan_speed;
	int tilt_speed;
	int dir;
};

_VISCA_DECL_CMD3(pantilt_dir, int, pan_speed, int, tilt_speed, int, dir)
{
	struct visca_pantilt_dir arg = {
		.pan_speed = pan_speed,
		.tilt_speed = tilt_speed,
		.dir = dir,
	};
	return __visca_command(iface, visca_nr_pantilt_dir, cam_addr, (long) &arg);
}
VISCA_CMD3(pantilt_dir, int, pan_speed, int, tilt_speed, int, dir)


struct visca_pantilt_pos {
	int pan_speed;
	int tilt_speed;
	int pan_pos;
	int tilt_pos;
};
_VISCA_DECL_CMD4(pantilt_absolute_pos, int, pan_speed, int, tilt_speed, int, pan_pos, int, tilt_pos)
{
	struct visca_pantilt_pos arg = {
		.pan_speed = pan_speed,
		.tilt_speed = tilt_speed,
		.pan_pos = pan_pos,
		.tilt_pos = tilt_pos
	};
	return __visca_command(iface, visca_nr_pantilt_absolute_pos, cam_addr, (long) &arg);
} 
VISCA_CMD4(pantilt_absolute_pos, int, pan_speed, int, tilt_speed, int, pan_pos, int, tilt_pos)

_VISCA_DECL_CMD4(pantilt_relative_pos, int, pan_speed, int, tilt_speed, int, pan_pos, int, tilt_pos)
{
	struct visca_pantilt_pos arg = {
		.pan_speed = pan_speed,
		.tilt_speed = tilt_speed,
		.pan_pos = pan_pos,
		.tilt_pos = tilt_pos
	};
	return __visca_command(iface, visca_nr_pantilt_relative_pos, cam_addr, (long) &arg);
} 
VISCA_CMD4(pantilt_relative_pos, int, pan_speed, int, tilt_speed, int, pan_pos, int, tilt_pos)

VISCA_DEFINE_INQ1(zoom_pos, int*, pos)

struct visca_version {
	int vendor;
	int model;
	int rom_version;
};
_VISCA_DECL_INQ3(version, int*, vendor, int*, model, int*, rom_version)
{
	int err = 0;
	struct visca_version version;
	
	if ((err = __visca_command(iface, visca_nr_inq_version, cam_addr, (long) &version)))
		return err;
	
	*vendor = version.vendor;
	*model = version.model;
	*rom_version = version.rom_version;
	return err;
}
VISCA_INQ3(version, int*, vendor, int*, model, int*, rom_version)

#endif
