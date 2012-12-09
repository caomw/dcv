#ifndef __VISCA_H__
#define __VISCA_H__

#include <stdio.h>
#include <stdint.h>
#include <pthread.h>
#include <stdlib.h>
#include <err.h>

#define ARRAY_SIZE(x) (sizeof((x))/sizeof((x)[0]))
#define true		1
#define false		0

#define __pr(fmt, ...) do { \
	fprintf(stderr, fmt, ##__VA_ARGS__); \
} while(0)

#define pr_warn(fmt, ...) __pr("WARN: "fmt, ##__VA_ARGS__)
#define pr_err(fmt, ...) __pr("ERROR: "fmt, ##__VA_ARGS__)

#define pr_warn_str(str) pr_warn("%s", str)
#define pr_err_str(str) pr_err("%s", str)

#ifdef DEBUG
#define dbg  __pr
/* debug with line position*/
#define dbgl(fmt, ...) dbg("DEBUG %s,%d: "fmt, __FILE__, __LINE__, __VA_ARGS__)
#else
#define dbg
#define dbgl
#endif

#define likely(x)      __builtin_expect(!!(x), 1)
#define unlikely(x)    __builtin_expect(!!(x), 0)

#define EXIT_ERR	-1
#define EXIT_BUG	-2
#define EXIT_SYS_BUG	-3

#define BUG() do {							    \
	__pr("BUG failure at %s:%d/%s()!\n", __FILE__, __LINE__, __func__); \
        exit(EXIT_BUG);							\
} while(0)

#define BUG_ON(cond) do { if(unlikely(cond)) BUG(); } while(0)

#define __SYS_BUG(call) do {				\
	err(EXIT_SYS_BUG, "syscall " #call		\
		"BUG failure at %s:%d/%s()!\n",		\
		__FILE__, __LINE__, __func__);	\
} while(0)

#define SYS_BUG(call) do {				\
	if(unlikely((call)))				\
		__SYS_BUG(call);			\
} while(0)

#define __SYS_BUG_RET(ret, call, cond) ({	\
	ret = (call);			\
	if (unlikely(cond)) \
		__SYS_BUG(call); \
	ret; \
})

#define SYS_BUG1(ret, call, errcode1) \
	__SYS_BUG_RET(ret, call, ret && ret != -(errcode1))

#define RW_SYS_BUG(ret, call)			\
	__SYS_BUG_RET(ret, call, ret < 0)

#define RW_SYS_BUG1(ret, call, errcode1)		\
	__SYS_BUG_RET(ret, call, ret < 0 && ret != -(errcode1))

#ifndef VISCA_IMAGE_FILP
#define VISCA_IMAGE_FILP 0
#endif

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
	visca_nr_cmd_set_address,
	visca_nr_cmd_clear_if,
	visca_nr_cmd_zoom_stop,
	visca_nr_cmd_zoom_tele,
	visca_nr_cmd_zoom_wide,
	visca_nr_cmd_zoom_tele_speed,
	visca_nr_cmd_zoom_wide_speed,
	visca_nr_cmd_zoom_direct,
	visca_nr_cmd_pantilt_dir,
	visca_nr_cmd_pantilt_stop,
	visca_nr_cmd_pantilt_absolute_pos,
	visca_nr_cmd_pantilt_relative_pos,
	visca_nr_cmd_pantilt_home,
	visca_nr_cmd_pantilt_reset,
	visca_nr_inq_zoom_pos,
	visca_nr_inq_version,
	visca_nr_inq_pantilt_status,
	visca_nr_inq_pantilt_maxspeed,
	visca_nr_inq_pantilt_pos, 
	visca_nr_max,
};

#define VISCA_IF_BUF_SIZE 16
typedef unsigned char byte_t;

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
int __visca_command(struct visca_interface *iface, int cmd_idx, 
		      int cam_addr, long arg, void *ret);
#define __visca_cmd_command(iface, cmd_idx, cam_addr, arg) \
	__visca_command(iface, cmd_idx, cam_addr, arg, NULL)
#define __visca_inq_command(iface, cmd_idx, cam_addr, ret) \
	__visca_command(iface, cmd_idx, cam_addr, 0, ret)

#define __ARG_DECL1(t1, a1)		t1 a1
#define __ARG_DECL2(t2, a2, ...)	t2 a2, __ARG_DECL1(__VA_ARGS__)
#define __ARG_DECL3(t3, a3, ...)	t3 a3, __ARG_DECL2(__VA_ARGS__)
#define __ARG_DECL4(t4, a4, ...)	t4 a4, __ARG_DECL3(__VA_ARGS__)

#define __ARG_CAST1(t1, a1)		(t1) a1
#define __ARG_CAST2(t2, a2, ...)	(t2) a2, __ARG_CAST1(__VA_ARGS__)
#define __ARG_CAST3(t3, a3, ...)	(t3) a3, __ARG_CAST2(__VA_ARGS__)
#define __ARG_CAST4(t4, a4, ...)	(t4) a4, __ARG_CAST3(__VA_ARGS__)

#define VISCA_DEFAULT_CAM_ADDR		1

#define VISCA_DEFINE_DEFAULT0(name)					\
	inline int visca_##name(struct visca_interface *iface) \
	{ \
		return _visca_##name(iface, VISCA_DEFAULT_CAM_ADDR); \
	}

#define VISCA_DEFINE_DEFAULTx(x, name, ...)			\
	inline int visca_##name(struct visca_interface *iface,  \
				__ARG_DECL##x(__VA_ARGS__))	\
	{						\
		return _visca_##name(iface, VISCA_DEFAULT_CAM_ADDR,  \
				     __ARG_CAST##x(__VA_ARGS__));    \
	}

#define VISCA_DECLx(x, name, ...)				\
	int _visca_##name(struct visca_interface *iface,  \
			  int cam_addr, __ARG_DECL##x(__VA_ARGS__));

#define VISCA_DEFINEx(x, name, ...)	\
	VISCA_DECLx(x, name, __VA_ARGS__)	\
	VISCA_DEFINE_DEFAULTx(x, name, __VA_ARGS__)

#define VISCA_DEFINE_CMD0(name)					\
	int _visca_##name(struct visca_interface *iface, \
			      int cam_addr)				\
	{								\
		return __visca_cmd_command(iface, \
					   visca_nr_cmd_##name,	\
					   cam_addr, 0);	\
	}	\
	VISCA_DEFINE_DEFAULT0(name)

#define VISCA_DEFINE_CMD1(name, type1, arg1) \
	int _visca_##name(struct visca_interface *iface, \
				   int cam_addr, type1 arg1)	  \
	{ \
		return __visca_cmd_command(iface, visca_nr_cmd_##name,	      \
					   cam_addr, (long) arg1);    \
	} \
	VISCA_DEFINE_DEFAULTx(1, name, type1, arg1)

#define VISCA_DEFINE_CMD2(name, ...) VISCA_DEFINEx(2, name, __VA_ARGS__)
#define VISCA_DEFINE_CMD3(name, ...) VISCA_DEFINEx(3, name, __VA_ARGS__)
#define VISCA_DEFINE_CMD4(name, ...) VISCA_DEFINEx(4, name, __VA_ARGS__)

#define VISCA_DEFINE_INQ1(name, type1, arg1)				\
	int _visca_inq_##name(struct visca_interface *iface, \
				      int cam_addr, type1 ret)	\
	{ \
		return __visca_inq_command(iface, visca_nr_inq_##name, \
					   cam_addr, ret); \
	} \
	VISCA_DEFINE_DEFAULTx(1, inq_##name, type1, arg1)

#define VISCA_DEFINE_INQ2(name, ...) VISCA_DEFINEx(2, inq_##name, __VA_ARGS__)
#define VISCA_DEFINE_INQ3(name, ...) VISCA_DEFINEx(3, inq_##name, __VA_ARGS__)
#define VISCA_DEFINE_INQ4(name, ...) VISCA_DEFINEx(4, inq_##name, __VA_ARGS__)

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
VISCA_DEFINE_CMD3(pantilt_dir, int, pan_speed, int, tilt_speed, int, dir)
VISCA_DEFINE_CMD4(pantilt_absolute_pos, int, pan_speed, 
		  int, tilt_speed, int, pan_pos, int, tilt_pos)
VISCA_DEFINE_CMD4(pantilt_relative_pos, int, pan_speed, int, tilt_speed, 
		  int, pan_pos, int, tilt_pos)

VISCA_DEFINE_INQ1(zoom_pos, int*, pos)
VISCA_DEFINE_INQ3(version, int*, vendor, int*, model, int*, rom_version)
VISCA_DEFINE_INQ1(pantilt_status, int*, status)
VISCA_DEFINE_INQ2(pantilt_maxspeed, int*, pan_speed, int*, tilt_speed)
VISCA_DEFINE_INQ2(pantilt_pos, int*, pan_pos, int*, tilt_pos)
#endif
