#ifndef __VISCA_H__
#define __VISCA_H__

#include <stdio.h>
#include <stdint.h>
#include <pthread.h>
#include <stdlib.h>
#include <err.h>

#include "common.h"

#define VISCA_ERR -1
#define mutex_lock(lock) LCALL(pthread_mutex_lock(lock))
#define mutex_unlock(lock) LCALL(pthread_mutex_unlock(lock))

#ifndef VISCA_IMAGE_FILP
#define VISCA_IMAGE_FILP 0
#endif

#define VISCA_Z_SPEED_MIN	0x00
#define VISCA_Z_SPEED_MAX	0x07
#define VISCA_Z_POS_MIN		0x00
#define VISCA_Z_POS_MAX		0x4000
#define VISCA_P_SPEED_MIN	0x01
#define VISCA_P_SPEED_MAX	0x18
#define VISCA_T_SPEED_MIN    	0x01
#define VISCA_T_SPEED_MAX	0x17

#define VISCA_P_POS_MIN       ((int16_t)0xf725)
#define VISCA_P_POS_MAX       ((int16_t)0x08db)

#if !VISCA_IMAGE_FILP
#define VISCA_T_POS_MIN	((int16_t)0xfe70)
#define VISCA_T_POS_MAX	((int16_t)0x04b0)
#else
#define VISCA_T_POS_MIN	((int16_t)0xfb50)
#define VISCA_T_POS_MAX	((int16_t)0x0190)
#endif

#define __P_LEFT	        0x01
#define __P_RIGHT     		0x02
#define __T_UP        		0x01
#define __T_DOWN	        0x02
#define __PT_STOP		0x03

#define P_DIR_SHIFT		8

#define PT_UP    		(__PT_STOP << P_DIR_SHIFT) | __T_UP)
#define PT_DOWN        		(__PT_STOP << P_DIR_SHIFT) | __T_DOWN)
#define PT_LEFT		        (__P_LEFT << P_DIR_SHIFT) | __PT_STOP)
#define PT_RIGHT		(__P_RIGHT << P_DIR_SHIFT | __PT_STOP)
#define PT_UPLEFT		(__P_LEFT << P_DIR_SHIFT | __T_UP)
#define PT_UPRIGHT		(__P_RIGHT << P_DIR_SHIFT | __T_UP)
#define PT_DOWNLEFT		(__P_LEFT << P_DIR_SHIFT | __T_DOWN)
#define PT_DOWNRIGHT		(__P_RIGHT << P_DIR_SHIFT | __T_DOWN)
#define PT_STOP	       		(__PT_STOP << P_DIR_SHIFT | __PT_STOP)

#define STAT_P_DIR_MASK		0x8083
#define STAT_P_DIR_SHIFT	0
#define STAT_P_DIR_LEFT		(__P_LEFT << STAT_P_DIR_SHIFT)
#define STAT_P_DIR_RIGHT	(__P_RIGHT << STAT_P_DIR_SHIFT)

#define STAT_T_DIR_MASK		0x808c
#define STAT_T_DIR_SHIFT	2
#define STAT_T_DIR_UP       	(__T_UP << STAT_T_DIR_SHIFT)
#define STAT_T_DIR_DOWN		(__T_DOWN << STAT_T_DIR_SHIFT)

#define __PT_ERR		0x00
#define __PT_POS_ERR		0x01
#define __PT_MEC_ERR		0x02

#define STAT_P_ERR_MASK		0x8030
#define STAT_P_ERR_SHIFT	4
#define STAT_P_ERR		(__PT_ERR << STAT_P_ERR_SHIFT)
#define STAT_P_POS_ERR		(__PT_POS_ERR << STAT_P_ERR_SHIFT)
#define STAT_P_MEC_ERR		(__PT_MEC_ERR << STAT_P_ERR_SHIFT)

#define STAT_T_ERR_MASK		0x8380
#define STAT_T_ERR_SHIFT	8
#define STAT_T_ERR		(__PT_ERR << STAT_T_ERR_SHIFT)
#define STAT_T_POS_ERR		(__PT_POS_ERR << STAT_T_ERR_SHIFT)
#define STAT_T_MEC_ERR		(__PT_MEC_ERR << STAT_T_ERR_SHIFT)

#define STAT_PT_ERR_MASK	(STAT_P_ERR_MASK | STAT_T_ERR_MASK)

#define STAT_PT_CMD_MASK       	0x8c80
#define STAT_PT_CMD_SHIFT	10
#define STAT_PT_CMD_NONE	(0x00 << STAT_PT_CMD_SHIFT)
#define STAT_PT_CMD_MIDST	(0x01 << STAT_PT_CMD_SHIFT)
#define STAT_PT_CMD_COMPLETED	(0x02 << STAT_PT_CMD_SHIFT)
#define STAT_PT_CMD_FAILED	(0x03 << STAT_PT_CMD_SHIFT)

#define STAT_PT_INIT_MASK	0xb080
#define STAT_PT_INIT_SHIFT      12
#define STAT_PT_INIT_NONE	(0x00 << STAT_PT_INIT_SHIFT)
#define STAT_PT_INITING		(0x01 << STAT_PT_INIT_SHIFT)
#define STAT_PT_INITED		(0x02 << STAT_PT_INIT_SHIFT)
#define STAT_PT_INIT_FAILED	(0x03 << STAT_PT_INIT_SHIFT)

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
	visca_nr_cmd_dzoom_mode,
	visca_nr_inq_zoom_pos,
	visca_nr_inq_version,
	visca_nr_inq_pantilt_status,
	visca_nr_inq_pantilt_maxspeed,
	visca_nr_inq_pantilt_pos, 
	visca_nr_inq_dzoom_mode,
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
	.opened = 0,	   \
}
struct visca_interface *visca_alloc_init_if();
void visca_free_if(struct visca_interface **iface);
int visca_open_if(struct visca_interface *iface, char *dev_name);
int visca_if_opened(struct visca_interface *iface)
{
	return iface->opened;
}
int visca_close_if(struct visca_interface *iface);


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
VISCA_DEFINE_CMD1(dzoom_mode, bool, flag)

VISCA_DEFINE_INQ1(zoom_pos, int*, pos)
VISCA_DEFINE_INQ3(version, int*, vendor, int*, model, int*, rom_version)
VISCA_DEFINE_INQ1(pantilt_status, int*, status)
VISCA_DEFINE_INQ2(pantilt_maxspeed, int*, pan_speed, int*, tilt_speed)
VISCA_DEFINE_INQ2(pantilt_pos, int*, pan_pos, int*, tilt_pos)
VISCA_DEFINE_INQ1(dzoom_mode, bool*, flag)

#endif
