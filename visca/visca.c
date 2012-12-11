#include <unistd.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <arpa/inet.h>

#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/file.h>

#include "visca.h"

#define PLACE_HOLDER	0x00
#define PH2   	PLACE_HOLDER, PLACE_HOLDER
#define PH3   	PH2, PLACE_HOLDER
#define PH4	PH3, PLACE_HOLDER
#define PH5	PH4, PLACE_HOLDER
#define PH6	PH5, PLACE_HOLDER
#define PH7	PH6, PLACE_HOLDER
#define PH8	PH7, PLACE_HOLDER
#define PH9	PH8, PLACE_HOLDER
#define PH10	PH9, PLACE_HOLDER

static inline uint16_t ntoh2(const byte_t *buf) 
{
	return ntohs(*(uint16_t *)buf);
}
static inline void hton2(byte_t *buf, uint16_t x)
{
	*(uint16_t *)buf = htons(x);
}
static inline uint32_t ntoh4l(const byte_t *buf) 
{
	BUG_ON((buf[0] & 0xf0) || (buf[1] & 0xf0) 
	       || (buf[2] & 0xf0) || (buf[3] & 0xf0));
	
	return buf[0] << 12 | buf[1] << 8 | buf[2] << 4 | buf[3];
}

static inline void hton4l(byte_t *buf, uint32_t x) 
{	
	buf[0] = (x & 0xf000) >> 12;
	buf[1] = (x & 0x0f00) >> 8;
	buf[2] = (x & 0x00f0) >> 4;
	buf[3] = (x & 0x000f);
}

static inline bool check_limit(char *name, int x, int x_min, int x_max) 
{
	if (x < x_min || x > x_max) {
		pr_warn("%s %d out of band (%d, %d)\n", 
			name, x, x_min, x_max);
		return false;
	}
	return true;
}

/* begin commands*/
/* zoom_tele_speed, zoom_wide_speed */
static bool check_z_speed(long arg)
{	
	return check_limit("zoom_speed", (int) arg, 
		    VISCA_Z_SPEED_MIN, VISCA_Z_SPEED_MAX);
}
static void fill_z_speed(byte_t *buf, long arg) 
{
	buf[0] |= (byte_t) arg;
}

/* inq_version */
struct version {
	int vendor;
	int model;
	int rom_version;
};
static void parse_version(const byte_t *buf, void *ret)
{
	struct version *version = (struct version *)ret;
	
	version->vendor = ntoh2(buf);
	version->model = ntoh2(buf + 2);
	version->rom_version = ntoh2(buf + 4);
}
int _visca_inq_version(struct visca_interface *iface, int cam_addr,
		       int *vendor, int *model, int *rom_version)
{
	int error;
	struct version version;
	
	if ((error = __visca_inq_command(iface, visca_nr_inq_version, 
				       cam_addr, &version)))
		return error;
	
	*vendor = version.vendor;
	*model = version.model;
	*rom_version = version.rom_version;
	return 0;
}

/* pantilt_status */
static void parse_pt_status(const byte_t *buf, void *ret)
{
	*(int *)ret = ntoh2(buf);
}

/* zoom_pos, zoom_direct */
static bool check_z_pos(long arg)
{
	return check_limit("zoom_pos", arg, VISCA_Z_POS_MIN, 
			   VISCA_Z_POS_MAX);
}
static void fill_z_pos(byte_t *buf, long arg) 
{
	hton4l(buf, (int) arg);
}
static void parse_z_pos(const byte_t *buf, void *ret)
{
	*(int*)ret = ntoh4l(buf);
}

/* inq_pantilt_maxspeed */
struct pt_speed {
	int p;
        int t;
};
#define PT_SPEED_INIT(p_speed, t_speed) {	\
		.p = p_speed,			\
		.t = t_speed,		\
}
#define P_SPEED_SHIFT	8
#define SPEED_MASK	0xff

static void speedton2(byte_t *buf, struct pt_speed *speed)
{
	hton2(buf, speed->p << P_SPEED_SHIFT | speed->t);
}
static inline bool __check_pt_speed(const struct pt_speed *speed) 
{
	return check_limit("pan_speed", speed->p, 
			   VISCA_P_SPEED_MIN, VISCA_P_SPEED_MAX)
		&& check_limit("tilt_speed", speed->t, 
			       VISCA_T_SPEED_MIN, VISCA_T_SPEED_MAX);
}
static void parse_pt_speed(const byte_t *buf, void *ret)
{
	struct pt_speed *speed = (struct pt_speed *) ret;
	int x;

	x = ntoh2(buf);
	speed->p = x >> P_SPEED_SHIFT & SPEED_MASK;
	speed->t = x & SPEED_MASK;
}
int _visca_inq_pantilt_maxspeed(struct visca_interface *iface, int cam_addr,
				int *pan_speed, int *tilt_speed)
{
	int error;
	struct  pt_speed speed;
	
	if ((error = __visca_inq_command(iface, 
					 visca_nr_inq_pantilt_maxspeed, 
					 cam_addr, &speed)))
		return error;
	
	*pan_speed = speed.p;
	*tilt_speed = speed.t;
	return 0;
}

/* pantilt_dir */
#define P_DIR_SHIFT	8
#define DIR_MASK	0xff

static inline bool __check_pt_dir(int dir)
{
	int p_dir = (dir >> P_DIR_SHIFT) & DIR_MASK;
	int t_dir = dir & DIR_MASK;
	
	if (!(p_dir == __P_LEFT || p_dir == __P_RIGHT 
	      || p_dir == __PT_STOP)) {
		pr_warn("p_dir %d invalid\n", p_dir);
		return false;
	}

	if (!(t_dir == __T_UP || t_dir == __T_DOWN
	      || t_dir == __PT_STOP)) {
		pr_warn("t_dir %d invalid\n", t_dir);
		return false;
	}
	
	if (p_dir == __PT_STOP && t_dir == __PT_STOP) {
		pr_warn("pan_dir tilt_dir must not be 0x03 0x03\n"
			"use command pantilt_stop instead\n");
		return false;
	}
	return true;
}

struct pt_speed_dir {
	struct pt_speed speed;
	int dir;
};
#define PT_SPEED_DIR_INIT(p_speed, t_speed, dir) {		\
		.speed = PT_SPEED_INIT(p_speed, t_speed),	\
		.dir = dir,				\
}

static bool check_pt_speed_dir(long arg)
{
	struct pt_speed_dir *speed_dir = (struct pt_speed_dir *) arg;

	return __check_pt_speed(&speed_dir->speed)
		&& __check_pt_dir(speed_dir->dir);
}
static void fill_pt_speed_dir(byte_t *buf, long arg) 
{
	struct pt_speed_dir *speed_dir = (struct pt_speed_dir *) arg;

	speedton2(buf, &speed_dir->speed);
	hton2(buf + 2, speed_dir->dir);
}
int _visca_pantilt_dir(struct visca_interface *iface, int cam_addr, 
		       int pan_speed, int tilt_speed, int dir)
{
	struct pt_speed_dir speed_dir = PT_SPEED_DIR_INIT(pan_speed,
							  tilt_speed, dir);
	return __visca_cmd_command(iface, visca_nr_cmd_pantilt_dir, 
				   cam_addr, (long) &speed_dir);
}

/* inq_pantilt_pos */
struct pt_pos {
	int p;
	int t;
};
#define PT_POS_INIT(p_pos, t_pos) {		\
		.p = p_pos,			\
			.t = t_pos,		\
			}
static void parse_pt_pos(const byte_t *buf, void *ret)
{
	struct pt_pos *pt_pos = (struct pt_pos *) ret;

	pt_pos->p = ntoh4l(buf);
	pt_pos->t = ntoh4l(buf + 4);
}
int _visca_inq_pantilt_pos(struct visca_interface *iface, int cam_addr,
			   int *pan_pos, int *tilt_pos)
{
	int error;
	struct  pt_pos pt_pos;
	
	if ((error = __visca_inq_command(iface, visca_nr_inq_pantilt_pos, 
				       cam_addr, &pt_pos)))
		return error;
	
	*pan_pos = pt_pos.p;
	*tilt_pos = pt_pos.t;
	return 0;
}

/* pantilt_{absolute,relative}_pos */
struct pt_speed_pos {
	struct pt_speed speed;
	struct pt_pos pos;
};
#define PT_SPEED_POS_INIT(p_speed, t_speed, p_pos, t_pos) {	\
		.speed = PT_SPEED_INIT(p_speed, t_speed),	\
		.pos = PT_POS_INIT(p_pos, t_pos),	\
}
static inline bool __check_pt_pos(struct pt_pos *pos) 
{
	return check_limit("pan_pos", pos->p, 
			   VISCA_P_POS_MIN, VISCA_P_POS_MAX)
		&& check_limit("tilt_pos", pos->t, 
			       VISCA_T_POS_MIN, VISCA_T_POS_MAX);
}
static bool check_pt_speed_pos(long arg)
{
	struct pt_speed_pos *speed_pos = (struct pt_speed_pos *) arg;
	
	return __check_pt_speed(&speed_pos->speed)
		&& __check_pt_pos(&speed_pos->pos);
}
static void fill_pt_speed_pos(byte_t *buf, long arg) 
{
	struct pt_speed_pos *speed_pos = (struct pt_speed_pos *) arg;

	speedton2(buf, &speed_pos->speed);
	hton4l(buf + 2, speed_pos->pos.p);
	hton4l(buf + 6, speed_pos->pos.t);
}
int _visca_pantilt_absolute_pos(struct visca_interface *iface, int cam_addr,
				int pan_speed, int tilt_speed, 
				int pan_pos, int tilt_pos)
{
	struct pt_speed_pos speed_pos = PT_SPEED_POS_INIT(pan_speed, 
							  tilt_speed,
							  pan_pos, tilt_pos);
	return __visca_cmd_command(iface, visca_nr_cmd_pantilt_absolute_pos, 
				   cam_addr, (long) &speed_pos);
} 
int _visca_pantilt_relative_pos(struct visca_interface *iface, int cam_addr, 
				int pan_speed, int tilt_speed, 
				int pan_pos, int tilt_pos)
{
	struct pt_speed_pos speed_pos = PT_SPEED_POS_INIT(pan_speed, 
							  tilt_speed,
							  pan_pos, tilt_pos);
	return __visca_cmd_command(iface, visca_nr_cmd_pantilt_relative_pos, 
				   cam_addr, (long) &speed_pos);
} 

/* {inq_}dzoom_mode */
#define DZOOM_MODE_ON		0x02
#define DZOOM_MODE_OFF		0x03
#define check_dzoom_mode(x)  ((x) == DZOOM_MODE_ON || (x) == DZOOM_MODE_OFF)
static void parse_dzoom_mode(const byte_t *buf, void *ret) 
{
	bool mode;

	mode = buf[0];
	BUG_ON(!check_dzoom_mode(mode));
	*(bool*)ret = mode == DZOOM_MODE_ON;
}
static void fill_dzoom_mode(byte_t *buf, long arg) {
	bool mode = (bool) arg;

	if (mode)
		buf[0] = DZOOM_MODE_ON;
	else
		buf[0] = DZOOM_MODE_OFF;
}
/* the end */

#define BYTE_BROADCAST	0x88
#define BYTE_DIRECT     0x80

#define BYTE_COMMAND	0x01
#define BYTE_INQUIRY    0x09

#define BYTE_INTERFACE	0x00
#define BYTE_CAMERA1	0x04
#define BYTE_PANTILTER	0x06

#define HEAD_BROADCAST(x, ...)	BYTE_BROADCAST, x, ##__VA_ARGS__
#define HEAD_DIRECT(x, ...)     BYTE_DIRECT, x, ##__VA_ARGS__

#define COMMAND(x, ...)       	HEAD_DIRECT(BYTE_COMMAND, x, ##__VA_ARGS__)
#define INQUIRY(x, ...)         HEAD_DIRECT(BYTE_INQUIRY, x, ##__VA_ARGS__)

#define CMD_INTERFACE(x, ...) 	COMMAND(BYTE_INTERFACE, x, ##__VA_ARGS__)
#define INQ_INTERFACE(x, ...)	INQUIRY(BYTE_INTERFACE, x, ##__VA_ARGS__)

#define CMD_CAMERA1(x, ...)	COMMAND(BYTE_CAMERA1, x, ##__VA_ARGS__)
#define INQ_CAMERA1(x, ...) 	INQUIRY(BYTE_CAMERA1, x, ##__VA_ARGS__)

#define CMD_PANTILTER(x, ...)   COMMAND(BYTE_PANTILTER, x, ##__VA_ARGS__)
#define INQ_PANTILTER(x, ...)	INQUIRY(BYTE_PANTILTER, x, ##__VA_ARGS__)

#define CMD_ZOOM(x, ...)	CMD_CAMERA1(0x07, x, ##__VA_ARGS__)

#define COMMAND_DATA_SIZE	16
struct command {
	bool (*check_arg)(long arg);
	int arg_start;
	void (*fill_packet)(byte_t *buf, long arg);
	/* for inquire */
	int ret_start;
	void (*parse_packet)(const byte_t *buf, void *ret);

	int count;
	/* must be constant, or init by macro will fail */
	byte_t data[COMMAND_DATA_SIZE];
};

/* cant make as 
 *	... _parse_packet, d0, ...)		\
 *	...
 *	.data = {d0, __VA_ARGS},
 * it wont work
 */      
#define __COMMAND_INIT(_check_arg, _arg_start, _fill_packet,	\
		       _ret_start, _parse_packet, ...)	\
{							       \
	.check_arg = (_check_arg),			       \
	.arg_start = (_arg_start),			\
	.fill_packet = (_fill_packet),				\
	.ret_start = (_ret_start),				\
	.parse_packet = (_parse_packet),		       \
	.count = ARG_NR(byte_t, __VA_ARGS__), \
	.data = {__VA_ARGS__},     \
}

#define COMMAND_INIT(check_arg, arg_start, fill_packet, ...)  \
	__COMMAND_INIT(check_arg, arg_start, fill_packet, \
		       -1, NULL, __VA_ARGS__)
#define INQUIRY_INIT(ret_start, parse_packet, ...)	\
	__COMMAND_INIT(NULL, -1, NULL, ret_start,	\
		       parse_packet, __VA_ARGS__)
#define COMMAND_INIT0(...) COMMAND_INIT(NULL, -1, NULL, __VA_ARGS__)

#define CMD_ENTRY0(name, ...)				\
	[visca_nr_cmd_##name] = COMMAND_INIT0(__VA_ARGS__)
#define CMD_ENTRY(name, check_arg, arg_start, fill_packet, ...) \
	[visca_nr_cmd_##name] = COMMAND_INIT(check_arg, arg_start, \
					     fill_packet, __VA_ARGS__)
#define INQ_ENTRY(name, ret_start, parse_packet, ...)	\
	[visca_nr_inq_##name] = INQUIRY_INIT(ret_start, parse_packet, \
					     __VA_ARGS__)

static struct command cmds[] = {	
	/* commands without inquiry */
	/* 	no arguments */
	CMD_ENTRY0(set_address, HEAD_BROADCAST(0x30, 0x01)),
	CMD_ENTRY0(clear_if, HEAD_BROADCAST(BYTE_COMMAND, BYTE_INTERFACE,
					    0x01)),
	CMD_ENTRY0(zoom_stop, CMD_ZOOM(0x00)),
	CMD_ENTRY0(zoom_tele, CMD_ZOOM(0x02)),
	CMD_ENTRY0(zoom_wide, CMD_ZOOM(0x03)),
	CMD_ENTRY0(pantilt_stop,
		   CMD_PANTILTER(0x01, 0x00, 0x00, __PT_STOP, __PT_STOP)),
	CMD_ENTRY0(pantilt_home, CMD_PANTILTER(0x04)),
	CMD_ENTRY0(pantilt_reset, CMD_PANTILTER(0x05))
	/*	have some arguments */,
	CMD_ENTRY(zoom_tele_speed, check_z_speed,
		  4, fill_z_speed, CMD_ZOOM(0x20)),
	CMD_ENTRY(zoom_wide_speed, check_z_speed, 
		  4, fill_z_speed, CMD_ZOOM(0x30)),

	/* inquiries without command */
	INQ_ENTRY(version, 2, parse_version, INQ_INTERFACE(0x02)),
	INQ_ENTRY(pantilt_status, 2, parse_pt_status, INQ_PANTILTER(0x10)),
	
	/* /\* general *\/ */
	INQ_ENTRY(zoom_pos, 2, parse_z_pos, INQ_CAMERA1(0x47)),
	CMD_ENTRY(zoom_direct, check_z_pos, 4, fill_z_pos,
		  CMD_CAMERA1(0x47, 0x00, 0x00, 0x00, 0x00)),

	INQ_ENTRY(pantilt_maxspeed, 2, parse_pt_speed, INQ_PANTILTER(0x11)),
	CMD_ENTRY(pantilt_dir, check_pt_speed_dir, 4, fill_pt_speed_dir,
		  CMD_PANTILTER(0x01, PH4)),

	INQ_ENTRY(pantilt_pos, 2, parse_pt_pos, INQ_PANTILTER(0x12)),
	CMD_ENTRY(pantilt_absolute_pos, check_pt_speed_pos, 
		  4, fill_pt_speed_pos, CMD_PANTILTER(0x02, PH10)),
	CMD_ENTRY(pantilt_relative_pos, check_pt_speed_pos,
		  4, fill_pt_speed_pos, CMD_PANTILTER(0x03, PH10)),

	INQ_ENTRY(dzoom_mode, 2, parse_dzoom_mode, INQ_CAMERA1(0x06)),
	CMD_ENTRY(dzoom_mode, NULL, 4, fill_dzoom_mode,
		  CMD_CAMERA1(0x06, PLACE_HOLDER)),
};

#define PACKET_IS_BROADCAST(buf) ((buf)[0] == BYTE_BROADCAST)
#define PACKET_TYPE(buf) ((buf)[1] & 0xf0 ? (buf)[1] & 0xf0 : (buf)[1])
#define ERROR_CODE(buf) ((buf)[2])

#define DIR_INIT 0
#define DIR_SEND 1
#define DIR_RECV 2

#define TERMINATOR	0xFF

#ifdef DEBUG
static inline void dbg_pack(const byte_t *buf, int dir) 
{	
	int i;

	BUG_ON(dir != DIR_SEND && dir != DIR_RECV);
	if (dir == DIR_SEND)
		dbg("S");
	else
		dbg("R");
	for (i = 0; i < VISCA_IF_BUF_SIZE && buf[i] != TERMINATOR; i++)
		dbg(" %02x", buf[i]);
	dbg("\n");	
}
#else
static inline void dbg_pack(const byte_t *buf, int dir) {}
#endif

struct error_msg {
	byte_t code;
	char *desc;
};

#define ERROR_MAX 6
#define ERROR_CMD_CANCELLED  0x04

static const struct error_msg error_tab[] = {
	{.code = 0x01, .desc = "message length error"},
	{.code = 0x02, .desc = "syntax error"},
	{.code = 0x03, .desc = "command buffer full"},
	{.code = ERROR_CMD_CANCELLED, .desc = "command cancelled"},
	{.code = 0x05, .desc = "no socket"},
	{.code = 0x41, .desc = "command not executable"},
		};

#define SERIAL_DELAY  20000 /*us*/
#define RECV_POLL_WAIT  1000 /*us*/

#define BYTE_ERROR		0x60

static int recv_packet(int fd, byte_t *buf, int *count)
{
	int i;
	int pos = 0;
	int bytes_read;
	
	LCALL(usleep(SERIAL_DELAY));

	for(;;) {
		RWCALL1(bytes_read, read(fd, buf + pos, 1), EPERM);
		if (bytes_read > 0) {
			if (buf[pos] == TERMINATOR)
				break;
			pos += bytes_read;				
			if (pos > VISCA_IF_BUF_SIZE) {
				pr_warn("receive pos(%d) > buf_size(%d)\n", 
					pos, VISCA_IF_BUF_SIZE);
				return VISCA_ERR;
			}
		}
		LCALL(usleep(RECV_POLL_WAIT));
	}
	*count = pos + 1;

	dbg_pack(buf, DIR_RECV);

	if (PACKET_TYPE(buf) == BYTE_ERROR) {
		int error_code = ERROR_CODE(buf);

		for (i = 0; i < ERROR_MAX; i++) {
			if (error_tab[i].code == error_code) {
				pr_warn_str(error_tab[i].desc);
				break;
			}
		}

		if (error_code != ERROR_CMD_CANCELLED)
			return VISCA_ERR;
	}
	return 0;
}

static int send_packet(int fd, const byte_t *buf, int count) {
	int ret;
	
	RWCALL(ret, write(fd, buf, count));
	if (ret < count) {
		pr_warn("write packet to fd %d failed\n", fd);
		return VISCA_ERR;		
	}

	LCALL(usleep(SERIAL_DELAY));
	return 0;
}

#define BYTE_ACK		0x40
#define BYTE_COMPLETION         0x50

static int send_with_reply(int fd, const byte_t *send_buf, int send_count, 
			   byte_t *recv_buf, int *recv_count) 
{
	int error;
	byte_t type;

	if ((error = send_packet(fd, send_buf, send_count)))
		return error;

	dbg_pack(send_buf, DIR_SEND);

	if (PACKET_IS_BROADCAST(send_buf)) {
		if ((error = recv_packet(fd, recv_buf, recv_count)))
			return error;
		if (!PACKET_IS_BROADCAST(recv_buf)) {
			pr_warn("'broadcast 0x88' expect 'broadcast 0x88'"
				" reply\n");
			return VISCA_ERR;
		}
		return 0;
	}

	type = PACKET_TYPE(send_buf);
	switch (type) {
	case BYTE_COMMAND:
		do {
			if ((error = recv_packet(fd, recv_buf, recv_count)))
				return error;
		} while(PACKET_TYPE(recv_buf) == BYTE_ACK);
		
		if (PACKET_TYPE(recv_buf) != BYTE_COMPLETION) {
			pr_warn("'command' expect 'completion' reply\n");
			return VISCA_ERR;
		}
		break;
	case BYTE_INQUIRY:
		if ((error = recv_packet(fd, recv_buf, recv_count)))
			return error;
		if (PACKET_TYPE(recv_buf) != BYTE_COMPLETION) {
			pr_warn("'inquiry' expect one 'completion' reply\n");
			return VISCA_ERR;
		}
		break;
	default:
		BUG();
	}
	return 0;
}

static inline int if_send_with_reply(struct visca_interface *iface) 
{
	return send_with_reply(iface->fd, 
			       iface->send_buf, iface->send_pos, 
			       iface->recv_buf, &iface->recv_pos);
}

static int fill_const_packet(const byte_t *data, int data_count,
			     byte_t *buf, int *count, int cam_addr) 
{
	int i;

	
	if (PACKET_IS_BROADCAST(data)){
		buf[0] = data[0];
	} else {
		BUG_ON(data[0] != BYTE_DIRECT);
		buf[0] = data[0] | cam_addr;
	}
	
	/* one terminator */
	BUG_ON(data_count + 1 > VISCA_IF_BUF_SIZE);
	for (i = 1; i < data_count; i++)
		buf[i] = data[i];

	buf[data_count] = TERMINATOR;
	*count = data_count + 1;
	return 0;
}

static inline void cmd_fill_const_packet(struct command *cmd, byte_t *buf, 
					int *count, int cam_addr) 
{
	fill_const_packet(cmd->data, cmd->count, buf, count, cam_addr);
}
static inline void __cmd_fill_packet(struct command *cmd, byte_t *buf, 
				     long arg)
{
	BUG_ON(cmd->arg_start < 0 
	       || cmd->arg_start >= VISCA_IF_BUF_SIZE);	
	cmd->fill_packet(buf + cmd->arg_start, arg);
}
static inline void __cmd_parse_packet(struct command *cmd, byte_t *buf, 
				      void *ret) 
{
	BUG_ON(cmd->ret_start < 0
	       || cmd->ret_start >= VISCA_IF_BUF_SIZE);
	cmd->parse_packet(buf + cmd->ret_start, ret);
}

/* static inline int if_trylock(struct visca_interface *iface)  */
/* { */
/* 	int error; */

/* 	LCALL1(error, pthread_mutex_trylock(&iface->lock), EBUSY); */
/* 	if (error == -EBUSY) { */
/* 		pr_warn("inteface already locked\n"); */
/* 		return VISCA_ERR; */
/* 	} */
/* 	return 0; */
/* } */
static inline void mutex_lock(pthread_mutex_t *lock)
{
	LCALL(pthread_mutex_lock(lock));
}
static inline void mutex_unlock(pthread_mutex_t *lock) {
	LCALL(pthread_mutex_unlock(lock));
}

int __visca_command(struct visca_interface *iface, int cmd_idx, 
		    int cam_addr, long arg, void *ret) 
{
	int error;
	struct command *cmd;

	if (!iface) {
		pr_warn("interface NULL\n");
		return VISCA_ERR;
	}

	if (cmd_idx >= visca_nr_max || cmd_idx < 0) {
		pr_warn("cmd_idx %d invalid\n", cmd_idx);
		return VISCA_ERR;
	}	

	if (cam_addr > 8 || cam_addr < 1) {
		pr_warn("cam_addr %d invalid\n", cam_addr);
		return VISCA_ERR;
	}

	cmd = cmds + cmd_idx;

	if (cmd->check_arg && !cmd->check_arg(arg))
		return VISCA_ERR;

	if (cmd->parse_packet && !ret) {
		pr_warn("command need non-NULL ret");
		return VISCA_ERR;
	}

	mutex_lock(&iface->lock);	
	
	if (!iface->opened) {
		pr_warn("iface is not opened\n");
		goto unlock;
	}
		
	cmd_fill_const_packet(cmd, iface->send_buf, 
			      &iface->send_pos, cam_addr);

	if (cmd->fill_packet)
	    __cmd_fill_packet(cmd, iface->send_buf, arg);	

	if ((error = if_send_with_reply(iface)))
		goto unlock;
	
	if (cmd->parse_packet)
		__cmd_parse_packet(cmd, iface->recv_buf, ret);

	/* fix visca pantilt_stop bug or error 
	 * 'command not executable' will occurs */
	if (cmd_idx == visca_nr_cmd_pantilt_stop) {
		/* minimal time interval for stop is around 163ms(by test)
		 * for current baud rate (9600) 
		 * we apply 180ms here
		 */
		LCALL(usleep(180000));
	}

unlock:
	mutex_unlock(&iface->lock);
	return error;
}

struct visca_interface *visca_alloc_init_if()
{
	struct visca_interface *iface;

	iface = (struct visca_interface *) malloc(sizeof(*iface));
	if (!iface)
		return NULL;
	iface->opened = 0;
	LCALL(pthread_mutex_init(&iface->lock, NULL));
	return iface;
}

static void __visca_close_if(struct visca_interface *iface)
{
	LCALL(flock(iface->fd, LOCK_UN));
	LCALL(close(iface->fd));

	iface->fd = -1;
	iface->opened = 0;	
}
int visca_close_if(struct visca_interface *iface)
{
	if (!iface) {
		pr_warn("interface NULL\n");
		return VISCA_ERR;
	}

	mutex_lock(&iface->lock);
	if (!iface->opened) {
		pr_warn("inteface already closed\n");
		goto fail;
	}

	__visca_close_if(iface);

	mutex_unlock(&iface->lock);
	return 0;
fail:
	mutex_unlock(&iface->lock);
	return VISCA_ERR;
}
void visca_free_if(struct visca_interface **_iface)
{
	struct visca_interface *iface = *_iface;
	
	if (!iface)
		return;

	mutex_lock(&iface->lock);
	*_iface = NULL;
	mutex_unlock(&iface->lock);

	if (iface->opened)
		__visca_close_if(iface);

	LCALL(pthread_mutex_destroy(&iface->lock));
	free(iface);
}

int visca_open_if(struct visca_interface *iface, char *devfile)
{
	int fd;
	struct termios options;

	if (!iface) {
		pr_warn("interface NULL\n");
		return VISCA_ERR;
	}
	
	mutex_lock(&iface->lock);

	if (iface->opened) {
		pr_warn("interface already opened\n");
		goto fail;
	}

	fd = open(devfile, O_RDWR | O_NDELAY | O_NOCTTY);
	if (fd < 0) {
		pr_warn("cannot open serial device %s\n", devfile);
		goto fail;
	}

	LCALL(flock(fd, LOCK_EX | LOCK_NB));

	LCALL(tcgetattr(fd, &options));
	LCALL(cfsetispeed(&options,B9600));
	cfmakeraw(&options);
	LCALL(tcsetattr(fd, TCSANOW, &options));
	
	iface->fd = fd;
	iface->opened = 1;

	mutex_unlock(&iface->lock);
	return 0;
fail:
	mutex_unlock(&iface->lock);
	return VISCA_ERR;
}


