#include <unistd.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>

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

static inline void __to_byte(byte_t* buf, int x)
{
	buf[0] = (byte_t) x;
}
static inline int __from_byte(const byte_t *buf) 
{
	return buf[0];
}
static inline int __from_2bytes(const byte_t *buf)
{
	return buf[0] << 8 | buf[1];
}
static inline void to_byte(byte_t *buf, int *pos, int x) 
{
	buf[(*pos)++] = (byte_t) x;
}

static inline void to_4lbytes(byte_t *buf, int *pos, int x) 
{
	to_byte(buf, pos, (x & 0xf000) >> 12);
	to_byte(buf, pos, (x & 0x0f00) >> 8);
	to_byte(buf, pos, (x & 0x00f0) >> 4);
	to_byte(buf, pos, (x & 0x000f));
}

static inline int from_byte(const byte_t *buf, int *pos) 
{
	return buf[(*pos)++];
}

static inline int from_2bytes(const byte_t *buf, int *pos) {
	int ret = 0;
	
	ret |= (from_byte(buf, pos) << 8);
	ret |= from_byte(buf, pos);
	return ret;
}

static inline int from_4lbytes(const byte_t *buf, int *pos) 
{
	int ret = 0;

	BUG_ON((buf[*pos] & 0xf0) || (buf[*pos + 1] & 0xf0) 
	       || (buf[*pos + 2] & 0xf0) || (buf[*pos + 3] & 0xf0));
	
	ret |= (from_byte(buf, pos) << 12);
	ret |= (from_byte(buf, pos) << 8);
	ret |= (from_byte(buf, pos) << 4);
	ret |= from_byte(buf, pos);
	return ret;
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

/* zoom_tele_speed, zoom_wide_speed */
#define check_z_speed(x)  check_limit("zoom_speed", (x),	\
				      VISCA_ZOOM_SPEED_MIN,	\
				      VISCA_ZOOM_SPEED_MAX)
static int fill_z_speed(byte_t *buf, long arg) 
{
	int z_speed = (int) arg;
	if (!check_z_speed(z_speed))
		return EXIT_ERR;
	    	
	buf[0] |= (byte_t) z_speed;
	return 0;
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
	int pos = 0;
	
	version->vendor = from_2bytes(buf, &pos);
	version->model = from_2bytes(buf, &pos);
	version->rom_version = from_2bytes(buf, &pos);
}

/* pantilt_status */
static void parse_pt_status(const byte_t *buf, void *ret)
{
	int *status = (int *)ret;

	*status = __from_2bytes(buf);
}

/* zoom_pos, zoom_direct */
#define check_z_pos(x)	check_limit("zoom_pos", (x),	\
				    VISCA_ZOOM_POS_MIN,	\
				    VISCA_ZOOM_POS_MAX)
static int fill_z_pos(byte_t *buf, long arg) 
{
	int pos = 0;	
	int z_pos = (int) arg;
	
	if (!check_z_pos(z_pos))
		return EXIT_ERR;
	
	to_4lbytes(buf, &pos, arg);
	return 0;
}
static void parse_z_pos(const byte_t *buf, void *ret)
{
	int *z_pos = (int*) ret;
	int pos = 0;

	*z_pos = from_4lbytes(buf, &pos);

	BUG_ON(!check_z_pos(*z_pos));
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
static inline void __fill_pt_speed(byte_t *buf, int *pos, 
				   const struct pt_speed *speed) 
{
	to_byte(buf, pos, speed->p);
	to_byte(buf, pos, speed->t);
}
static inline bool check_pt_speed(const struct pt_speed *speed) 
{
	return check_limit("pan_speed", speed->p, 
			   VISCA_PAN_SPEED_MIN, VISCA_PAN_SPEED_MAX)
		&& check_limit("tilt_speed", speed->t, 
			       VISCA_TILT_SPEED_MIN, VISCA_TILT_SPEED_MAX);
}
static void parse_pt_speed(const byte_t *buf, void *ret)
{
	struct pt_speed *pt_speed = (struct pt_speed *) ret;
	int pos = 0;
	
	pt_speed->p = from_byte(buf, &pos);
	pt_speed->t = from_byte(buf, &pos);
	
	BUG_ON(!check_pt_speed(pt_speed));
}

/* pantilt_dir */
static inline bool __check_pt_dir(char *name, int dir) 
{
	if (!(dir == __PT_N || dir == __PT_P || dir == __PT_STOP)) {
		pr_warn("%s %d invalid\n", name, dir);
		return false;
	}
	return true;
}
static inline bool check_pt_dir(int p_dir, int t_dir)
{
	if (!__check_pt_dir("pan_dir", p_dir))
		return false;

	if (!__check_pt_dir("tilt_dir", t_dir))
		return false;
	
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
#define PAN_DIR(dir)  (((dir) & 0x0f00) >> 8)
#define TILT_DIR(dir) ((dir) & 0x000f)
static int fill_pt_speed_dir(byte_t *buf, long arg) 
{
	struct pt_speed_dir *speed_dir = (struct pt_speed_dir *) arg;
	int pos = 0;
	int p_dir = PAN_DIR(speed_dir->dir);
	int t_dir = TILT_DIR(speed_dir->dir);

        if (!check_pt_speed(&speed_dir->speed))
		return EXIT_ERR;

	if (!check_pt_dir(p_dir, t_dir))
		return EXIT_ERR;

	__fill_pt_speed(buf, &pos, &speed_dir->speed);
	to_byte(buf, &pos, p_dir);
	to_byte(buf, &pos, t_dir);
	return 0;
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
static inline bool check_pt_pos(struct pt_pos *pos) 
{
	return check_limit("pan_pos", pos->p, 
			   VISCA_PAN_POS_MIN, VISCA_PAN_POS_MAX)
		&& check_limit("tilt_pos", pos->t, 
			       VISCA_TILT_POS_MIN, VISCA_TILT_POS_MAX);
}
static void parse_pt_pos(const byte_t *buf, void *ret)
{
	struct pt_pos *pt_pos = (struct pt_pos *) ret;
	int pos = 0;

	pt_pos->p = from_4lbytes(buf, &pos);
	pt_pos->t = from_4lbytes(buf, &pos);
	BUG_ON(!check_pt_pos(pt_pos));
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
static int fill_pt_speed_pos(byte_t *buf, long arg) 
{
	struct pt_speed_pos *speed_pos = (struct pt_speed_pos *) arg;
	int pos = 0;

        if (!check_pt_speed(&speed_pos->speed))
		return EXIT_ERR;

	if (!check_pt_pos(&speed_pos->pos))
		return EXIT_ERR;
	
	__fill_pt_speed(buf, &pos, &speed_pos->speed);
	to_4lbytes(buf, &pos, speed_pos->pos.p);
	to_4lbytes(buf, &pos, speed_pos->pos.t);
	return 0;
}

/* {inq_}dzoom_mode */
#define DZOOM_MODE_ON		0x02
#define DZOOM_MODE_OFF		0x03
#define check_dzoom_mode(x)  ((x) == DZOOM_MODE_ON || (x) == DZOOM_MODE_OFF)
static void parse_dzoom_mode(const byte_t *buf, void *ret) 
{
	bool mode;

	mode = __from_byte(buf);
	BUG_ON(!check_dzoom_mode(mode));
	*(bool*)ret = mode == DZOOM_MODE_ON;
}
static int fill_dzoom_mode(byte_t *buf, long arg) {
	bool mode = (bool) arg;

	if (mode)
		__to_byte(buf, DZOOM_MODE_ON);
	else
		__to_byte(buf, DZOOM_MODE_OFF);
	return 0;
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
	/* must be constant, or init by macro will fail */
	byte_t data[COMMAND_DATA_SIZE];
	int count;
	int arg_start;
	int (*fill_packet)(byte_t *buf, long arg);
	/* for inquire */
	int ret_start;
	void (*parse_packet)(const byte_t *buf, void *ret);
};

#define __COMMAND_INIT(_arg_start, _fill_packet,		\
		       _ret_start, _parse_packet, ...)	\
{							       \
	.arg_start = (_arg_start),			\
	.fill_packet = (_fill_packet),				\
	.ret_start = (_ret_start),				\
		.parse_packet = (_parse_packet),		       \
	 .count = ARG_NR(byte_t, __VA_ARGS__), \
		.data = {__VA_ARGS__},      \
}

#define COMMAND_INIT(arg_start, fill_packet, ...)			\
	__COMMAND_INIT(arg_start, fill_packet, -1, NULL, __VA_ARGS__)

#define COMMAND_INIT0(...) COMMAND_INIT(-1, NULL, __VA_ARGS__)

#define INQUIRY_INIT(ret_start, parse_packet,  ...)		\
	__COMMAND_INIT(-1, NULL, ret_start, parse_packet, __VA_ARGS__)

#define CMD_ENTRY0(name, ...)					\
	[visca_nr_cmd_##name] = COMMAND_INIT0(__VA_ARGS__)
#define CMD_ENTRY(name, arg_start, fill_packet, ...)		\
	[visca_nr_cmd_##name] = COMMAND_INIT(arg_start, fill_packet,	\
					      __VA_ARGS__)
#define INQ_ENTRY(name, ret_start, parse_packet, ...)		\
	[visca_nr_inq_##name] = INQUIRY_INIT(ret_start, parse_packet,	\
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
		   CMD_PANTILTER(0x01, VISCA_PAN_SPEED_MAX,
				 VISCA_TILT_SPEED_MAX,
				 __PT_STOP, __PT_STOP)),
	CMD_ENTRY0(pantilt_home, CMD_PANTILTER(0x04)),
	CMD_ENTRY0(pantilt_reset, CMD_PANTILTER(0x05))
	/*	have some arguments */,
	CMD_ENTRY(zoom_tele_speed, 4, fill_z_speed, CMD_ZOOM(0x20)),
	CMD_ENTRY(zoom_wide_speed, 4, fill_z_speed, CMD_ZOOM(0x30)),

	/* inquiries without command */
	INQ_ENTRY(version, 2, parse_version, INQ_INTERFACE(0x02)),
	INQ_ENTRY(pantilt_status, 2, parse_pt_status, INQ_PANTILTER(0x10)),
	
	/* /\* general *\/ */
	INQ_ENTRY(zoom_pos, 2, parse_z_pos, INQ_CAMERA1(0x47)),
	CMD_ENTRY(zoom_direct, 4, fill_z_pos,
		  CMD_CAMERA1(0x47, 0x00, 0x00, 0x00, 0x00)),

	INQ_ENTRY(pantilt_maxspeed, 2, parse_pt_speed, INQ_PANTILTER(0x11)),
	CMD_ENTRY(pantilt_dir, 4, fill_pt_speed_dir,
		  CMD_PANTILTER(0x01, PH4)),

	INQ_ENTRY(pantilt_pos, 2, parse_pt_pos, INQ_PANTILTER(0x12)),
	CMD_ENTRY(pantilt_absolute_pos, 4, fill_pt_speed_pos,
		  CMD_PANTILTER(0x02, PH10)),
	CMD_ENTRY(pantilt_relative_pos, 4, fill_pt_speed_pos,
		  CMD_PANTILTER(0x03, PH10)),

	INQ_ENTRY(dzoom_mode, 2, parse_dzoom_mode, INQ_CAMERA1(0x06)),
	CMD_ENTRY(dzoom_mode, 4, fill_dzoom_mode,
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
	
	SYS_BUG(usleep(SERIAL_DELAY));

	for(;;) {
		RW_SYS_BUG1(bytes_read, read(fd, buf + pos, 1), EPERM);
		if (bytes_read > 0) {
			if (buf[pos] == TERMINATOR)
				break;
			pos += bytes_read;				
			if (pos > VISCA_IF_BUF_SIZE) {
				pr_warn("receive pos(%d) > buf_size(%d)\n", 
					pos, VISCA_IF_BUF_SIZE);
				return EXIT_ERR;
			}
		}
		SYS_BUG(usleep(RECV_POLL_WAIT));
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
			return EXIT_ERR;
	}
	return 0;
}

static int send_packet(int fd, const byte_t *buf, int count) {
	int ret;
	
	RW_SYS_BUG(ret, write(fd, buf, count));
	if (ret < count) {
		pr_warn("write packet to fd %d failed\n", fd);
		return EXIT_ERR;		
	}

	SYS_BUG(usleep(SERIAL_DELAY));
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
			return EXIT_ERR;
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
			return EXIT_ERR;
		}
		break;
	case BYTE_INQUIRY:
		if ((error = recv_packet(fd, recv_buf, recv_count)))
			return error;
		if (PACKET_TYPE(recv_buf) != BYTE_COMPLETION) {
			pr_warn("'inquiry' expect one 'completion' reply\n");
			return EXIT_ERR;
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

	if (cam_addr > 8 || cam_addr < 1) {
		pr_warn("cam_addr %d invalid\n", cam_addr);
		return EXIT_ERR;
	}
	
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

static inline int cmd_fill_const_packet(struct command *cmd, byte_t *buf, 
					int *count, int cam_addr) 
{
	return fill_const_packet(cmd->data, cmd->count, buf, count, cam_addr);
}
static inline int __cmd_fill_packet(struct command *cmd, byte_t *buf, long arg)
{
	BUG_ON(cmd->arg_start < 0 
	       || cmd->arg_start >= VISCA_IF_BUF_SIZE);	
	return cmd->fill_packet(buf + cmd->arg_start, arg);
}
static inline void __cmd_parse_packet(struct command *cmd, byte_t *buf, void *ret) {
	BUG_ON(cmd->ret_start < 0
	       || cmd->ret_start >= VISCA_IF_BUF_SIZE);
	cmd->parse_packet(buf + cmd->ret_start, ret);
}

static inline int if_trylock(struct visca_interface *iface) 
{
	int error;

	SYS_BUG1(error, pthread_mutex_trylock(&iface->lock), EBUSY);
	if (error == -EBUSY) {
		pr_warn("inteface already locked\n");
		return EXIT_ERR;
	}
	return 0;
}

static inline void if_unlock(struct visca_interface *iface) {
	SYS_BUG(pthread_mutex_unlock(&iface->lock));
}

int visca_close_serial(struct visca_interface *iface)
{
	int error = 0;

	if ((error = if_trylock(iface)))
		return error;

	if (!iface->opened) {
		error = EXIT_ERR;
		pr_warn("inteface already closed\n");
		goto unlock;
	}

	SYS_BUG(flock(iface->fd, LOCK_UN));
	SYS_BUG(close(iface->fd));

	iface->fd = -1;
	iface->opened = 0;

unlock:
	if_unlock(iface);
	return error;
}

int visca_open_serial(struct visca_interface *iface, char *devfile)
{
	int fd;
	int error = 0;
	struct termios options;

	if ((error = if_trylock(iface)))
		return error;

	if (iface->opened) {
		error = EXIT_ERR;
		pr_warn("interface already opened\n");
		goto unlock;
	}

	fd = open(devfile, O_RDWR | O_NDELAY | O_NOCTTY);
	if (fd < 0) {
		error = fd;
		pr_warn("cannot open serial device %s\n", devfile);
		goto unlock;
	}
	
	SYS_BUG(flock(fd, LOCK_EX | LOCK_NB));

	SYS_BUG(tcgetattr(fd, &options));
	SYS_BUG(cfsetispeed(&options,B9600));
	cfmakeraw(&options);
	SYS_BUG(tcsetattr(fd, TCSANOW, &options));
	
	iface->fd = fd;
	iface->opened = 1;

unlock:
	if_unlock(iface);
	return error;
}

int __visca_command(struct visca_interface *iface, int cmd_idx, int cam_addr, 
		    long arg, void *ret) 
{
	int error = 0;
	struct command *cmd;

	if ((error = if_trylock(iface)))
		return error;
		
	if (!iface->opened) {
		pr_warn("iface is not opened\n");
		goto unlock;
	}

	BUG_ON(cmd_idx >= visca_nr_max);
	cmd = cmds + cmd_idx;

	if ((error = cmd_fill_const_packet(cmd, iface->send_buf, 
					   &iface->send_pos, cam_addr)))
		goto unlock;

	if (cmd->fill_packet) {
		if ((error = __cmd_fill_packet(cmd, iface->send_buf, arg)))
			goto unlock;
	}

	if ((error = if_send_with_reply(iface)))
		goto unlock;

	
	if (ret && cmd->parse_packet)
		__cmd_parse_packet(cmd, iface->recv_buf, ret);


	if_unlock(iface);
	/* fix visca pantilt_stop bug or error 
	 * 'command not executable' will occurs */
	if (cmd_idx == visca_nr_cmd_pantilt_stop) {
		/* minimal time interval for stop is around 163ms(by test)
		 * for current baud rate (9600) 
		 * we apply 180ms here
		 */
		SYS_BUG(usleep(180000));
	}
	return error;

unlock:
	if_unlock(iface);
	return error;
}

int _visca_pantilt_dir(struct visca_interface *iface, int cam_addr, 
		       int pan_speed, int tilt_speed, int dir)
{
	struct pt_speed_dir speed_dir = PT_SPEED_DIR_INIT(pan_speed,
							  tilt_speed, dir);
	return __visca_cmd_command(iface, visca_nr_cmd_pantilt_dir, 
				   cam_addr, (long) &speed_dir);
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

int _visca_inq_version(struct visca_interface *iface, int cam_addr,
		       int *vendor, int *model, int *rom_version)
{
	int err = 0;
	struct version version;
	
	if ((err = __visca_inq_command(iface, visca_nr_inq_version, 
				       cam_addr, &version)))
		return err;
	
	*vendor = version.vendor;
	*model = version.model;
	*rom_version = version.rom_version;
	return err;
}

int _visca_inq_pantilt_maxspeed(struct visca_interface *iface, int cam_addr,
				int *pan_speed, int *tilt_speed)
{
	int err = 0;
	struct  pt_speed pt_speed;
	
	if ((err = __visca_inq_command(iface, visca_nr_inq_pantilt_maxspeed, 
				       cam_addr, &pt_speed)))
		return err;
	
	*pan_speed = pt_speed.p;
	*tilt_speed = pt_speed.t;
	return err;
}
int _visca_inq_pantilt_pos(struct visca_interface *iface, int cam_addr,
			   int *pan_pos, int *tilt_pos)
{
	int err = 0;
	struct  pt_pos pt_pos;
	
	if ((err = __visca_inq_command(iface, visca_nr_inq_pantilt_pos, 
				       cam_addr, &pt_pos)))
		return err;
	
	*pan_pos = pt_pos.p;
	*tilt_pos = pt_pos.t;
	return err;
}

