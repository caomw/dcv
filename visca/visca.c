#include <unistd.h>
#include <stdio.h>
#include <termios.h>
#include <err.h>
#include <fcntl.h>
#include <errno.h>

#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/file.h>

#include "visca.h"

#define TERMINATOR	0xFF
#define SERIAL_DELAY  20000 /*us*/
#define RECV_POLL_WAIT  1000 /*us*/

/* 8X QQ RR ... FF, QQ = Command/Inquiry RR = category code */
#define BYTE_COMMAND	0x01
#define BYTE_INQUIRY    0x09

#define BYTE_INTERFACE	0x00
#define BYTE_CAMERA1	0x04
#define BYTE_PANTILTER	0x06

#define BYTE_ACK		0x40
#define BYTE_COMPLETION         0x50
#define BYTE_ERROR		0x60

#define BYTE_BROADCAST	0x88
#define BYTE_DIRECT     0x80

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

#define PACKET_IS_BROADCAST(buf) ((buf)[0] == BYTE_BROADCAST)
#define PACKET_TYPE(buf) ((buf)[1] & 0xf0 ? (buf)[1] & 0xf0 : (buf)[1])
#define ERROR_CODE(buf) ((buf)[2])

#define DIR_INIT 0
#define DIR_SEND 1
#define DIR_RECV 2

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

static inline int if_trylock(struct visca_interface *iface) 
{
	int error;
	error = pthread_mutex_trylock(&iface->lock);
	BUG_ON(error && error != -EBUSY);
	
	if (error == -EBUSY) {
		pr_warn("inteface already locked\n");
		return -1;
	}
	return 0;
}

static inline void if_unlock(struct visca_interface *iface) {
	BUG_ON(pthread_mutex_unlock(&iface->lock));
}

int visca_close_serial(struct visca_interface *iface)
{
	int error = 0;

	if ((error = if_trylock(iface)))
		return error;

	if (!iface->opened) {
		error = -1;
		pr_warn("inteface already closed\n");
		goto unlock;
	}

	BUG_ON(flock(iface->fd, LOCK_UN));
	BUG_ON(close(iface->fd));

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
		error = -1;
		pr_warn("interface already opened\n");
		goto unlock;
	}

	fd = open(devfile, O_RDWR | O_NDELAY | O_NOCTTY);
	if (fd < 0) {
		error = fd;
		pr_warn("cannot open serial device %s\n", devfile);
		goto unlock;
	}
	
	BUG_ON(flock(fd, LOCK_EX | LOCK_NB));

	BUG_ON(tcgetattr(fd, &options));
	BUG_ON(cfsetispeed(&options,B9600));
	cfmakeraw(&options);
	BUG_ON(tcsetattr(fd, TCSANOW, &options));
	
	iface->fd = fd;
	iface->opened = 1;

unlock:
	if_unlock(iface);
	return error;
}

struct command {
	byte_t *data;
	int count;
	int arg_start;
	int (*fill_packet)(const struct command *cmd, byte_t *buf, long arg);
	/* for inquire */
	int ret_start;
	void (*parse_packet)(const struct command *cmd, const byte_t *buf, void *ret);
};

#define __COMMAND_INIT(_data, _arg_start,  _fill_packet, _ret_start, _parse_packet) { \
		.data = (_data),				\
		.count = ARRAY_SIZE((_data)),			\
		.arg_start = (_arg_start),			\
		.fill_packet = (_fill_packet),			\
		.ret_start = (_ret_start),			\
		.parse_packet = (_parse_packet)                 \
}

#define COMMAND_INIT(data, arg_start, fill_packet) \
	__COMMAND_INIT(data, arg_start, fill_packet, -1, NULL)

#define COMMAND_INIT0(data) COMMAND_INIT(data, -1, NULL)

#define INQUIRY_INIT(data, ret_start, parse_packet) \
	__COMMAND_INIT(data, -1, NULL, ret_start, parse_packet)

static inline void to_4bytes(byte_t *buf, int *pos, int arg) {
	buf[(*pos)++] = (arg & 0xf000) >> 12;
	buf[(*pos)++] = (arg & 0x0f00) >> 8;
	buf[(*pos)++] = (arg & 0x00f0) >> 4;
	buf[(*pos)++] = (arg & 0x000f);
}


static inline int check_arg_limit(char *arg_name, int arg, int arg_min, int arg_max) 
{
	if (arg < arg_min || arg > arg_max) {
		pr_warn("%s %d out of band (%d, %d)\n", arg_name, arg, arg_min, arg_max);
		return 0;
	}
	return 1;
}

static inline int check_pt_speed(int p, int t) {
	return check_arg_limit("pan_speed", p, VISCA_PAN_SPEED_MIN, VISCA_PAN_SPEED_MAX)
		&& check_arg_limit("tilt_speed", t, VISCA_TILT_SPEED_MIN, VISCA_TILT_SPEED_MAX);
}

static int fill_zoom_speed(const struct command *cmd, byte_t *buf, long arg) 
{
	if (!check_arg_limit("zoom_speed", (int)arg, VISCA_ZOOM_SPEED_MIN, VISCA_ZOOM_SPEED_MAX))
		return -1;

	buf[cmd->arg_start] |= (byte_t) arg;
	return 0;
}

static int fill_zoom_pos(const struct command *cmd, byte_t *buf, long _arg) 
{
	int pos;	
	int arg = (int) _arg;
	
	if (!check_arg_limit("zoom_pos", arg, VISCA_ZOOM_POS_MIN, VISCA_ZOOM_POS_MAX))
		return -1;
	
	pos = cmd->arg_start;
	to_4bytes(buf, &pos, arg);
	return 0;
}

#define PAN_DIR(dir)  (((dir) & 0x0f00) >> 8)
#define TILT_DIR(dir) ((dir) & 0x000f)

static inline int check_pt_dir(char *arg_name, int arg) {
	if (!(arg == __PT_N || arg == __PT_P || arg == __PT_STOP)) {
		pr_warn("%s %d invalid\n", arg_name, arg);
		return 0;
	}
	return 1;
}

#define __fill_pt_speed(buf, pos, arg) do {	    \
		(buf)[(pos)++] = (arg)->pan_speed;   \
		(buf)[(pos)++] = (arg)->tilt_speed;  \
} while(0)

static int fill_pt_dir(const struct command *cmd, byte_t *buf, long _arg) 
{
	struct visca_pantilt_dir *arg = (struct visca_pantilt_dir *) _arg;
	int pos;
	int pan_dir, tilt_dir;

        if (!(check_pt_speed(arg->pan_speed, arg->tilt_speed)))
		return -1;
	
	pan_dir = PAN_DIR(arg->dir);
	if (!check_pt_dir("pan_dir", pan_dir))
		return -1;

	tilt_dir = TILT_DIR(arg->dir);
	if (!check_pt_dir("tilt_dir", tilt_dir))
		return -1;
	
	if (pan_dir == __PT_STOP && tilt_dir == __PT_STOP) {
		pr_warn("pan_dir tilt_dir must not be 0x03 0x03\n"
		     "use command pantilt_stop instead\n");
		return -1;
	}
	
	pos = cmd->arg_start;
	__fill_pt_speed(buf, pos, arg);
	buf[pos++] = pan_dir;
	buf[pos++] = tilt_dir;
	return 0;
}

static int fill_pt_pos(const struct command *cmd, byte_t *buf, long _arg) 
{
	struct visca_pantilt_pos *arg = (struct visca_pantilt_pos *) _arg;
	int pos;

        if (!(check_pt_speed(arg->pan_speed, arg->tilt_speed)))
		return -1;

	if (!(check_arg_limit("pan_pos", arg->pan_pos, 
			       VISCA_PAN_POS_MIN, VISCA_PAN_POS_MAX)
	      && check_arg_limit("tilt_pos", arg->tilt_pos, 
				  VISCA_TILT_POS_MIN, VISCA_TILT_POS_MAX)))
		return -1;
	
	pos = cmd->arg_start;
	__fill_pt_speed(buf, pos, arg);
	to_4bytes(buf, &pos, arg->pan_pos);
	to_4bytes(buf, &pos, arg->tilt_pos);
	return 0;
}

static void parse_zoom_pos(const struct command *cmd, const byte_t *buf, void *_ret)
{
	int *ret = (int *) _ret;
	int pos = cmd->ret_start;

	BUG_ON((buf[pos] & 0xf0) || (buf[pos + 1] & 0xf0) || (buf[pos + 2] & 0xf0) || (buf[pos + 3] & 0xf0));
	*ret = buf[pos] << 12 | buf[pos + 1] << 8 | buf[pos + 2] << 4 | buf[pos + 3];
}

static inline int from_2bytes(const byte_t *buf, int *pos) {
	int ret = buf[*pos] << 8 | buf[*pos + 1];
	(*pos) += 2;
	return ret;
}

static void parse_version(const struct command *cmd, const byte_t *buf, void *_ret)
{
	struct visca_version *ret = (struct visca_version *)_ret;
	int pos = cmd->ret_start;
	
	ret->vendor = from_2bytes(buf, &pos);
	ret->model = from_2bytes(buf, &pos);
	ret->rom_version = from_2bytes(buf, &pos);
}

static void parse_pantilt_status(const struct command *cmd, const byte_t *buf, void *_ret)
{
	int *ret = (int *)_ret;
	int pos = cmd->ret_start;

	*ret = from_2bytes(buf, &pos);
}

#define HEAD_BROADCAST(...)	{BYTE_BROADCAST, ##__VA_ARGS__}
#define HEAD_DIRECT(...)        {BYTE_DIRECT, ##__VA_ARGS__}

#define COMMAND(...)       	HEAD_DIRECT(BYTE_COMMAND, ##__VA_ARGS__)
#define INQUIRY(...)  		HEAD_DIRECT(BYTE_INQUIRY, ##__VA_ARGS__)

#define CMD_INTERFACE(...)     COMMAND(BYTE_INTERFACE, ##__VA_ARGS__)
#define INQ_INTERFACE(...)	INQUIRY(BYTE_INTERFACE, ##__VA_ARGS__)

#define CMD_CAMERA1(...)	COMMAND(BYTE_CAMERA1, ##__VA_ARGS__)
#define INQ_CAMERA1(...) 	INQUIRY(BYTE_CAMERA1, ##__VA_ARGS__)

#define CMD_PANTILTER(...)     COMMAND(BYTE_PANTILTER, ##__VA_ARGS__)
#define INQ_PANTILTER(...)	INQUIRY(BYTE_PANTILTER, ##__VA_ARGS__)

#define CMD_ZOOM(...)		CMD_CAMERA1(0x07, ##__VA_ARGS__)

static byte_t cmd_set_address[] = HEAD_BROADCAST(0x30, 0x01);
static byte_t cmd_clear_if[] = HEAD_BROADCAST(BYTE_COMMAND, BYTE_INTERFACE, 0x01);
static byte_t cmd_zoom_stop[] = CMD_ZOOM(0x00);
static byte_t cmd_zoom_tele[] = CMD_ZOOM(0x02);
static byte_t cmd_zoom_wide[] = CMD_ZOOM(0x03);
static byte_t cmd_zoom_tele_speed[] = CMD_ZOOM(0x20);
static byte_t cmd_zoom_wide_speed[] = CMD_ZOOM(0x30);
static byte_t cmd_zoom_direct[] = CMD_CAMERA1(0x47, 0x00, 0x00, 0x00, 0x00);
static byte_t cmd_pantilt_dir[] = CMD_PANTILTER(0x01, 0x00, 0x00, 0x00, 0x00);
static byte_t cmd_pantilt_stop[] = CMD_PANTILTER(0x01, VISCA_PAN_SPEED_MAX, VISCA_TILT_SPEED_MAX, __PT_STOP, __PT_STOP);
static byte_t cmd_pantilt_absolute_pos[] = CMD_PANTILTER(0x02, 0x00, 0x00, 
							   0x00, 0x00, 0x00, 0x00,
							   0x00, 0x00, 0x00, 0x00);
static byte_t cmd_pantilt_relative_pos[] = CMD_PANTILTER(0x03, 0x00, 0x00,
							 0x00, 0x00, 0x00, 0x00,
							 0x00, 0x00, 0x00, 0x00);
static byte_t cmd_pantilt_home[] = CMD_PANTILTER(0x04);
static byte_t cmd_pantilt_reset[] = CMD_PANTILTER(0x05);

static byte_t inq_zoom_pos[] = INQ_CAMERA1(0x47);
static byte_t inq_version[] = INQ_INTERFACE(0x02);
static byte_t inq_pantilt_status[] = INQ_PANTILTER(0x10);

#define CMD_ENTRY0(name) \
	[visca_nr_##name] = COMMAND_INIT0(cmd_##name)
#define CMD_ENTRY(name, ...) \
	[visca_nr_##name] = COMMAND_INIT(cmd_##name, __VA_ARGS__)
#define INQ_ENTRY(name, ...) \
	[visca_nr_inq_##name] = INQUIRY_INIT(inq_##name, __VA_ARGS__)

static struct command cmds[] = {
	CMD_ENTRY0(set_address),
	CMD_ENTRY0(clear_if),
	CMD_ENTRY0(zoom_stop),
	CMD_ENTRY0(zoom_tele),
	CMD_ENTRY0(zoom_wide),
	CMD_ENTRY(zoom_tele_speed, 4, fill_zoom_speed),
	CMD_ENTRY(zoom_wide_speed, 4, fill_zoom_speed),
	CMD_ENTRY(zoom_direct, 4, fill_zoom_pos),
	CMD_ENTRY(pantilt_dir, 4, fill_pt_dir),
	CMD_ENTRY0(pantilt_stop),
	CMD_ENTRY(pantilt_absolute_pos, 4, fill_pt_pos),
	CMD_ENTRY(pantilt_relative_pos, 4, fill_pt_pos),
	CMD_ENTRY0(pantilt_home),
	CMD_ENTRY0(pantilt_reset),
	INQ_ENTRY(zoom_pos, 2, parse_zoom_pos),
	INQ_ENTRY(version, 2, parse_version),
	INQ_ENTRY(pantilt_status, 2, parse_pantilt_status),
};

static int recv_packet(int fd, byte_t *buf, int *count)
{
	int i;
	int pos = 0;
	int bytes_read;
	
	BUG_ON(usleep(SERIAL_DELAY));

	for(;;) {
		bytes_read = read(fd, buf + pos, 1);
		BUG_ON(bytes_read < 0 && bytes_read != -EPERM);
		if (bytes_read > 0) {
			if (buf[pos] == TERMINATOR)
				break;
			pos += bytes_read;				
			if (pos > VISCA_IF_BUF_SIZE) {
				pr_warn("receive pos(%d) > VISCA_IF_BUF_SIZE(%d)\n", pos, VISCA_IF_BUF_SIZE);
				return -1;
			}
		}
		usleep(RECV_POLL_WAIT);
	}
	*count = pos + 1;

	dbg_pack(buf, DIR_RECV);

	if (PACKET_TYPE(buf) == BYTE_ERROR) {
		int error_code = ERROR_CODE(buf);

		for (i = 0; i < ERROR_MAX; i++) {
			if (error_tab[i].code == error_code) {
				pr_warn(error_tab[i].desc);
				break;
			}
		}

		if (error_code != ERROR_CMD_CANCELLED)
			return -1;
	}
	return 0;
}

static int send_packet(int fd, const byte_t *buf, int count) {
	int ret;
	
	ret = write(fd, buf, count);
	BUG_ON(ret < 0);
	if (ret < count) {
		pr_warn("write packet to fd %d failed\n", fd);
		return -1;		
	}

	BUG_ON(usleep(SERIAL_DELAY));
	return 0;
}

static int send_with_reply(int fd, const byte_t *send_buf, int send_count, byte_t *recv_buf, int *recv_count) 
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
			pr_warn("'broadcast 0x88' expect 'broadcast 0x88' reply\n");
			return -1;
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
			return -1;
		}
		break;
	case BYTE_INQUIRY:
		if ((error = recv_packet(fd, recv_buf, recv_count)))
			return error;
		if (PACKET_TYPE(recv_buf) != BYTE_COMPLETION) {
			pr_warn("'inquiry' expect one 'completion' reply\n");
			return -1;
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

static int fill_const_packet(struct command *cmd, byte_t *buf, int *count, int cam_addr) {
	int i;

	if (cam_addr > 8 || cam_addr < 1) {
		pr_warn("cam_addr %d invalid\n", cam_addr);
		return -1;
	}
	
	if (PACKET_IS_BROADCAST(cmd->data)){
		buf[0] = cmd->data[0];
	} else {
		BUG_ON(cmd->data[0] != BYTE_DIRECT);
		buf[0] = cmd->data[0] | cam_addr;
	}
	
	/* one terminator */
	BUG_ON(cmd->count + 1 > VISCA_IF_BUF_SIZE);
	for (i = 1; i < cmd->count; i++)
		buf[i] = cmd->data[i];

	buf[cmd->count] = TERMINATOR;
	*count = cmd->count + 1;
	return 0;
}

int __visca_command(struct visca_interface *iface, int cmd_idx, int cam_addr, long arg) 
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

	if ((error = fill_const_packet(cmd, iface->send_buf, &iface->send_pos, cam_addr)))
		goto unlock;

	if (cmd->fill_packet) {
		BUG_ON(cmd->arg_start < 0);
		if ((error = cmd->fill_packet(cmd, iface->send_buf, arg)))
			goto unlock;
	}

	if ((error = if_send_with_reply(iface)))
		goto unlock;

	
	if (cmd->parse_packet) {
		BUG_ON(cmd->ret_start < 0);
		cmd->parse_packet(cmd, iface->recv_buf, (void *) arg);
	}


	if_unlock(iface);
        /* fix visca pantilt_stop bug or error 
	 * 'command not executable' will occurs */
	if (cmd_idx == visca_nr_pantilt_stop) {
		/* minimal time interval for stop is around 163ms(by test)
		 * for current baud rate (9600) 
		 * we apply 180ms here
		 */
		BUG_ON(usleep(180000));
	}
	return error;

unlock:
	if_unlock(iface);
	return error;
}
