#include <unistd.h>
#include <stdio.h>
#include <termios.h>
#include <err.h>
#include <fcntl.h>
#include <libgen.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/file.h>

#include "visca.h"

#define ARR(...) __VA_ARGS__
#define ARRAY_SIZE(x) (sizeof((x))/sizeof((x)[0]))

#define TERMINATOR	0xFF
#define DELAY  20000 /*us*/
#define RECEIVE_POLL_INTERVAL  1000 /*us*/

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

struct err_msg {
	byte_t code;
	char *desc;
};

#define ERR_MAX 6
#define ERR_CMD_CANCELLED  0x04

static const struct err_msg err_tab[] = {
	{.code = 0x01, .desc = "message length error"},
	{.code = 0x02, .desc = "syntax error"},
	{.code = 0x03, .desc = "command buffer full"},
	{.code = ERR_CMD_CANCELLED, .desc = "command cancelled"},
	{.code = 0x05, .desc = "no socket"},
	{.code = 0x41, .desc = "command not executable"},
};

static void print_err_msg(byte_t code) {
	int i;

	for (i = 0; i < ERR_MAX; i++) {
		if (err_tab[i].code == code) {
			warnx(err_tab[i].desc);
			return;
		}
	}
}

#define PACKET_IS_BROADCAST(pack) ((pack)->buf[0] == BYTE_BROADCAST)
#define __PACKET_TYPE(x) ((x) & 0xf0 ? (x) & 0xf0 : (x))
#define PACKET_TYPE(pack) __PACKET_TYPE(pack->buf[1])
#define ERR_CODE(pack) ((pack)->buf[2])

#define DIR_INIT 0
#define DIR_SEND 1
#define DIR_RECV 2

void visca_init_pack(struct visca_packet *pack) {
	pack->len = 0;
	pack->dir = DIR_INIT;
}

#ifdef DEBUG
void debug_pack(struct visca_packet *pack) 
{	
	int i;
	
	BUG_ON(pack->dir != DIR_SEND && pack->dir != DIR_RECV);
	if (pack->dir == DIR_SEND)
		debug("S");
	else
		debug("R");
	for (i = 0; i < pack->len || pack->buf[i] == TERMINATOR; i++)
		debug(" %02x", pack->buf[i]);
	debug("\n");	
}
#else
void debug_pack(struct visca_packet *pack) {}
#endif

void visca_close_serial(int fd)
{
	flock(fd, LOCK_UN);
	close(fd);
}

int visca_open_serial(char *devfile)
{
	int fd;
	int err;
	struct termios options;
	
	fd = open(devfile, O_RDWR | O_NDELAY | O_NOCTTY);
	if (fd == -1) {
		warn("cannot open serial device %s\n", devfile);
		return -1;
	}
	
	if ((err = flock(fd, LOCK_EX | LOCK_NB) < 0)) {
		warn("cannot lock file %d", fd);
		return -1;
	}

	if ((err = fcntl(fd, F_SETFL,0)) < 0) {
		warn("fcntl fd %d failed", fd);
		return -1;
	}

	tcgetattr(fd, &options);
	cfsetispeed(&options,B9600);
	cfmakeraw(&options);
	tcsetattr(fd, TCSANOW, &options);

	return fd;
}

struct command {
	byte_t *data;
	int len;
	int para_start;
	int (*fill_packet)(const struct command *cmd, struct visca_packet *pack, long para);
	/* for inquire */
	int ret_start;
	void (*parse_packet)(const struct command *cmd, const struct visca_packet *pack, void *ret);
};

#define __COMMAND_INITIALIZER(_data, _para_start,  _fill_packet, _ret_start, _parse_packet) { \
		.data = (_data),				\
		.len = ARRAY_SIZE((_data)),			\
		.para_start = (_para_start),			\
		.fill_packet = (_fill_packet),			\
		.ret_start = (_ret_start),			\
		.parse_packet = (_parse_packet)                 \
}

#define COMMAND_INITIALIZER(data, para_start, fill_packet) \
	__COMMAND_INITIALIZER(data, para_start, fill_packet, -1, NULL)

#define COMMAND_INITIALIZER0(data) COMMAND_INITIALIZER(data, -1, NULL)

#define INQUIRY_INITIALIZER(data, ret_start, parse_packet) \
	__COMMAND_INITIALIZER(data, -1, NULL, ret_start, parse_packet)

static void to_4bytes(byte_t *buf, int *pos, int para) {
	buf[(*pos)++] = (para & 0xf000) >> 12;
	buf[(*pos)++] = (para & 0x0f00) >> 8;
	buf[(*pos)++] = (para & 0x00f0) >> 4;
	buf[(*pos)++] = (para & 0x000f);
}


static int check_para_limit(char *para_name, int para, int para_min, int para_max) 
{
	if (para < para_min || para > para_max) {
		warnx("%s %d out of band (%d, %d)", para_name, para, para_min, para_max);
		return 0;
	}
	return 1;
}

static int check_pt_speed(int p, int t) {
	return check_para_limit("pan_speed", p, VISCA_PAN_SPEED_MIN, VISCA_PAN_SPEED_MAX)
		&& check_para_limit("tilt_speed", t, VISCA_TILT_SPEED_MIN, VISCA_TILT_SPEED_MAX);
}

static int fill_zoom_speed(const struct command *cmd, struct visca_packet *pack, long para) 
{
	if (!check_para_limit("zoom_speed", (int)para, VISCA_ZOOM_SPEED_MIN, VISCA_ZOOM_SPEED_MAX))
		return -1;

	pack->buf[cmd->para_start] |= (byte_t) para;
	return 0;
}

static int fill_zoom_pos(const struct command *cmd, struct visca_packet *pack, long _para) 
{
	int pos;	
	int para = (int) _para;
	
	if (!check_para_limit("zoom_pos", para, VISCA_ZOOM_POS_MIN, VISCA_ZOOM_POS_MAX))
		return -1;
	
	pos = cmd->para_start;
	to_4bytes(pack->buf, &pos, para);
	return 0;
}

#define PAN_DIR(dir)  (((dir) & 0x0f00) >> 8)
#define TILT_DIR(dir) ((dir) & 0x000f)

static int check_pt_dir(char *para_name, int para) {
	if (!(para == PT_N || para == PT_P || para == PT_STOP)) {
		warnx("%s %d invalid", para_name, para);
		return 0;
	}
	return 1;
}

#define __fill_pt_speed(buf, pos, para) do {	    \
		(buf)[(pos)++] = (para)->pan_speed;   \
		(buf)[(pos)++] = (para)->tilt_speed;  \
} while(0)

static int fill_pt_dir(const struct command *cmd, struct visca_packet *pack, long _para) 
{
	struct visca_pantilt_dir *para = (struct visca_pantilt_dir *) _para;
	int pos;
	int pan_dir, tilt_dir;

        if (!(check_pt_speed(para->pan_speed, para->tilt_speed)))
		return -1;
	
	pan_dir = PAN_DIR(para->dir);
	if (!check_pt_dir("pan_dir", pan_dir))
		return -1;

	tilt_dir = TILT_DIR(para->dir);
	if (!check_pt_dir("tilt_dir", tilt_dir))
		return -1;
	
	if (pan_dir == PT_STOP && tilt_dir == PT_STOP) {
		warn("pan_dir tilt_dir must not be 0x03 0x03,"
		     "use command pantilt_stop instead");
		return -1;
	}
	
	pos = cmd->para_start;
	__fill_pt_speed(pack->buf, pos, para);
	pack->buf[pos++] = pan_dir;
	pack->buf[pos++] = tilt_dir;
	return 0;
}

static int fill_pt_pos(const struct command *cmd, struct visca_packet *pack, long _para) 
{
	struct visca_pantilt_pos *para = (struct visca_pantilt_pos *) _para;
	int pos;

        if (!(check_pt_speed(para->pan_speed, para->tilt_speed)))
		return -1;

	if (!(check_para_limit("pan_pos", para->pan_pos, 
			       VISCA_PAN_POS_MIN, VISCA_PAN_POS_MAX)
	      && check_para_limit("tilt_pos", para->tilt_pos, 
				  VISCA_TILT_POS_MIN, VISCA_TILT_POS_MAX)))
		return -1;
	
	pos = cmd->para_start;
	__fill_pt_speed(pack->buf, pos, para);
	to_4bytes(pack->buf, &pos, para->pan_pos);
	to_4bytes(pack->buf, &pos, para->tilt_pos);
	return 0;
}

static void parse_zoom_pos(const struct command *cmd, const struct visca_packet *pack, void *_ret)
{
	int *ret = (int *) _ret;
	const byte_t *buf = pack->buf + cmd->ret_start;
		
	BUG_ON((buf[0] & 0xf0) || (buf[1] & 0xf0) || (buf[2] & 0xf0) || (buf[3] & 0xf0));
	*ret = buf[0] << 12 | buf[1] << 8 | buf[2] << 4 | buf[3];
}

static int from_2bytes(const byte_t *buf, int *pos) {
	int ret = buf[*pos] << 8 | buf[*pos + 1];
	(*pos) += 2;
	return ret;
}

static void parse_version(const struct command *cmd, const struct visca_packet *pack, void *_ret)
{
	struct visca_version *ret = (struct visca_version *)_ret;
	int pos = cmd->ret_start;
	
	ret->vendor = from_2bytes(pack->buf, &pos);
	ret->model = from_2bytes(pack->buf, &pos);
	ret->rom_version = from_2bytes(pack->buf, &pos);
}

static void parse_pantilt_status(const struct command *cmd, const struct visca_packet *pack, void *_ret)
{
	int *ret = (int *)_ret;
	int pos = cmd->ret_start;

	*ret = from_2bytes(pack->buf, &pos);
}

#define HEAD_BROADCAST(...)	{BYTE_BROADCAST, ##__VA_ARGS__}
#define HEAD_DIRECT(...)        {BYTE_DIRECT, ##__VA_ARGS__}

#define COMMAND(...)       HEAD_DIRECT(BYTE_COMMAND, ##__VA_ARGS__)
#define CMD_INTERFACE(...)     COMMAND(BYTE_INTERFACE, ##__VA_ARGS__)
#define CMD_CAMERA1(...)	COMMAND(BYTE_CAMERA1, ##__VA_ARGS__)
#define CMD_PANTILTER(...)     COMMAND(BYTE_PANTILTER, ##__VA_ARGS__)

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
static byte_t cmd_pantilt_stop[] = CMD_PANTILTER(0x01, VISCA_PAN_SPEED_MAX, VISCA_TILT_SPEED_MAX, PT_STOP, PT_STOP);
static byte_t cmd_pantilt_absolute_pos[] = CMD_PANTILTER(0x02, 0x00, 0x00, 
							   0x00, 0x00, 0x00, 0x00,
							   0x00, 0x00, 0x00, 0x00);
static byte_t cmd_pantilt_relative_pos[] = CMD_PANTILTER(0x03, 0x00, 0x00,
							 0x00, 0x00, 0x00, 0x00,
							 0x00, 0x00, 0x00, 0x00);
static byte_t cmd_pantilt_home[] = CMD_PANTILTER(0x04);
static byte_t cmd_pantilt_reset[] = CMD_PANTILTER(0x05);

static struct command cmds[] = {
	[VISCA_CMD_SET_ADDRESS] = COMMAND_INITIALIZER0(cmd_set_address),
	[VISCA_CMD_CLEAR_IF] = COMMAND_INITIALIZER0(cmd_clear_if),
	[VISCA_CMD_ZOOM_STOP] = COMMAND_INITIALIZER0(cmd_zoom_stop),
	[VISCA_CMD_ZOOM_TELE] = COMMAND_INITIALIZER0(cmd_zoom_tele),
	[VISCA_CMD_ZOOM_WIDE] = COMMAND_INITIALIZER0(cmd_zoom_wide),
	[VISCA_CMD_ZOOM_TELE_SPEED] = COMMAND_INITIALIZER(cmd_zoom_tele_speed, 4, fill_zoom_speed),
	[VISCA_CMD_ZOOM_WIDE_SPEED] = COMMAND_INITIALIZER(cmd_zoom_wide_speed, 4, fill_zoom_speed),
	[VISCA_CMD_ZOOM_DIRECT] = COMMAND_INITIALIZER(cmd_zoom_direct, 4, fill_zoom_pos),
	[VISCA_CMD_PANTILT_DIR] = COMMAND_INITIALIZER(cmd_pantilt_dir, 4, fill_pt_dir),
	[VISCA_CMD_PANTILT_STOP] = COMMAND_INITIALIZER0(cmd_pantilt_stop),
	[VISCA_CMD_PANTILT_ABSOLUTE_POS] = COMMAND_INITIALIZER(cmd_pantilt_absolute_pos, 4,fill_pt_pos),
	[VISCA_CMD_PANTILT_RELATIVE_POS] = COMMAND_INITIALIZER(cmd_pantilt_relative_pos, 4,
fill_pt_pos),
	[VISCA_CMD_PANTILT_HOME] = COMMAND_INITIALIZER0(cmd_pantilt_home),
	[VISCA_CMD_PANTILT_RESET] = COMMAND_INITIALIZER0(cmd_pantilt_reset),
};

#define INQUIRY(...)  		HEAD_DIRECT(BYTE_INQUIRY, ##__VA_ARGS__)
#define INQ_INTERFACE(...)	INQUIRY(BYTE_INTERFACE, ##__VA_ARGS__)
#define INQ_CAMERA1(...) 	INQUIRY(BYTE_CAMERA1, ##__VA_ARGS__)
#define INQ_PANTILTER(...)	INQUIRY(BYTE_PANTILTER, ##__VA_ARGS__)

static byte_t inq_zoom_pos[] = INQ_CAMERA1(0x47);
static byte_t inq_version[] = INQ_INTERFACE(0x02);
static byte_t inq_pantilt_status[] = INQ_PANTILTER(0x10);

static struct command inqs[] = {
	[VISCA_INQ_ZOOM_POS] = INQUIRY_INITIALIZER(inq_zoom_pos, 2, parse_zoom_pos),
	[VISCA_INQ_VERSION] = INQUIRY_INITIALIZER(inq_version, 2, parse_version),
	[VISCA_INQ_PANTILT_STATUS] = INQUIRY_INITIALIZER(inq_pantilt_status, 2, parse_pantilt_status),
};

static int recv_packet(int fd, struct visca_packet *pack)
{
	int len = 0;

	while (1) {
		read(fd, pack->buf + len, 1);
		if (pack->buf[len] == TERMINATOR)
			break;
		len++;
	}
	pack->len = len + 1;
	pack->dir = DIR_RECV;

	debug_pack(pack);

	if (PACKET_TYPE(pack) == BYTE_ERROR) {
		int err_code = ERR_CODE(pack);
		print_err_msg(err_code);
		if (err_code != ERR_CMD_CANCELLED)
			return -1;			
	}
	return 0;
}

static int send_packet(int fd, struct visca_packet *pack) {
	int err;
	byte_t type;

	if (write(fd, pack->buf, pack->len) < pack->len) {
		warn("write packet to fd %d failed", fd);
		return -1;
	}
	
	debug_pack(pack);
	usleep(DELAY);

	if (PACKET_IS_BROADCAST(pack)) {
		if ((err = recv_packet(fd, pack)))
			return -1;
		if (!PACKET_IS_BROADCAST(pack)) {
			ldebug("'broadcast 0x88' expect 'broadcast 0x88' reply\n");
			return -1;
		}
		return 0;
	}

	type = PACKET_TYPE(pack);
	switch (type) {
	case BYTE_COMMAND:
		do {
			usleep(DELAY);
			if ((err = recv_packet(fd, pack)))
				return -1;
		} while(PACKET_TYPE(pack) == BYTE_ACK);
		
		if (PACKET_TYPE(pack) != BYTE_COMPLETION) {
			ldebug("'command' expect 'completion' reply\n");
			return -1;
		}
		break;
	case BYTE_INQUIRY:
		usleep(DELAY);
		if ((err = recv_packet(fd, pack)))
			return -1;
		if (PACKET_TYPE(pack) != BYTE_COMPLETION) {
			ldebug("'inquiry' expect one 'completion' reply\n");
			return -1;
		}
		break;
	default:
		debug_pack(pack);
		BUG();
	}
	return 0;
}

static int fill_const_packet(struct command *cmd, struct visca_packet *pack, int cam_addr) {
	int i;

	if (cam_addr > 8 || cam_addr < 1) {
		warnx("cam_addr %d invalid\n", cam_addr);
		return -1;
	}
	
	if (cmd->data[0] == BYTE_BROADCAST){
		pack->buf[0] = cmd->data[0];
	} else {
		BUG_ON(cmd->data[0] != BYTE_DIRECT);
		pack->buf[0] = cmd->data[0] | cam_addr;
	}
	
	BUG_ON(cmd->len + 1 > PACKET_BUF_SIZE);

	for (i = 1; i < cmd->len; i++)
		pack->buf[i] = cmd->data[i];
	pack->len = cmd->len;		
	pack->buf[pack->len++] = TERMINATOR;
	pack->dir = DIR_SEND;
	return 0;
}

int __visca_command(int fd, struct visca_packet *pack, int cmd_idx, int cam_addr, long para) {
	int err;
	struct command *cmd;
	
	BUG_ON(cmd_idx >= VISCA_CMD_MAX);
	cmd = cmds + cmd_idx;

	if ((err = fill_const_packet(cmd, pack, cam_addr)))
		return err;

	if (cmd->fill_packet) {
		BUG_ON(cmd->para_start < 0);
		if (cmd->fill_packet(cmd, pack, para))
			return -1;
	}

	if ((err = send_packet(fd, pack)))
		return err;

        /* fix visca pantilt_stop bug or error 
	 * 'command not executable' will occurs */
	if (cmd_idx == VISCA_CMD_PANTILT_STOP) {
		/* minimal time interval for stop is around 163ms(by test)
		 * for current baud rate (9600) 
		 * we apply 180ms here
		 */
		usleep(180000);
	}
	return err;
}

int __visca_inquiry(int fd, struct visca_packet *pack, int inq_idx, int cam_addr, void *ret)
{
	int err;
	struct command *inq = inqs + inq_idx;

	BUG_ON(inq->ret_start < 0 || !inq->parse_packet);

	if ((err = fill_const_packet(inq, pack, cam_addr)))
		return -1;

	if ((err = send_packet(fd, pack)))
		return err;

	BUG_ON(pack->dir != DIR_RECV);
	inq->parse_packet(inq, pack, ret);
	return 0;
}
