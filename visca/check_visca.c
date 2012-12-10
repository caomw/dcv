#include <unistd.h>
#include <check.h>

#include "visca.h"

int fd;
struct visca_interface iface = VISCA_IF_INIT;
struct visca_interface *pif = &iface;
char *dev_name = "/dev/ttyS0";

#define WAIT_INTERVAL  3
#define wait() do { SYS_BUG(sleep(WAIT_INTERVAL)); } while(0)

#define fail_unless0(cond)  fail_unless(cond, NULL)

START_TEST(test_trivial)
{
	int vendor, model, rom_version;
	int dz_mode;

	SYS_BUG(visca_inq_version(pif, &vendor, &model, &rom_version));
	fail_unless(vendor == 0x0001 
		    && model == 0x040e
		    && rom_version == 0x0115, "vendor 0x%04x"
		    " model 0x%04x rom_version 0x%04x\n",
		    vendor, model, rom_version);

	SYS_BUG(visca_inq_dzoom_mode(pif, &dz_mode));
	fail_unless0(dz_mode == false);

	SYS_BUG(visca_dzoom_mode(pif, true));
	SYS_BUG(visca_inq_dzoom_mode(pif, &dz_mode));
	fail_unless0(dz_mode == true);

	SYS_BUG(visca_dzoom_mode(pif, false));

}
END_TEST

/* START_TEST(test_exhausting) */
/* { */
/* 	int stat; */

/* 	SYS_BUG(visca_pantilt_reset(pif)); */
/* } */
/* END_TEST */

START_TEST(test_zoom)
{
	int zoom_pos;

	SYS_BUG(visca_zoom_wide(pif));
	SYS_BUG(visca_zoom_stop(pif));

	SYS_BUG(visca_zoom_tele(pif));
	SYS_BUG(visca_zoom_stop(pif));

	fail_unless0(visca_zoom_tele_speed(pif, 
					   VISCA_Z_SPEED_MAX + 1) < 0);
	fail_unless0(visca_zoom_tele_speed(pif, 
					   VISCA_Z_SPEED_MIN - 1) < 0);
	
	SYS_BUG(visca_zoom_tele_speed(pif, VISCA_Z_SPEED_MIN));
	SYS_BUG(visca_zoom_wide_speed(pif, VISCA_Z_SPEED_MAX));

	fail_unless0(visca_zoom_direct(pif, 
				       VISCA_Z_POS_MIN - 1) == VISCA_ERR);
	fail_unless0(visca_zoom_direct(pif, 
				       VISCA_Z_POS_MAX + 1) == VISCA_ERR);

	SYS_BUG(visca_zoom_direct(pif, VISCA_Z_POS_MIN));
	SYS_BUG(visca_inq_zoom_pos(pif, &zoom_pos));
	fail_unless0(zoom_pos == VISCA_Z_POS_MIN);

	SYS_BUG(visca_zoom_direct(pif, VISCA_Z_POS_MAX));
	SYS_BUG(visca_inq_zoom_pos(pif, &zoom_pos));
	fail_unless0(zoom_pos == VISCA_Z_POS_MAX);
}
END_TEST

START_TEST(test_pantilt_dir)
{
	int stat;
	int p_speed, t_speed;
	
	SYS_BUG(visca_pantilt_home(pif));

	SYS_BUG(visca_inq_pantilt_status(pif, &stat));	
	fail_unless(!(stat & STAT_P_DIR_MASK)
		    && !(stat & STAT_T_DIR_MASK)
		    && (stat & STAT_PT_CMD_MASK) == STAT_PT_CMD_COMPLETED
		    && !(stat & STAT_PT_ERR_MASK)
		    && (stat & STAT_PT_INIT_MASK) == STAT_PT_INITED
		    , "status 0x%04x", stat);
	
	/* only check max speed not speed in last command*/
	SYS_BUG(visca_inq_pantilt_maxspeed(pif, &p_speed, &t_speed));
	fail_unless(p_speed == VISCA_P_SPEED_MAX
		    && t_speed == VISCA_T_SPEED_MAX,
		    "(p_speed,t_speed) (%d,%d) != (%d,%d)", 
		    p_speed, t_speed, VISCA_P_SPEED_MAX, VISCA_T_SPEED_MIN);

	fail_unless0(visca_pantilt_dir(pif,
				       VISCA_P_SPEED_MIN - 1,
				       1, 1) == VISCA_ERR);
	fail_unless0(visca_pantilt_dir(pif,
				       VISCA_P_SPEED_MAX + 1,
				       1, 1) == VISCA_ERR);
	fail_unless0(visca_pantilt_dir(pif, 1,
				       VISCA_T_SPEED_MIN - 1,
				       1) == VISCA_ERR);
	fail_unless0(visca_pantilt_dir(pif, 1,
				       VISCA_T_SPEED_MAX + 1,
				       1) == VISCA_ERR);

	SYS_BUG(visca_pantilt_dir(pif, VISCA_P_SPEED_MAX/2,
				  VISCA_T_SPEED_MAX/2,
				  PT_UPLEFT));
	
	SYS_BUG(visca_pantilt_stop(pif));

	SYS_BUG(visca_pantilt_dir(pif, VISCA_P_SPEED_MAX,
				  VISCA_T_SPEED_MAX,
				  PT_DOWNRIGHT));
	/* note STAT_PT_DIR only check whether current pos is at the
	 * corrsponding extreme postion, we need wait enough time for
	 * the position.
	 */
	wait();
	SYS_BUG(visca_pantilt_stop(pif));
	SYS_BUG(visca_inq_pantilt_status(pif, &stat));
	fail_unless((stat & STAT_P_DIR_MASK) == STAT_P_DIR_RIGHT
		&& (stat & STAT_T_DIR_MASK) == STAT_T_DIR_DOWN
		&& (stat & STAT_PT_CMD_MASK) == STAT_PT_CMD_COMPLETED
		&& !(stat & STAT_PT_ERR_MASK)
		&& (stat & STAT_PT_INIT_MASK) == STAT_PT_INITED
		, "status 0x%04x", stat);
}
END_TEST

START_TEST(test_pantilt_pos)
{	
	int p_pos, t_pos;

	fail_unless0(visca_pantilt_absolute_pos(
			     pif, VISCA_P_SPEED_MAX, 
			     VISCA_T_SPEED_MAX, 
			     VISCA_P_POS_MIN - 1, 0) == VISCA_ERR);
	fail_unless0(visca_pantilt_absolute_pos(
			     pif, VISCA_P_SPEED_MAX, 
			     VISCA_T_SPEED_MAX, 
			     VISCA_P_POS_MAX + 1, 0) == VISCA_ERR);
	fail_unless0(visca_pantilt_absolute_pos(
			     pif, VISCA_P_SPEED_MAX, 
			     VISCA_T_SPEED_MAX, 
			     0, VISCA_T_POS_MIN - 1) == VISCA_ERR);
	fail_unless0(visca_pantilt_absolute_pos(
			     pif, VISCA_P_SPEED_MAX, 
			     VISCA_T_SPEED_MAX, 
			     0, VISCA_T_POS_MAX + 1) == VISCA_ERR);
	
	SYS_BUG(visca_pantilt_absolute_pos(pif, VISCA_P_SPEED_MAX, 
					   VISCA_T_SPEED_MAX, 
					   VISCA_P_POS_MAX, 
					   VISCA_T_POS_MAX));
	SYS_BUG(visca_pantilt_absolute_pos(pif, VISCA_P_SPEED_MAX, 
					   VISCA_T_SPEED_MAX, 
					   VISCA_P_POS_MIN, 
					   VISCA_T_POS_MIN));
	SYS_BUG(visca_pantilt_relative_pos(pif, VISCA_P_SPEED_MAX, 
					   VISCA_T_SPEED_MAX, 
					   -VISCA_P_POS_MIN, 
					   -VISCA_T_POS_MIN));
	SYS_BUG(visca_inq_pantilt_pos(pif, &p_pos, &t_pos));
	fail_unless(p_pos == 0 && t_pos == 0,
		    "(p_pos,t_pos) (%d, %d)", p_pos, t_pos);	
}
END_TEST

static Suite * visca_suite(void)
{
	Suite *s = suite_create("visca");
	TCase *tc_core = tcase_create("Core");

	tcase_set_timeout(tc_core, 60);
	tcase_add_test(tc_core, test_trivial);
	tcase_add_test(tc_core, test_zoom);
	tcase_add_test(tc_core, test_pantilt_dir);
	tcase_add_test(tc_core, test_pantilt_pos);
	/* tcase_add_test(tc_core, test_exhausting); */
	suite_add_tcase(s, tc_core);
	return s;
}

int main(int argc, char **argv)
{
	Suite *s = visca_suite();
	SRunner *sr = srunner_create(s);	
	int number_failed;

	SYS_BUG(visca_open_serial(pif, dev_name));
	SYS_BUG(visca_set_address(pif));
	SYS_BUG(visca_clear_if(pif));
	SYS_BUG(visca_dzoom_mode(pif, false));
	
	srunner_run_all(sr, CK_NORMAL);
	number_failed = srunner_ntests_failed(sr);
	srunner_free(sr);

	SYS_BUG(visca_close_serial(pif));

	return -number_failed;
}
