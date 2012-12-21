#ifndef __TLD_H__
#define __TLD_H__

#include <stdlib.h>
#include <opencv2/core/types_c.h>
#include <opencv2/video/tracking.hpp>
#include "common.h"

struct elem {
	float val;	
	int idx;
};

struct rect {
	float x0;
	float y0;
	float x1;
	float y1;
};

#define RECT_INIT(_x0,_y0,_x1,_y1) {		\
	.x0 = _x0,				\
	.y0 = _y0, \
	.x1 = _x1, \
	.y1 = _y1,		\
}

#ifdef DEBUG
#define dbg_pts(pts, nr) do {			\
		int i;				\
						\
		dbgl("points:\n");		\
		for (i = 0; i < (nr); i++)				\
			dbg("%d: (%.4f,%.4f)\n", i+1, (pts)[i].x, (pts)[i].y); \
	} while(0)
#define dbg_arri(arr, nr) do { \
		int i; \
		\
		dbgl("arri:\n"); \
		for (i = 0; i < (nr); i++)		\
			dbg("%d: %d\n", i+1, (arr)[i]);	\
	} while(0)
#define dbg_arrf(arr, nr) do {\
		int i;		      \
		\
		dbgl("arrf:\n"); \
		for (i = 0; i < (nr); i++)		\
			dbg("%d: %.4f\n", i+1, (arr)[i]);	\
	} while(0)
#define dbg_arr_elem(arr, nr) do {	      \
		int i;		      \
		\
		dbgl("arr_elem:\n"); \
		for (i = 0; i < (nr); i++)		    \
			dbg("%d: %i, %.4f\n", i+1, (arr)[i].idx, \
			    (arr)[i].val);			 \
	} while(0)
#define dbg_rect(rect) do { \
		dbgl("rect:\n"); \
		dbg("x0,y0,x1,y1: %.4f,%.4f,%.4f,%.4f\n", \
		    (rect)->x0,(rect)->y0,(rect)->x1,(rect)->y1); \
	} while(0)
#else
#define dbg_pts(pts, nr)
#define dbg_arri(arr, nr)
#define dbg_arrf(arr, nr)
#define dbg_arr_item(arr, nr)
#define dbg_rect(rect)
#endif

/* forward-backward tracker */
/* 	bbgrid */
#define FBT_BBG_SIZE_W		10
#define FBT_BBG_SIZE_H		10
#define FBT_BBG_MARGIN	5
/* 	lucas-kanade tracker */
#define FBT_LKT_LEVEL		5
#define FBT_LKT_WIN_W		4
#define FBT_LKT_WIN_H		4
#define FBT_LKT_TERMCRIT_ITER	20
#define FBT_LKT_TERMCRIT_EPS    .03
/* 	point matcher */
#define FBT_PTM_WIN_W	10
#define FBT_PTM_WIN_H	10
#define FBT_PTM_METHOD	CV_TM_CCOEFF_NORMED
/*	forward-backward error thresh*/
#define FBT_FBE_TH	10


/* bounding box grid points */
struct bbgrid {	
	int margin;
	CvSize size;
	CvPoint2D32f *pts;
};

struct lktracker {
	int level;
	CvSize win_size;
	CvTermCriteria term_crit;

	CvSize img_size;
	IplImage *prev_img;
	IplImage *prev_pyr;
	IplImage *pyr;

	int pt_size;
	CvPoint2D32f *pts;
	CvPoint2D32f *fb_pts;
	char *status;
	char *fb_status;
};

struct ptmatcher {
	CvSize win_size;
	int method;

	IplImage *res;
	IplImage *rec0;
	IplImage *rec1;	

	int score_size;
	struct elem *scores;
};

/* forward backward error */
struct fberror {
	float th; /* thresh */
	int err_size;
	struct elem *errs;
};

struct fbtracker {
	int ready;
	int pt_size;
	struct bbgrid bbg;
	struct lktracker lkt;
	struct ptmatcher ptm;
	struct fberror fbe;
	
	int idx_cache_size;
	int *idx_cache;
	int median_cache_size;
	float *median_cache;
};

int alloc_fbtracker(struct fbtracker *fbt, CvSize img_size,
		    CvSize bbg_size, CvSize ptm_win_size);

void init_fbtracker(struct fbtracker *fbt, 
		    CvSize bb_nr, int bb_margin,
		    int lk_level, CvSize lk_win_size,
		    CvTermCriteria lk_term_crit, int ptm_method,
		    int fb_th);

int def_alloc_init_fbtracker(struct fbtracker *fbt, CvSize img_size)
{
	int error;
	CvSize bbg_size = cvSize(FBT_BBG_SIZE_W, FBT_BBG_SIZE_H);

	if ((error = alloc_fbtracker(fbt, img_size, bbg_size,
				     cvSize(FBT_PTM_WIN_W, FBT_PTM_WIN_H))))
		return error;

	init_fbtracker(fbt, bbg_size, 
		       FBT_BBG_MARGIN, FBT_LKT_LEVEL, 
		       cvSize(FBT_LKT_WIN_W, FBT_LKT_WIN_H),
		       	cvTermCriteria(CV_TERMCRIT_ITER	 | CV_TERMCRIT_EPS, 
				       FBT_LKT_TERMCRIT_ITER, 
				       FBT_LKT_TERMCRIT_EPS), 
		       FBT_PTM_METHOD, FBT_FBE_TH);
	return 0;
}

void free_fbtracker(struct fbtracker *fbt);

void prepare_fbtrack(struct fbtracker *fbt, IplImage *img)
{	
	fbt->lkt.prev_img = img;
	fbt->ready = 1;
}
int fbtrack(struct fbtracker *fbt,IplImage *img,
	    struct rect *bb, struct rect *out_bb);

#endif
