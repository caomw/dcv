/*
 * see:
 * [1] Z.Kalal forward backward error: automatic detection of 
 *	tracking failures
 */
#include <stdlib.h>
#include <math.h>
#include <limits.h>

#include <opencv2/core/core_c.h>
#include <opencv2/highgui/highgui_c.h>

#include "tld.h"

static int alloc_bbgrid(struct bbgrid *bbg, CvSize size)
{
	bbg->pts = (CvPoint2D32f *) malloc(size.width * size.height
					   * sizeof(CvPoint2D32f));
	if (!bbg->pts)
		return ERR;
	bbg->size = size;
	return 0;
}

static void free_bbgrid(struct bbgrid *bbg)
{
	free(bbg->pts);
}
static void init_bbgrid(struct bbgrid *bbg, int margin)
{
	bbg->margin = margin;
}
static void gen_bbgrid(struct rect *rect, struct bbgrid *bbg)
{
	float x0 = rect->x0 + bbg->margin;
	float y0 = rect->y0 + bbg->margin;
	float x1 = rect->x1 -  bbg->margin;
	float y1 = rect->y1 - bbg->margin;
	int w = bbg->size.width, h = bbg->size.height;
	float w_step, h_step;
	CvPoint2D32f *tmp;
	int i, j;

	BUG_ON(w < 1 && h < 1);	
	w_step = (x1 - x0)/ (w - 1);
	h_step = (y1 - y0)/ (h - 1);

	tmp = bbg->pts;
	for (j = 0; j < w; j++) {
		for (i = 0; i < h; i++) {
			tmp->x = x0 + j * w_step;
			tmp->y = y0 + i * h_step;
			tmp++;
		}
	}	
}

static int alloc_lktracker(struct lktracker *lkt, int pt_size,
			   CvSize img_size)
{
	lkt->prev_pyr = cvCreateImage(img_size, 8, 1);
	if (!lkt->prev_pyr)
		return ERR;
	lkt->pyr = cvCreateImage(img_size, 8, 1);
	if (!lkt->pyr)
		goto free_prev_pyr;	
	lkt->pts = (CvPoint2D32f *)malloc(pt_size 
					      * sizeof(CvPoint2D32f));
	if (!lkt->pts)
		goto free_pyr;
	lkt->fb_pts = (CvPoint2D32f *)malloc(pt_size 
						  * sizeof(CvPoint2D32f));
	if (!lkt->fb_pts)
		goto free_pts;	
	lkt->status = (char *) malloc(pt_size * sizeof(char));
	if (!lkt->status)
		goto free_fb_pts;
	lkt->fb_status = (char *) malloc(pt_size * sizeof(char));
	if (!lkt->fb_status)
		goto free_status;

	lkt->pt_size = pt_size;
	lkt->img_size = img_size;
	return 0;

free_status:
	free(lkt->status);
free_fb_pts:
	free(lkt->fb_pts);
free_pts:
	free(lkt->pts);
free_pyr:
	cvReleaseImage(&lkt->pyr);
free_prev_pyr:
	cvReleaseImage(&lkt->prev_pyr);
	return ERR;
}

static void free_lktracker(struct lktracker *lkt)
{
	free(lkt->fb_status);
	free(lkt->status);
	free(lkt->fb_pts);
	free(lkt->pts);
	cvReleaseImage(&lkt->pyr);
	cvReleaseImage(&lkt->prev_pyr);
}

static void init_lktracker(struct lktracker *lkt, int level, 
			   CvSize win_size, CvTermCriteria term_crit)
{
	lkt->level = level;
	lkt->win_size = win_size;
	lkt->term_crit = term_crit;
}

static int alloc_ptmatcher(struct ptmatcher *ptm, int score_size, 
			   CvSize win_size)
{
	ptm->res = cvCreateImage(cvSize(1, 1), IPL_DEPTH_32F, 1);
	if (!ptm->res)
		return ERR;

	ptm->rec0 = cvCreateImage(win_size, 8, 1);
	if (!ptm->rec0) {
		goto free_res;
	}
	ptm->rec1 = cvCreateImage(win_size, 8, 1);
	if (!ptm->rec1)
		goto free_rec0;

	ptm->scores = (struct elem*) malloc(score_size * sizeof(struct elem));
	if (!ptm->scores)
		goto free_rec1;	
	ptm->score_size = score_size;

	ptm->win_size = win_size;
	return 0;

free_rec1:
	cvReleaseImage(&ptm->rec1);
free_rec0:
	cvReleaseImage(&ptm->rec0);
free_res:
	cvReleaseImage(&ptm->res);
	return ERR;
}

static void free_ptmatcher(struct ptmatcher *ptm)
{
	free(ptm->scores);
	cvReleaseImage(&ptm->rec1);
	cvReleaseImage(&ptm->rec0);
	cvReleaseImage(&ptm->res);
}

static void init_ptmatcher(struct ptmatcher *ptm, int ptm_method)
{
	ptm->method = ptm_method;
}

static int alloc_fberror(struct fberror *fbe, int size)
{
	fbe->errs = (struct elem *)malloc(size * sizeof(struct elem));
	if (!fbe->errs)
		return ERR;
	fbe->err_size = size;
	return 0;
}
static void free_fberror(struct fberror *fbe)
{
	free(fbe->errs);
}
static void init_fberror(struct fberror *fbe, float th)
{
	fbe->th = th;
}
int alloc_fbtracker(struct fbtracker *fbt, CvSize img_size,
			   CvSize bbg_size, CvSize ptm_win_size)
{
	int error;
	int pt_size = bbg_size.width * bbg_size.height;
	
	if ((error = alloc_bbgrid(&fbt->bbg, bbg_size)))
		return error;

	if ((error = alloc_lktracker(&fbt->lkt, pt_size, img_size)))
		goto free_bbgrid;

	if ((error = alloc_ptmatcher(&fbt->ptm, pt_size, ptm_win_size)))
		goto free_lktracker;
	
	if ((error = alloc_fberror(&fbt->fbe, pt_size)))
		goto free_ptmatcher;
	
	fbt->idx_cache = (int *)malloc(pt_size * sizeof(int));
	if (!fbt->idx_cache)
		goto free_fberror;
	fbt->idx_cache_size = pt_size;

	fbt->median_cache = (float *)malloc(pt_size * pt_size * sizeof(float));
	if (!fbt->median_cache)
		goto free_idx_cache;
	fbt->median_cache_size = pt_size * pt_size;

	fbt->pt_size = pt_size;
	return 0;

free_idx_cache:
	free(fbt->idx_cache);
free_fberror:
	free_fberror(&fbt->fbe);
free_ptmatcher:
	free_ptmatcher(&fbt->ptm);
free_lktracker:
	free_lktracker(&fbt->lkt);
free_bbgrid:
	free_bbgrid(&fbt->bbg);	
	return error;
}

void free_fbtracker(struct fbtracker *fbt)
{
	free(fbt->median_cache);
	free(fbt->idx_cache);
	free_fberror(&fbt->fbe);
	free_ptmatcher(&fbt->ptm);
	free_lktracker(&fbt->lkt);
	free_bbgrid(&fbt->bbg);	
}

void init_fbtracker(struct fbtracker *fbt, 
		    CvSize bb_nr, int bb_margin,
		    int lk_level, CvSize lk_win_size,
		    CvTermCriteria lk_term_crit, int ptm_method,
		    int fb_th)
{
	init_bbgrid(&fbt->bbg, bb_margin);
	init_lktracker(&fbt->lkt, lk_level, lk_win_size, lk_term_crit);
	init_ptmatcher(&fbt->ptm, ptm_method);
	init_fberror(&fbt->fbe, fb_th);
	fbt->ready = 0;
}

static void lktrack(struct lktracker *lkt, IplImage *img, 
		    CvPoint2D32f *pts0, int pt_nr)
{
	int i;

	BUG_ON(pt_nr <= 0);
	for (i = 0; i < pt_nr; i++) {
		lkt->pts[i] = pts0[i];
		lkt->fb_pts[i] = pts0[i];
	}
	
	/* status must be alloced, or error will occured 
	 * seems CV_LKFLOW_* and *_pyr didnt used in opencv
	 */
	cvCalcOpticalFlowPyrLK(lkt->prev_img, img, lkt->prev_pyr, lkt->pyr, 
			       pts0, lkt->pts, pt_nr,
			       lkt->win_size, lkt->level, lkt->status, 0, 
			       lkt->term_crit, 
			       CV_LKFLOW_INITIAL_GUESSES);

	cvCalcOpticalFlowPyrLK(img, lkt->prev_img, lkt->pyr, lkt->prev_pyr,
			       lkt->pts, lkt->fb_pts, pt_nr,
			       lkt->win_size, lkt->level, lkt->fb_status, 
			       0, lkt->term_crit,
			       CV_LKFLOW_INITIAL_GUESSES 
			       | CV_LKFLOW_PYR_A_READY 
			       | CV_LKFLOW_PYR_B_READY);
	
	for (i = 0; i < pt_nr; i++) {
		if (!lkt->status[i]) {
			lkt->pts[i] = cvPoint2D32f(-1.0, -1.0);
			if (lkt->fb_status[i] == 1)
				lkt->fb_status[i] = 0;
		}
		if (!lkt->fb_status[i])
			lkt->fb_pts[i] = cvPoint2D32f(-1.0, -1.0);
	}
	
	/* step next */
	lkt->prev_img = img;
}

#define MAGNIFY	1024
static int cmp_elem(const void *a, const void *b) 
{
	return (int)((((struct elem *)a)->val - ((struct elem *)b)->val) 
		     * MAGNIFY);
}
static int cmp_float(const void *a, const void *b)
{
	return (int)((*(float *)b - *(float *)a) * MAGNIFY);
}

#define DEFINE_QSORT(name, type)	\
	static inline void qsort_##name(type *arr, int nr) \
	{ \
		qsort(arr, nr, sizeof(type), cmp_##name); \
	}
DEFINE_QSORT(float, float)
DEFINE_QSORT(elem, struct elem)

static int cmp_elem_dec(const void *a, const void *b) 
{
	return (int) ((((struct elem *)b)->val - ((struct elem *)a)->val) 
		      * MAGNIFY);
}
#define DEFINE_QSORT_DEC(name, type) DEFINE_QSORT(name##_dec, type)
DEFINE_QSORT_DEC(elem, struct elem)

static inline float get_median_elem(struct elem *arr, int nr) 
{
	return (arr[nr / 2].val + arr[(nr - 1) / 2].val)/2;
}
#define get_median(arr, nr) ({\
	((arr)[(nr) / 2] + (arr)[((nr) - 1) / 2]) / 2;	\
})

static int __init_idx(struct elem *data, int nr, 
		       int *idxes, int *idx_nr, float thresh)
{
	int i;

	for (i = 0; i < nr && data[i].val <= thresh; i++)
		idxes[i] = data[i].idx;	
	if (!i) 
		return ERR;

	*idx_nr = i;
	return 0;
}

static void __idx_filter_dec(struct elem *data, int nr,
			     int *idxes,  int *idx_nr, float thresh)
{
	int i,j,k = 0;

	BUG_ON(!(*idx_nr));
	for (i = 0; i < *idx_nr; i++) {
		int found = 0;
		int idx = idxes[i];
		
		for (j = 0; j < nr && data[j].val >= thresh; j++) {
			if (data[j].idx == idx) {
				found = 1;
				break;
			}
		}
		if (found) {
			if (k < i)
				idxes[k] = idx;
			k++;
		}
	}
	*idx_nr = k;
}

static void ptmatch_filter(struct ptmatcher *ptm, 
			   IplImage *img0, IplImage *img1,
			   CvPoint2D32f *pts0, CvPoint2D32f *pts1, 
			   int pt_size, char *status, 
			   int *idxes, int *idx_nr) 
{
	int i;
	struct elem *tmp = ptm->scores;
	int score_nr = 0;
	float score_median;

	for (i = 0; i < pt_size; i++) {
		if (!status[i])
			continue;

		cvGetRectSubPix(img0, ptm->rec0, pts0[i]);
		cvGetRectSubPix(img1, ptm->rec1, pts1[i]);
		cvMatchTemplate(ptm->rec0, ptm->rec1, ptm->res, ptm->method);

		tmp->idx = i;
		tmp->val = ((float *) (ptm->res->imageData))[0];
		tmp++;
		score_nr++;
	}
	
	qsort_elem_dec(ptm->scores, score_nr);
	score_median = get_median_elem(ptm->scores, score_nr);
	__idx_filter_dec(ptm->scores, score_nr, 
			 idxes, idx_nr, score_median);
}

static inline float pt_dist(CvPoint2D32f pt0, CvPoint2D32f pt1)
{
	return sqrt((pt0.x - pt1.x) * (pt0.x - pt1.x)
		    + (pt0.y - pt1.y) * (pt0.y - pt1.y));
}

static inline int fberror_filter(struct fberror *fbe, 
				 CvPoint2D32f *pts0, CvPoint2D32f *pts, 
				 int pt_size, char *status, 
				 int *idxes, int *idx_nr)
{
	int i;
	struct elem *tmp = fbe->errs;
	int err_nr = 0;
	float err_median;
	
	for (i = 0; i < pt_size; i++) {
		if (!status[i])			
			continue;

		tmp->idx = i;
		tmp->val = pt_dist(pts0[i], pts[i]);
		tmp++;
		err_nr++;
	}
	if (!err_nr)
		return ERR;

	qsort_elem(fbe->errs, err_nr);
	err_median = get_median_elem(fbe->errs, err_nr);	
	if (err_median > fbe->th)
		return ERR;
	
	return __init_idx(fbe->errs, err_nr,
			  idxes, idx_nr, err_median);
}

static inline float sort_get_media_float(float *arr, int nr)
{
	qsort_float(arr, nr);
	return get_median(arr, nr);
}

static void predict_bb(CvPoint2D32f *pts0, CvPoint2D32f *pts1,
		       int *idxes, int idx_nr, struct rect *bb,
		       struct rect *out_bb, float *median_cache)
{
	int i, j;
	float dx, dy, s, sx, sy;
	int k = idx_nr;

	for (i = 0; i < idx_nr; i++)
		median_cache[i] = pts1[idxes[i]].x 
			- pts0[idxes[i]].x;	
	dx = sort_get_media_float(median_cache, idx_nr);

	for (i = 0; i < idx_nr; i++)
		median_cache[i] = pts1[idxes[i]].y 
			- pts0[idxes[i]].y;
	dy = sort_get_media_float(median_cache, idx_nr);
	
	k = 0;
	for (i = 0; i < idx_nr; i++)
		for (j = i + 1; j < idx_nr; j++)
			median_cache[k++] = pt_dist(pts1[idxes[i]],
						       pts1[idxes[j]])
				/ pt_dist(pts0[idxes[i]],
					  pts0[idxes[j]]);
	s = sort_get_media_float(median_cache, k);
	
	sx = 0.5 * (s - 1) * (bb->x1 - bb->x0 + 1);
	sy = 0.5 * (s - 1) * (bb->y1 - bb->y0 + 1);
	
	out_bb->x0 = bb->x0 - sx + dx;
	out_bb->y0 = bb->y0 - sy + dy;
	out_bb->x1 = bb->x1 + sx + dx;
	out_bb->y1 = bb->y1 + sy + dy;
}

int fbtrack(struct fbtracker *fbt, IplImage *img, 
	    struct rect *bb, struct rect *out_bb)
{
	int error;
	int pt_size = fbt->pt_size;
	int idx_nr = 0;
	struct bbgrid *bbg = &fbt->bbg;
	struct lktracker *lkt = &fbt->lkt;
	struct ptmatcher *ptm = &fbt->ptm;
	struct fberror *fbe = &fbt->fbe;
	
	if (!fbt->ready) {
		pr_warn("fbtracker isn't ready\n");
		return ERR;
	}

	gen_bbgrid(bb, bbg);
	
	lktrack(lkt, img, bbg->pts, pt_size);
	
	if ((error = fberror_filter(fbe, bbg->pts, lkt->fb_pts, pt_size,
				    lkt->fb_status, fbt->idx_cache, 
				    &idx_nr)))
		return error;

	ptmatch_filter(ptm, lkt->prev_img, img, bbg->pts, lkt->pts, pt_size,
		       lkt->status, fbt->idx_cache, &idx_nr);

	predict_bb(bbg->pts, lkt->pts,
		   fbt->idx_cache, idx_nr,  bb, out_bb,
		   fbt->median_cache);
	
	if (out_bb->x0 < 1 || out_bb->y0 < 1
	    || out_bb->x1 > lkt->img_size.width - 1
	    || out_bb->y1 > lkt->img_size.height - 1)
		return ERR;

	return 0;
}
