#include <stdlib.h>
#include <unistd.h>
#include <opencv2/highgui/highgui_c.h>
#include "tld.h"

int main(int argc, char **argv) 
{
	int opt;
	char *file_pattern = NULL;
	char filename[NAME_SIZE];
	int start_idx = -1, end_idx = -1;
	int idx;

	struct fbtracker fbt;
	IplImage *img0, *img1;
	struct rect bb0 = RECT_INIT(-1,-1,-1,-1);
	struct rect bb1;
	int track_failed = 0;

	while((opt = getopt(argc, argv, "i:s:e:b:")) != -1) {
		switch(opt) {
		case 'i':
			file_pattern = optarg;
			break;
		case 's':
			start_idx = atoi(optarg);
			break;
		case 'e':
			end_idx = atoi(optarg);
			break;
		case 'b':
			sscanf(optarg, "%f,%f,%f,%f", 
			       &bb0.x0, &bb0.y0, &bb0.x1, &bb0.y1);
			break;
		default:
			pr_err("\n-i input file pattern\n"
			    "-s start offset\n"
			    "-e end offset\n");
			return ERR;
		}
	}
	
	if (!file_pattern) {
		pr_err("input file pattern needed\n");
		return ERR;
	}
	
	if (start_idx == -1 || end_idx <= start_idx) {
		pr_err("invalid start_idx %d end_idx %d", 
		       start_idx, end_idx);
		return ERR;
	}
	
	if (bb0.x0 < 0) {
		pr_err("invalid bouding box(%.2f,%.2f,%.2f,%.2f)",
		       bb0.x0, bb0.x1, bb0.y0, bb0.y1);
		return ERR;
	}
	
	idx = start_idx;
	sprintf(filename, file_pattern, idx);	
	img0 = cvLoadImage(filename, CV_LOAD_IMAGE_GRAYSCALE);
	idx++;

        BUG_ON(def_alloc_init_fbtracker(&fbt, cvGetSize(img0)));
	prepare_fbtrack(&fbt, img0);

	for (; idx <= end_idx; idx++) {
		if (idx % 50 == 0)
			dbg("F:%d\n", idx);
		sprintf(filename, file_pattern, idx);
		img1 = cvLoadImage(filename, CV_LOAD_IMAGE_GRAYSCALE);
		if (!img1) {
			pr_warn("file %s doesnt exist!\n", filename);
			break;
		}			
		if (!(track_failed = fbtrack(&fbt, img1, &bb0, &bb1)))
			printf("%.2f,%.2f,%.2f,%.2f\n", 
			       bb1.x0, bb1.x1, bb1.y0, bb1.y1);
		else
			printf("NaN,NaN,NaN,NaN\n");

		cvReleaseImage(&img0);
		img0 = img1;

		if (!track_failed)
			bb0 = bb1;
	}
	
	free_fbtracker(&fbt);
	cvReleaseImage(&img0);
	return 0;
}
