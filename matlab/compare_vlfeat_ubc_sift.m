function compare_vlfeat_ubc_sift

global dcv_root

filename1 = 'scene';
filename2 = 'box';
fileext = '.pgm';
filepath = [dcv_root '/data'];

I1 = imread([filepath '/' filename1 fileext]);
[ubc_f1, ubc_d1] = d_ubc_sift(I1, filename1);
[vl_f1, vl_d1] = d_vl_sift(I1);

I2 = imread([filepath '/' filename2 fileext]);
[ubc_f2, ubc_d2] = d_ubc_sift(I2, filename2);
[vl_f2, vl_d2] = d_vl_sift(I2);

ubc_matches = d_ubc_match(ubc_d1, ubc_d2);
vl_matches = d_ubc_match(vl_d1, vl_d2);

show_im = appendimages(I1, I2);
offset = [size(I1, 2); 0];
figure(1); clf; colormap gray; axis image;
imagesc(show_im);
set(gca,'dataAspectRatio',[1 1 1]);

[h1,h2,h3] = d_plot_sift_match(vl_f1, vl_f2, offset, vl_matches); 
set([h1;h2;h3],'color','b','linewidth',2);

[h1,h2,h3] = d_plot_sift_match(ubc_f1, ubc_f2, offset, ubc_matches);
set([h1;h2;h3],'color','g','linewidth',1);
