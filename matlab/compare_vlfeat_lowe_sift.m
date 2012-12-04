function compare_vlfeat_lowe_sift

global dcv_root

filename1 = 'scene';
filename2 = 'box';
fileext = '.pgm';
filepath = [dcv_root '/data'];

I1 = imread([filepath '/' filename1 fileext]);
[lowe_f1, lowe_d1] = d_lowe_sift(I1, filename1);
[vl_f1, vl_d1] = d_vl_sift(I1);

I2 = imread([filepath '/' filename2 fileext]);
[lowe_f2, lowe_d2] = d_lowe_sift(I2, filename2);
[vl_f2, vl_d2] = d_vl_sift(I2);

lowe_matches = d_ubcmatch(lowe_d1, lowe_d2);
vl_matches = d_ubcmatch(vl_d1, vl_d2);

show_im = appendimages(I1, I2);
offset = [size(I1, 2); 0];
figure(1); clf; colormap gray; axis image;
imagesc(show_im);
set(gca,'dataAspectRatio',[1 1 1]);

h1 = d_plot_sift_match(vl_f1, vl_f2, offset, vl_matches); 
set([h1{1};h1{2};h1{3}],'color','b','linewidth',2);

h2 = d_plot_sift_match(lowe_f1, lowe_f2, offset, lowe_matches);
set([h2{1};h2{2};h2{3}],'color','g','linewidth',1);
% legend([h2;h5], 'vlfeat sift', 'lowe sift', 'Location', 'SouthEast');
