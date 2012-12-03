function compare_vlfeat_lowe_sift

filenames = {};
filenames{1} = 'scene';
filenames{2} = 'box';
fileext = '.pgm';
rootdir = '..';
filepath = [rootdir '/data'];

lowe_f = cell(2);
lowe_d = cell(2);
vl_f = cell(2);
vl_d = cell(2);
gray_img = cell(2);

for i = 1:length(filenames)
    filename = filenames{i};
    I0 = imread([filepath '/' filename fileext]);
    if ismatrix(I0)
        gray_img{i} = I0;
    elseif ndims(I0) == 3
        gray_img{i} = rgb2gray(I0);
    end

    pgmfile = [rootdir '/cache/' filename, '.pgm'];
    if ~exist(pgmfile, 'file')
        imwrite(gray_img{i}, pgmfile, 'pgm');
    end

    keyfile = [rootdir '/cache/' filename, '.sift'];
    if ~exist(keyfile, 'file')
        eval(['!' rootdir '/dropins/siftDemoV4/sift <', pgmfile, ' >', keyfile]);
    end

    [lowe_f{i}, lowe_d{i}] = vl_ubcread(keyfile);
    I = single(gray_img{i});
    [vl_f{i}, vl_d{i}] = vl_sift(I, 'FirstOctave', -1, 'peakthresh', 3.4);
end

lowe_matches = vl_ubcmatch(lowe_d{1}, lowe_d{2}, 2.7);
vl_matches = vl_ubcmatch(vl_d{1}, vl_d{2}, 2.7);

show_im = appendimages(gray_img{1}, gray_img{2});
offset = [size(gray_img{1}, 2); 0];

figure(1); clf; colormap gray; axis image;
imagesc(show_im);
set(gca,'dataAspectRatio',[1 1 1]);

h1 = vl_plotframe(vl_f{1});
f2 = vl_f{2};
f2(1,:) = f2(1,:) + offset(1);
f2(2,:) = f2(2,:) + offset(2);

h2 = vl_plotframe(f2);
h3 = d_plot_match(vl_f{1}, vl_f{2}, offset, vl_matches); 
set([h1;h2;h3],'color','b','linewidth',2);

h4 = vl_plotframe(lowe_f{1});
f2 = lowe_f{2};
f2(1,:) = f2(1,:) + offset(1);
f2(2,:) = f2(2,:) + offset(2);
h5 = vl_plotframe(f2);
h6 = d_plot_match(lowe_f{1}, lowe_f{2}, offset, lowe_matches);
set([h4;h5;h6],'color','g','linewidth',1);
legend([h1,h4], 'vlfeat sift', 'lowe sift', 'Location', 'SouthEast');
