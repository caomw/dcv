function interpolate_zoom
global dcv_root

opt.plot = 1;
opt.save = 0;

% match sift scale
k = 2^0.5;

zoom_tab = load([dcv_root '/data/ptz_zoom_tab.txt'])';
X = zoom_tab(1,:);
Y = zoom_tab(2,:);
pp = spline(X, Y);

max_zoom = max(zoom_tab(1,:));
tmp = log(max_zoom)/log(k);
levels = floor(tmp);

xx = k.^(0:levels);
if tmp > levels
    xx(end+1) = k^tmp;
end
yy = fix(ppval(pp, xx));

if opt.save
    fp = fopen([dcv_root '/data/ptz_zoom_interpolated.txt'], 'w');
    fprintf(fp, '%.4f\t%d\n', [xx; yy]);
    fclose(fp);
end

if opt.plot
    xx1 = 1:.1:max_zoom;
    plot(xx, yy, 'o', xx1, ppval(pp, xx1));
end