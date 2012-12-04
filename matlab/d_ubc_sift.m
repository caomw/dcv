function [f, d] = d_ubc_sift(I, cache_path)
global dcv_root

if ismatrix(I)
    I1 = I;
elseif ndims(I) == 3
    I1 = rgb2gray(I);
end

if isempty(strfind(cache_path, dcv_root))
    cache_path = [dcv_root '/data/cache/' cache_path];
end

dir = fileparts(cache_path);
if ~exist(dir, 'dir')
    mkdir(dir);
end

pgmfile = [cache_path '.pgm'];
if ~exist(pgmfile, 'file')
    imwrite(I1, pgmfile, 'pgm');
end

keyfile = [cache_path '.sift'];
if ~exist(keyfile, 'file')
    eval(['!' dcv_root '/contrib/siftDemoV4/sift <', pgmfile, ' >', keyfile]);
end

[f, d] = vl_ubcread(keyfile);
