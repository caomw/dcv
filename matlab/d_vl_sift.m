function [f, d] = d_vl_sift(I, first_octave, peak_thresh)

if nargin < 3
    peak_thresh = 3.4;
end
if nargin < 2
    first_octave = -1;
end

if ismatrix(I)
    I1 = I;
elseif ndims(I) == 3
    I1 = rgb2gray(I);
end

[f, d] = vl_sift(single(I1), 'FirstOctave', first_octave, 'peakthresh', peak_thresh);