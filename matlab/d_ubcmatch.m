function [matches,scores] = d_ubcmatch(d1, d2, dist_ratio)

if nargin < 3
    dist_ratio = 2.7;
end

[matches,scores] = vl_ubcmatch(d1, d2, dist_ratio);
