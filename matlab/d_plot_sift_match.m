function [h_matches, h_matched_pts, h_unmatched_pts] = d_plot_sift_match(f1, f2, offset2, matches, plot_sel)

if nargin < 5
    plot_sel = [1;1;1];
end

f2_show = f2;
f2_show(1,:) = f2(1,:) + offset2(1);
f2_show(2,:) = f2(2,:) + offset2(2);

h_matches = [];
h_unmatched_pts = [];
h_matched_pts = [];

if plot_sel(1)
    h_matches = line([f1(1,matches(1,:));f2_show(1,matches(2,:))], ...
            [f1(2,matches(1,:));f2_show(2,matches(2,:))]);
end
if plot_sel(3)
    h_unmatched_pts = [vl_plotframe(f1); vl_plotframe(f2_show)];
end
if plot_sel(2)
    h_matched_pts = [vl_plotframe(f1(:,matches(1,:))); ...
            vl_plotframe(f2_show(:,matches(2,:)))];
end