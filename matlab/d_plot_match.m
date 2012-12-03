function h = d_plot_match(frame1, frame2, offset2, matches)

xa = frame1(1,matches(1,:));
xb = frame2(1,matches(2,:)) + offset2(1);
ya = frame1(2,matches(1,:));
yb = frame2(2,matches(2,:)) + offset2(2);
h = line([xa ; xb], [ya ; yb]);
