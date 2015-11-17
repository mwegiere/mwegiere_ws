close all;

[t,h] = meshgrid(0:.0001:0.02, 0.2:0.01:0.5);
dx = 0.5;
V = (h * dx ./ t) ./ 1032;
%surf(t,h,V);
%set(gca, 'XScale', 'lin', 'YScale', 'lin', 'ZScale', 'log');
plot(0:.0001:0.02,(0.2 * dx ./ t) ./ 1032);
hold on
plot(0:.0001:0.02,(0.3 * dx ./ t) ./ 1032);
plot(0:.0001:0.02,(0.4 * dx ./ t) ./ 1032);
plot(0:.0001:0.02,(0.5 * dx ./ t) ./ 1032);
set(gca, 'XScale', 'lin', 'YScale', 'log');