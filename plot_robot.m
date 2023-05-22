pic=2000;
pcom = Xout(pic,:)';
fig=figure();
hold on; grid on;axis square;axis equal;
xlim = [pcom(1)-0.5 pcom(1)+0.5];
ylim = [pcom(2)-0.5 pcom(2)+0.5];
zlim = [-0.2 0.6];

viewPt = [0.2,0.5,0.2];
view(viewPt);
fig_plot_robot(Xout(pic,:)',Uout(pic,:)',zeros(3,1),p);