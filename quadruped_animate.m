function quadruped_animate(pic_end,Xout,Uout,pfout,p)
    figure();
    pic_prop=10;
    for pic=1:pic_prop:pic_end
        clf;hold on; grid on;axis square;axis equal;
        pcom = Xout(pic,:)';
        xlim = [pcom(1)-0.5 pcom(1)+0.5];
        ylim = [pcom(2)-0.5 pcom(2)+0.5];
        zlim = [-0.1 0.6];
        axis([xlim ylim zlim]);
        viewPt = [0.2,0.5,0.2];
        view(viewPt);
        scatter3(Xout(1:pic,1),Xout(1:pic,2),Xout(1:pic,3),1,'r.');
        % scatter3(pfout(1:pic,1),pfout(1:pic,2),pfout(1:pic,3),1,'b.');
        % scatter3(pfout(1:pic,4),pfout(1:pic,5),pfout(1:pic,6),1,'b.');
        % scatter3(pfout(1:pic,7),pfout(1:pic,8),pfout(1:pic,9),1,'b.');
        % scatter3(pfout(1:pic,10),pfout(1:pic,11),pfout(1:pic,12),1,'b.');
        plot3(pfout(1:pic,1),pfout(1:pic,2),pfout(1:pic,3),'b');
        plot3(pfout(1:pic,4),pfout(1:pic,5),pfout(1:pic,6),'b');
        plot3(pfout(1:pic,7),pfout(1:pic,8),pfout(1:pic,9),'b');
        plot3(pfout(1:pic,10),pfout(1:pic,11),pfout(1:pic,12),'b');
        fig_plot_robot(Xout(pic,:)',Uout(pic,:)',zeros(3,1),p);
        
        frame = getframe(figure(1));
        [A,map]=rgb2ind(frame2im(frame),256);
        delay=p.simTimeStep*pic_prop;
        if pic==1
            imwrite(A,map,'testAnimated.gif','gif','LoopCount',Inf,'DelayTime',delay);
        else
            imwrite(A,map,'testAnimated.gif','gif','WriteMode','append','DelayTime',delay);
        end
    end
    close(1);
end