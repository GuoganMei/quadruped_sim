

% p0=[0,0,0]';
% pf=[1,1,0]';
% ps=[];
% step_ht=0.5;
% for phase=0:0.01:1
%     p=computeSwingTrajectoryBezier(phase,p0,pf,step_ht);
%     ps=[p,ps];
% end
% grid on
% plot3(ps(1,:),ps(2,:),ps(3,:));
switch 4
    case 0
        0
    case 1
        1
    case 2
        2
    otherwise
        3
end

function y=cubicBezier(y0,yf,x)
    %y0:initial point
    %yf:final point
    %x:phase:0~1
    y_Diff=yf-y0;
    bezier=x^3+3*(x^2*(1-x));
    y=y0+bezier*y_Diff;
end

function dy=cubicBezierFirstDerivative(y0,yf,x)
    yDiff=yf-y0;
    bezier=6*x*(1-x);
    dy=bezier*yDiff;
end

function ddy=cubicBezierSecondDerivative(y0,yf,x)
    yDiff=yf-y0;
    bezier=6-12*x;
    ddy=bezier*yDiff;
end
% def computeSwingTrajectoryBezier(phase,swingTime,p0,pf,step_ht):
%     p=cubicBezier(p0,pf,phase)
%     v=cubicBezierFirstDerivative(p0,pf,phase)/swingTime
%     a=cubicBezierSecondDerivative(p0,pf,phase)/(swingTime*swingTime)
%     if phase < 0.5:
%         zp = cubicBezier(p0[2], p0[2] + step_ht, phase * 2)
%         zv = cubicBezierFirstDerivative(p0[2], p0[2] + step_ht, phase * 2) * 2 / swingTime
%         za = cubicBezierSecondDerivative(p0[2], p0[2] + step_ht, phase * 2) * 4 / (swingTime * swingTime)
%     else:
%         zp = cubicBezier(p0[2] + step_ht, pf[2], phase * 2 - 1)
%         zv = cubicBezierFirstDerivative(p0[2] + step_ht, pf[2], phase * 2 - 1) * 2 / swingTime
%         za = cubicBezierSecondDerivative(p0[2] + step_ht, pf[2], phase * 2 - 1) * 4 / (swingTime * swingTime)
%     p[2]=zp
%     v[2]=zv
%     a[2]=za
%     return p,v,a
function p=computeSwingTrajectoryBezier(phase,p0,pf,step_ht)
    %p0 is whole trajectory initial point
    % only in caluculate dy ddy that swingTime will be used
    p=cubicBezier(p0,pf,phase);
    if phase<0.5
        zp=cubicBezier(p0(3),p0(3)+step_ht,phase*2);
    else
        zp=cubicBezier(p0(3)+step_ht,pf(3),phase * 2 - 1);
    end
    p(3)=zp;
end
