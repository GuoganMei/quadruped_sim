%% get the desire trajectory XdN with foot trajectory and update FSM
function XdN=get_XdN(Xt,XdN,FSM4,vcmd,wcmd,contact_state4N,p)
    %(TOBE delete)FSM4_next_step:next Tmpc time four leg FSM state
    p_com_init=reshape(Xt(1:3),[3,1]);
    v=reshape(Xt(4:6),[3,1]);
    R = reshape(Xt(7:15),[3,3]);
    angle=R2eulzyx(R);
    psi=angle(3);
    pfd3N_cell=cell(1,4);
    for i=1:4
        pfd3N_cell{i}=get_pfd3N(FSM4{i},p_com_init,v,psi,vcmd,wcmd,i,contact_state4N(i,:),p);
    end
    pfd12N=[pfd3N_cell{1};pfd3N_cell{2};pfd3N_cell{3};pfd3N_cell{4}];
    XdN(19:30,:)=pfd12N;
end

%% one foot location
%finite state machine
%FSM.state=1/2/3/4
%FSM.p0=trajectory startpoint
%FSM.pf=trajectory endpoint
function pfd3N = get_pfd3N(FSM,p_com_init,v,psi,vcmd,wcmd,hip_id,contact_state1N,p)
    %(TOBE delete)FSM_next_step:next Tmpc time one leg FSM state
    step=raibert_step(v,psi,vcmd,wcmd,hip_id,p);
    pfd3N=zeros(3,p.predHorizon);
%     (TOBE delete)FSM_next_step=[];
    for i=1:p.predHorizon
        switch FSM.state %last time FSM state
            case 1
                FSM.state=2;%now FSM state
                pfd3N(:,i)=computeSwingTrajectoryBezier(contact_state1N(i),FSM.p0,FSM.pf,p);
            case 2
                if contact_state1N(i)==1
                    FSM.state=3;
                    FSM.p0=FSM.pf;
                    pfd3N(:,i)=FSM.p0;
                else
                    pfd3N(:,i)=computeSwingTrajectoryBezier(contact_state1N(i),FSM.p0,FSM.pf,p);
                end
            case 3
                FSM.state=4;
                pfd3N(:,i)=FSM.p0;
            case 4
                if contact_state1N(i)==1
                    pfd3N(:,i)=FSM.p0;
                else
                    FSM.state=1;
                    p_com_swst=p_com_init+(v+cross(v,wcmd))*(i*p.Tmpc+p.Tsw);
                    p_com_swst(3)=0;%in xy plane
                    FSM.pf=p_com_swst+step;
                    pfd3N(:,i)=computeSwingTrajectoryBezier(contact_state1N(i),FSM.p0,FSM.pf,p);
                end
        end
%         (TOBE delete)if i==1
%             FSM_next_step=FSM;
%         end
    end
    
end
%% get swing position trajectory with step height 
function p_phase=computeSwingTrajectoryBezier(phase,p0,pf,p)
    %p0 is whole trajectory initial point
    % only in caluculate dy ddy that swingTime will be used
    step_ht=p.step_ht;
    p_phase=cubicBezier(p0,pf,phase);
    if phase<0.5
        zp=cubicBezier(p0(3),p0(3)+step_ht,phase*2);
    else
        zp=cubicBezier(p0(3)+step_ht,pf(3),phase * 2 - 1);
    end
    p_phase(3)=zp;
end
%% get cubicBezier position trajectory in x phase
function y=cubicBezier(y0,yf,x)
    %y0:initial point(3dim or 1dim)
    %yf:final point(3dim or 1dim)
    %x:phase:0~1
    y_Diff=yf-y0;
    bezier=x^3+3*(x^2*(1-x));
    y=y0+bezier*y_Diff;
end



