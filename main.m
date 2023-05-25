clc;
clear;
close all;
addpath("fcns_mpc");
addpath("fcns_sim");
addpath("fcns_motion_planning");
p=get_params();
%---------data storage-------------
pfout=[];
%---------set initial state--------
[Xt,FSM4]= init_Xt_FSM4(p);
%---------change param-------------
p.Tmpc=0.04;
p.predHorizon=6;

p.Tst = 0.2;
p.Tsw = 0.2;
p.k=0.04;
p.gaitT=p.Tst+p.Tsw;

p.Q_p=[30,30,1e-1,1,1,60];
p.Q_v=[1e-1,1e-1,1e-1,5,5,1e-1];
p.Q_f=diag([1e-4,1e-4,1e-4]);
p.state_weight=blkdiag(diag(p.Q_p),diag(p.Q_v),0);%state weight matrix
p.input_weight=blkdiag(p.Q_f,p.Q_f,p.Q_f,p.Q_f);
p.fzmax=40;
p.fzmin=0;
%----------set desire velocity------
vcmd=[0.7,0,0]';
wcmd=[0,0,0]';
%------simulation configure------
sim_time=9;%total sim time
control_loop_time=0.01;%each control loop time (which can not be Tmpc)
%--------------------------------
[tout,Xout,Uout] = deal([]);
h_waitbar = waitbar(0,'Calculating...');

for i=1:floor(sim_time/control_loop_time)
    t_arr=(i-1)*control_loop_time:p.simTimeStep:i*control_loop_time;
    %-------motion planing-------
    XdN=get_XdN_nopf(vcmd,wcmd,Xt,p);
%     Xd=zeros(30,1);
%     R=rz(0)*ry(0)*rx(0);
%     pf34=p.pf34;
%     Xd(3)=p.z0;
%     Xd(1)=0;
%     Xd(2)=0;
%     Xd(7:15)=reshape(R,[9,1]);
%     Xd(19:30)=reshape(pf34,[12,1]);
%     XdN=repmat(Xd,1,p.predHorizon);

    contact_state4N = get_contact_state4N((i-1)*control_loop_time,p);
    %contact_state4N=ones(4,p.predHorizon);
    
    XdN=get_XdN(Xt,XdN,FSM4,vcmd,wcmd,contact_state4N,p);
    %-------external force-------
    U_ext=zeros(3,1);
    p.p_ext=zeros(3,1);
    %-------mpc balance control--
    Ut=mpc_controller(Xt,XdN,contact_state4N,p);
    %-------sim------------------
    [t,X]=ode45(@(t,X)dynamics_SRB(t,X,Ut,XdN(:,1),U_ext,p),t_arr,Xt);
    Xt=X(end,:)';
    %----------------------------
    waitbar(i/floor(sim_time/control_loop_time),h_waitbar,'Calculating...');
    %-------record data----------
    lent = length(t(2:end));
    tout = [tout;t(2:end)];
    Xout = [Xout;X(2:end,:)];
    Uout = [Uout;repmat(Ut',[lent,1])];
    pfout=[pfout;X(2:end,19:30)];
    %-------update FSM4 for next control loop
    FSM4 = update_FSM4(FSM4,Xt,i*control_loop_time,vcmd,wcmd,p);
    if  Xt(3)<0 || Xt(3)>1
        break;
    end
end
close(h_waitbar)
fprintf('Calculation Complete!\n')
%% visulization
quadruped_animate(size(Xout,1),Xout,Uout,pfout,p);
subplot(3,1,1)
plot(Uout(:,1))
hold on
plot(Uout(:,2))
plot(Uout(:,3))
subplot(3,1,2)
angle=[];
for i=1:size(Xout,1)
    R=reshape(Xout(i,7:15),[3,3]);
    angle=[angle,R2eulzyx(R)];
end
plot(angle')
subplot(3,1,3)
plot(Xout(:,16:18));