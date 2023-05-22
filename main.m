clc;
clear;
close all;
addpath("fcns_mpc");
addpath("fcns_sim");
p=get_params();
X_ref12_N=zeros(12,p.predHorizon);

%---------set initial state--------
Xt=zeros(30,1);
Xt(1:3)=[0,0,0.2]';
Xt(4:6)=zeros(3,1);
R=rz(0)*ry(0)*rx(0);
Xt(7:15)=reshape(R,[9,1]);
Xt(16:18)=zeros(3,1);
pf34=[p.L/2,p.W/2,0;...
    p.L/2,-p.W/2,0;...
    -p.L/2,p.W/2,0;...
    -p.L/2,-p.W/2,0;]';
Xt(19:30)=reshape(pf34,[12,1]);
%----------set desire state------
Xd=Xt;
Xd(3)=0.15;
R=rz(0)*ry(-0.15)*rx(0);
Xd(7:15)=reshape(R,[9,1]);
%------simulation configure--------
sim_time=10;%total sim time
control_loop_time=p.Tmpc;%each control loop time (which can not be Tmpc)
%-----------------------------
[tout,Xout,Uout] = deal([]);
h_waitbar = waitbar(0,'Calculating...');

for i=1:floor(sim_time/p.Tmpc)
    t_arr=(i-1)*control_loop_time:p.simTimeStep:i*control_loop_time;

    contact_state=[1,1,1,1];
    xref=X2x(Xd,p);
    X_ref12_N=repmat(xref(1:12,1),1,p.predHorizon);
    U_ext=zeros(3,1);
    p.p_ext=zeros(3,1);

    Ut=mpc_controller(Xt,X_ref12_N,contact_state,p);
    [t,X]=ode45(@(t,X)dynamics_SRB(t,X,Ut,Xd,U_ext,p),t_arr,Xt);
    Xt=X(end,:)';
    waitbar(i/floor(sim_time/p.Tmpc),h_waitbar,'Calculating...');

    lent = length(t(2:end));
    tout = [tout;t(2:end)];
    Xout = [Xout;X(2:end,:)];
    Uout = [Uout;repmat(Ut',[lent,1])];
end
close(h_waitbar)
fprintf('Calculation Complete!\n')
%% visulization