clear;
clc;
close all;
addpath("fcns_sim");
addpath("fcns_mpc");
addpath("fcns_motion_planning");
p=get_params;
% p.predHorizon=100;
% p.Tmpc=0.01;
[Xt,FSM4]= init_Xt_FSM4(p);
vcmd=[0.1,0,0]';
wcmd=[0,0,0]';
XdN=get_XdN_nopf(vcmd,wcmd,Xt,p);
contact_state4N = get_contact_state4N(0,p);
[FSM4,XdN]=get_XdN(Xt,XdN,FSM4,vcmd,wcmd,contact_state4N,p);

figure();
scatter3(XdN(19,:),XdN(20,:),XdN(21,:),'.');
hold on
scatter3(XdN(22,:),XdN(23,:),XdN(24,:),'.');
scatter3(XdN(25,:),XdN(26,:),XdN(27,:),'.');
scatter3(XdN(28,:),XdN(29,:),XdN(30,:),'.');

