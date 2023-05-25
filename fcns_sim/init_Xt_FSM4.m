function [Xt,FSM4]= init_Xt_FSM4(p)
%get init Xt
    Xt=zeros(30,1);
    pc=[0,0,p.z0]';
    dpc=[0,0,0]';
    psi=0;theta=0;phi=0;
    wb=[0,0,0]';
    Xt(1:3)=pc;
    Xt(4:6)=dpc;
    R=rz(psi)*ry(theta)*rx(phi);
    Xt(7:15)=reshape(R,[9,1]);
    Xt(16:18)=wb;
    pf34=R*p.pf34+[pc(1),pc(2),0]';
    Xt(19:30)=reshape(pf34,[12,1]);
    FSM4=cell(1,4);
    for i=1:4
        FSM4{i}.state=4;
        FSM4{i}.p0=pf34(:,i);
        FSM4{i}.pf=pf34(:,i);
    end
end

