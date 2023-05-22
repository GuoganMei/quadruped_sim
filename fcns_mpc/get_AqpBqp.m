function [Aqp,Bqp] = get_AqpBqp(Xt,X_ref12_N,p)
    %decomposite
    avg_psi=mean(X_ref12_N(3,:));
    R0 = reshape(Xt(7:15),[3,3]);
    pf34 = reshape(Xt(19:30),[3,4]);
    
    eulzyx=R2eulzyx(R0);
    psi0=eulzyx(3);
    
    Ac0=get_Ac(psi0);
    Ad0=dis_A(Ac0,p.Tmpc);
    Bc0=get_Bc(psi0,pf34,p);
    Bd0=dis_B(Ac0,Bc0,p.Tmpc);
    
    Ac=get_Ac(avg_psi);
    Ad=dis_A(Ac,p.Tmpc);
    
    Aqp=[];
    Bqp=[];
    
    An=Ad0;
    Bn=[Bd0,zeros(13,12*(p.predHorizon-1))];
    for k=1:(p.predHorizon-1)
        Aqp=[Aqp;An];
        Bqp=[Bqp;Bn];
        
        An=Ad*An;
        Bc=get_Bc(X_ref12_N(3,k),pf34,p);
        Bd=dis_B(Ac,Bc,p.Tmpc);
        Bn=Ad*Bn+[zeros(13,12*k),Bd,zeros(13,12*(p.predHorizon-k-1))];
    end
    Aqp=[Aqp;An];
    Bqp=[Bqp;Bn];
end
%% get continus A
function Ac=get_Ac(psi)
%   rz from addpath("../fcns_sim");
    temp=zeros(3,1);
    temp(3,1)=1;
    Ac=[zeros(3),zeros(3),rz(psi),zeros(3),zeros(3,1);...
        zeros(3),zeros(3),zeros(3),eye(3),zeros(3,1);...
        zeros(3),zeros(3),zeros(3),zeros(3),zeros(3,1);...
        zeros(3),zeros(3),zeros(3),zeros(3),temp;...
        zeros(1,13)];
end
%% get continus B
function Bc= get_Bc(psi,pf34,p)
%   rz hatMaP from addpath("../fcns_sim");
    I=rz(psi)*p.J*rz(psi)';
    Bc=[zeros(3),zeros(3),zeros(3),zeros(3);...
        zeros(3),zeros(3),zeros(3),zeros(3);...
        I\hatMap(pf34(:,1)),I\hatMap(pf34(:,2)),I\hatMap(pf34(:,3)),I\hatMap(pf34(:,4));...
        eye(3)/p.mass,eye(3)/p.mass,eye(3)/p.mass,eye(3)/p.mass;...
        zeros(1,12)];
end
%% zero order hold discretized
function Ad=dis_A(Ac,T)
    n=4; %can be inf
    A_k=eye(size(Ac));
    Ad=A_k;
    for k=1:n
        A_k=Ac*A_k;
        Ad=Ad+A_k*T^k/factorial(k);
    end
end
%% zero order hold discretized
function Bd=dis_B(Ac,Bc,T)
    n=4;
    A_k=eye(size(Ac));
    Bd=T*Bc;
    for k=1:n
        A_k=Ac*A_k;
        Bd=Bd+A_k*Bc*T^(k+1)/factorial(k+1);
    end
end
