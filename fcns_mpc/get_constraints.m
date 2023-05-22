function[Aineq,bineq,Aeq,beq]=get_constraints(contact_state,p)
    [Aineq,bineq]=friction_constraint(p);
    [Aeq,beq]=swing_force_constraint(contact_state,p);
end

function[Aeq,beq]=swing_force_constraint(contact_state,p)
    [D,Z]=swing_select_matrix(contact_state);
    temp=repmat({D},p.predHorizon,1);
    Aeq=blkdiag(temp{:});
    beq=repmat(Z,p.predHorizon,1);
end

function [Aineq,bineq] = friction_constraint(p)
    Afc=[1,0,-0.7071*p.mu;...
        -1,0,-0.7071*p.mu;...
        0,1,-0.7071*p.mu;...
        0,-1,-0.7071*p.mu;...
        0,0,1;...
        0,0,-1;];
    bfc=[0;0;0;0;p.fzmax;-p.fzmin];
    %4 leg and predHorizong force
    temp=repmat({Afc},4*p.predHorizon,1);
    Aineq=blkdiag(temp{:});
    bineq=repmat(bfc,4*p.predHorizon,1);
end

function [D,Z]=swing_select_matrix(contact_state)
%swing leg is 0
%stance leg is 1
%D*[f1 f2 f3 f4]'=Z
%Z=0 is swing leg force
    D=[];
    Z=[];
    for i=1:4
        if contact_state(i)==0
            temp=zeros(3,3*4);
            temp(:,3*(i-1)+1:3*i)=eye(3);
            D=[D;temp];
            Z=[Z;zeros(3,1)];
        end
    end
end
