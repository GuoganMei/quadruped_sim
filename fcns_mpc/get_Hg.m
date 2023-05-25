function [H,g] = get_Hg(Aqp,Bqp,Xt,XdN,p)
% x0:13*1 initial state,x0(13)=-q.g
%formulate qp problem
    temp=repmat({p.state_weight},p.predHorizon,1);
    L=blkdiag(temp{:});
    temp=repmat({p.input_weight},p.predHorizon,1);
    K=blkdiag(temp{:});
    
    [~,X_ref13_N]=X2x(XdN,p);
    xref=reshape(X_ref13_N,[13*p.predHorizon,1]);
    [~,x0]=X2x(Xt,p);

    A=Bqp;
    B=Aqp*x0-xref;
    
    H=2*(A'*L*A+K);
    H=0.5*(H+H');
    g=A'*(L+L')*B;
end
