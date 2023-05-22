function out=mpc_controller(Xt,X_ref12_N,contact_state,p)
    [Aqp,Bqp]=get_AqpBqp(Xt,X_ref12_N,p);
    [H,g]=get_Hg(Aqp,Bqp,Xt,X_ref12_N,p);
    [A,b,Aeq,beq]=get_constraints(contact_state,p);
    
    U=quadprog(H,g,A,b,Aeq,beq);
    out=U(1:12);
end

