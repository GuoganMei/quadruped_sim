function out=mpc_controller(Xt,XdN,contact_state4N,p)
    [Aqp,Bqp]=get_AqpBqp(Xt,XdN,p);
    [H,g]=get_Hg(Aqp,Bqp,Xt,XdN,p);
    [A,b,Aeq,beq]=get_constraints(contact_state4N,p);
    
    U=quadprog(H,g,A,b,Aeq,beq);
    out=U(1:12);
end

