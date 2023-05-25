function  FSM4 = update_FSM4(FSM4,Xt,t,vcmd,wcmd,p)
    contact_state=get_contact_state(t,p);
    %i is foot id
    for i=1:4
        if contact_state(i)==1
            if FSM4{i}.state==2
                FSM4{i}.state=3;
                pft=reshape(Xt(19:30),[3,4]);%position of foot at time t
                FSM4{i}.p0=[pft(1,i),pft(2,i),0]';
                FSM4{i}.pf=FSM4{i}.p0;
            elseif FSM4{i}.state==3
                FSM4{i}.state=4;
            end
        elseif contact_state(i)~=1
            if FSM4{i}.state==4
                FSM4{i}.state=1;
                
                p_com_now=reshape(Xt(1:3),[3,1]);
                v=reshape(Xt(4:6),[3,1]);
                R=reshape(Xt(7:15),[3,3]);
                angle=R2eulzyx(R);
                psi=angle(3);
                
                p_com_swst=p_com_now+vcmd*p.Tsw;
                p_com_swst(3)=0;
                
                step=raibert_step(v,psi,vcmd,wcmd,i,p);
                FSM4{i}.pf=p_com_swst+step;
            elseif FSM4{i}.state==1
                FSM4{i}.state=2;
            end
        end
    end
end

