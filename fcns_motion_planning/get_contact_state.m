%%
function contact_state=get_contact_state(t,p)
    %get contact state of four foot at time t
    %1:stance 0~1:swing
    contact_state=zeros(4,1);
    contact_state(1)=get_swing_phase(t,0,p);
    contact_state(4)=get_swing_phase(t,0,p);
    
    contact_state(2)=get_swing_phase(t,p.dphase,p);
    contact_state(3)=get_swing_phase(t,p.dphase,p);
end
%%
function phase=get_swing_phase(t,dphase,p)
    %calculate a swing foot's swing phase(0~1),if stance 1
    %default swing->stance->swing->stance
    t=t+dphase*p.gaitT;
    t=mod(t,p.gaitT);
    phase=1;
    if t<p.Tsw
        phase=t/p.Tsw;
    end
end