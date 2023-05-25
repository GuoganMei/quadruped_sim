function contact_state4N = get_contact_state4N(t,p)
    %get contact state of four foot after time t in predict horizon
    %1:stance 0~1:swing
    
%     contact_state=get_contact_state(t+p.Tmpc,p);%next time step contact
%     state
%     contact_state4N=repmat(contact_state,1,p.predHorizon);
    
    contact_state4N=zeros(4,p.predHorizon);
    for i=1:p.predHorizon
        contact_state4N(:,i)=get_contact_state(t+i*p.Tmpc,p);
    end
end


