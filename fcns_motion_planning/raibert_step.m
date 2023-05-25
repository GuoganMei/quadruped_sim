%% step planner (in swing->stance time and need to add p_com in xy-plane)
function fp=raibert_step(v,psi,vcmd,wcmd,hip_id,p)
    %p_com:com position
    %v:com veclocity
    %psi:yaw
    %vcmd:control velocity
    %wcmd:control omega
    %hip_id:the ith hip
    
    %the position velocity here are all in xy-plane    
    v(3)=0;
    vcmd(3)=0;
    hip_com=p.pf34(:,hip_id);%hip location with respect to com
    hip_com(3)=0;
    
    p_hip=rz(psi)*hip_com;
    %p_sym=p.Tst/2*v+p.k*(vcmd-v);
    p_sym=p.Tst/2*(v+cross(v,wcmd))+p.k*(vcmd-v);
    p_cen=0.5*sqrt(p.z0/p.g)*cross(v,wcmd);
    fp=p_hip+p_sym+p_cen;%foot position
end