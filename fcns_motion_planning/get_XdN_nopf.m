function XdN = get_XdN_nopf(vcmd,wcmd,Xt,p)
    %get the desire trajectory in predict horizon but with no foot
    %information
    p_com_init=reshape(Xt(1:3),[3,1]);
    R = reshape(Xt(7:15),[3,3]);
    angle=R2eulzyx(R);
    psi=angle(3);
    
    XdN(4:6,:)=repmat(vcmd,1,p.predHorizon);%desire velocity
    XdN(16:18,:)=repmat(wcmd,1,p.predHorizon);%desire angle velocity in body frame
    
    v_cen=cross(vcmd,wcmd);
    for i=1:p.predHorizon
        Ri=rz(psi+wcmd(3)*i*p.Tmpc);
        XdN(7:15,i)=reshape(Ri,[9,1]);
        XdN(1:3,i)=p_com_init+(vcmd+v_cen)*i*p.Tmpc;
    end
    XdN(3,:)=p.z0;
end