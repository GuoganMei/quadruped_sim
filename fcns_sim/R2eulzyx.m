function eul = R2eulzyx(R)
    %transform rotation matrix to euler angle
    %eular angle Z-Y-X(YPR) but output is [phi theta psi] in [x,y,z]
    %sequence
    psi=atan2(R(2,1),R(1,1));
    theta=atan2(-R(3,1),sqrt(R(3,2)^2+R(3,3)^2));
    phi=atan2(R(3,2),R(3,3));
    eul=[phi theta psi]';
end

