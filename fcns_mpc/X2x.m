function x=X2x(Xt,p)
% transform state vector from X = [pc dpc vR wb pf]'to x=[Theta pc omega
% dpc -g]'
pc = reshape(Xt(1:3),[3,1]);
dpc = reshape(Xt(4:6),[3,1]);
R = reshape(Xt(7:15),[3,3]);
wb = reshape(Xt(16:18),[3,1]);

Theta=R2eulzyx(R);
w=R*wb;
x=[Theta;pc;w;dpc;-p.g];
end