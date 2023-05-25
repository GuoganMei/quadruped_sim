function [pf34N,x13N]=X2x(X30N,p)
% transform state vector from X = [pc dpc vR wb pf]'to x=[Theta pc omega
% dpc -g]',but can have N Xd data
    x13N=zeros(13,size(X30N,2));
    pf34N=cell(1,size(X30N,2));
    for i=1:size(X30N,2)
        [pf34N{i},x13N(:,i)]=X2x_single(X30N(:,i),p);
    end
end

function [pf34,x]=X2x_single(Xt,p)
% transform state vector from X = [pc dpc vR wb pf]'to x=[Theta pc omega
% dpc -g]' and decomposite position of foot to pf34
pc = reshape(Xt(1:3),[3,1]);
dpc = reshape(Xt(4:6),[3,1]);
R = reshape(Xt(7:15),[3,3]);
wb = reshape(Xt(16:18),[3,1]);
pf34 = reshape(Xt(19:30),[3,4]);


Theta=R2eulzyx(R);
w=R*wb;
x=[Theta;pc;w;dpc;-p.g];
end