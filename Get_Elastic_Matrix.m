function D = Get_Elastic_Matrix(E,nu)
% 基于泊松比和弹性模量得到弹性矩阵
D=zeros(3,3);
D0=E/(1-nu^2);
D(1,1)=1;
D(1,2)=nu;
D(2,1)=nu;
D(2,2)=1;
D(3,3)=(1-nu)/2;
D=D0*D;
end