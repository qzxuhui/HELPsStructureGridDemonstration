function U = Linear_Equations_Pivot_Gauss_Solver(K,P)
%% 函数说明
% 线性方程组求解器
% 算法：高斯循序消去法
% 求解 K*U=P
% ----------输入说明----------
% K：刚度矩阵
% P：载荷向量
% ----------输出说明----------
% U：位移向量
%% 矩阵参数
Mat_Scale=size(K,1);
%% 消元
for m=1:1:Mat_Scale-1  % 总共需要选择主元进行消元的次数
    %寻找列主元
    for n=m:1:Mat_Scale
        max_row = m;
            if K(m,m)<K(n,m)
                max_row=n;
            end
    end
    % 若当前元素不是主元，则进行行互换
    if max_row~=m
        %交换载荷向量
        temp=P(max_row);
        P(max_row)=P(m);
        P(m)=temp;
        for i = m:1:Mat_Scale
            %交换刚度矩阵
             temp=K(max_row,i);
             K(max_row,i)=K(m,i);
             K(m,i)=temp;
        end
    end
    % 在每次的消元中,对主元下方的元素进行消元素
    for i=m+1:1:Mat_Scale
        
        % 对载荷向量进行消元
        P(i)=P(i)-K(i,m)/K(m,m)*P(m);
        % 对单元刚度矩阵进行消元
        % 第i行和第m行进行加减运算,m行作为基准
        K(i,:)=K(i,:)-K(i,m)/K(m,m)*K(m,:);
    end
end
%% 回代
U=zeros(Mat_Scale,1);
U(Mat_Scale)=P(Mat_Scale)/K(Mat_Scale,Mat_Scale);
for i=Mat_Scale-1:-1:1
    U(i)=(P(i)-K(i,i+1:end)*U(i+1:end))/K(i,i);
end
end