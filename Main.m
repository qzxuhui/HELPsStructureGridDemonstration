%% 程序说明
% 使用有限单元法求解平面结构问题
% 求解区域为矩形
% 该程序作为标准程序,线性方程组求解器使用多级平衡线性方程组求解器,作为标准值为以后实验提供验证。
% 储存全部刚度矩阵和载荷向量
% 网格划分在本地
% 徐辉 20170210 于 浙江衢州
%% 初始化
clc
clear
close all
%% 求解器参数
Num_Node_Extend_X=2;%松弛时扩展计算的横向节点数（包含了最外层的一些固定节点）
Num_Node_Extend_Y=2;%松弛时扩展计算的纵向节点数（包含了最外层的一些固定节点）
Max_Iter=30;
tol=1e-5;
Algorithm_Factor=[Num_Node_Extend_X,Num_Node_Extend_Y];
Num_Displacement_Model=12;
%% 并行处理器参数
Num_Master_Processor=1;
Num_Slave_Processor_Row=3;
Num_Slave_Processor_Col=3;
Num_Slave_Processor=Num_Slave_Processor_Row*Num_Slave_Processor_Col;
%% 并行处理器初始化
%% 数值参数
INF=1e10;
%% 材料参数
E_0=100;
nu_0=0;
% 弹性矩阵
D=Get_Elastic_Matrix(E_0,nu_0);
%% 建立模型
X_Range_Model=10;
Y_Range_Model=10;
Thickness=1;
%% 从处理器分配模型空间
% 为结构体数组预分配内存空间
Slave_Processor(Num_Slave_Processor)=struct();
Master_Processor=struct();
% 从处理器获取从处理器序号和空间拓扑
Slave_Processor = Slave_Processor_Get_ID_Topy(Slave_Processor,Num_Slave_Processor_Row,Num_Slave_Processor_Col);
% 从处理器计算得到从处理器类型
Slave_Processor = Slave_Processor_Get_Processor_Type(Slave_Processor,Num_Slave_Processor_Row,Num_Slave_Processor_Col);
% 从处理器计算周围邻居序号
Slave_Processor = Slave_Processor_Get_Neighbour_ID(Slave_Processor,Num_Slave_Processor_Row,Num_Slave_Processor_Col);
%% 网格划分参数
% 全局网格划分参数
Num_X_Element_Global=9;
Num_X_Node_Global=Num_X_Element_Global+1;
Num_Y_Element_Global=9;
Num_Y_Node_Global=Num_Y_Element_Global+1;
dX=X_Range_Model/Num_X_Element_Global;
dY=Y_Range_Model/Num_Y_Element_Global;
Num_Node_Global=Num_X_Node_Global*Num_Y_Node_Global;
Num_Element_Global=Num_X_Element_Global*Num_Y_Element_Global*2;% 乘2的原因在于一个正方形中有两个三角形，形成上三角单元和下三角单元
Mesh_Factor=[dX,dY,...
    Num_X_Node_Global,Num_Y_Node_Global,Num_Node_Global,...
    Num_X_Element_Global,Num_Y_Element_Global,Num_Element_Global];
%% 处理器分配
% 从处理器分配参数
% 沿着X方向进行分配
Num_X_Node_Local_Floor=floor(Num_X_Node_Global/Num_Slave_Processor_Col);      %记录后面几个处理器的信息
Num_X_Node_Local_Ceil=ceil(Num_X_Node_Global/Num_Slave_Processor_Col);
if(Num_X_Node_Local_Floor==Num_X_Node_Local_Ceil)   %若恰好整除
    Num_Slave_Processor_Col_Using_Floor=Num_Slave_Processor_Col;
    Num_Slave_Processor_Col_Using_Ceil=0;
else%若不能整除,相当于两个未知数，两个方程，求解方程组
    %1.两类X从处理器之和等于X从处理器数
    %2.两类X从处理器上的局部节点数之和等于总节点数
    Num_Slave_Processor_Col_Using_Floor=Num_Slave_Processor_Col*Num_X_Node_Local_Ceil-Num_X_Node_Global;%这个看到这个公式不要去理解，就是解方程算出来的，要修改就再解一下方程看看
    Num_Slave_Processor_Col_Using_Ceil=Num_Slave_Processor_Col-Num_Slave_Processor_Col_Using_Floor;
end
% 沿着Y方向进行分配
Num_Y_Node_Local_Floor=floor(Num_Y_Node_Global/Num_Slave_Processor_Row);      %记录后面几个处理器的信息
Num_Y_Node_Local_Ceil=ceil(Num_Y_Node_Global/Num_Slave_Processor_Row);
if(Num_Y_Node_Local_Floor==Num_Y_Node_Local_Ceil)   %若恰好整除
    Num_Slave_Processor_Row_Using_Floor=Num_Slave_Processor_Row;
    Num_Slave_Processor_Row_Using_Ceil=0;
else%若不能整除,相当于两个未知数，两个方程，求解方程组
    %1.两类Y从处理器之和等于Y从处理器数
    %2.两类Y从处理器上的局部节点数之和等于总节点数
    Num_Slave_Processor_Row_Using_Floor=Num_Slave_Processor_Row*Num_Y_Node_Local_Ceil-Num_Y_Node_Global;%这个看到这个公式不要去理解，就是解方程算出来的，要修改就再解一下方程看看
    Num_Slave_Processor_Row_Using_Ceil=Num_Slave_Processor_Row-Num_Slave_Processor_Row_Using_Floor;
end
% 把分配结果给到每一个从处理器上
%每个从处理器计算当前处理器上分到的节点范围
m=1;
for i=1:1:Num_Slave_Processor_Row
    for j=1:1:Num_Slave_Processor_Col
        % 分配X方向节点信息
        if j<=Num_Slave_Processor_Col_Using_Ceil
            Slave_Processor(m).Num_X_Node_Local=Num_X_Node_Local_Ceil;
        else
            Slave_Processor(m).Num_X_Node_Local=Num_X_Node_Local_Floor;
        end
        % 分配Y方向节点信息
        if i<=Num_Slave_Processor_Row_Using_Ceil
            Slave_Processor(m).Num_Y_Node_Local=Num_Y_Node_Local_Ceil;
        else
            Slave_Processor(m).Num_Y_Node_Local=Num_Y_Node_Local_Floor;
        end
        Slave_Processor(m).Num_Node_Local=Slave_Processor(m).Num_X_Node_Local*Slave_Processor(m).Num_Y_Node_Local;
        % 计算X方向的全局节点标号的起始位置
        if j<=Num_Slave_Processor_Col_Using_Ceil
            Slave_Processor(m).Num_X_Node_Start_Global=(j-1)*Num_X_Node_Local_Ceil+1;
        else
            Slave_Processor(m).Num_X_Node_Start_Global=Num_Slave_Processor_Col_Using_Ceil*Num_X_Node_Local_Ceil+(j-Num_Slave_Processor_Col_Using_Ceil-1)*Num_X_Node_Local_Floor+1;
        end
        % 计算Y方向的全局节点标号的终止位置
        Slave_Processor(m).Num_X_Node_End_Global=Slave_Processor(m).Num_X_Node_Start_Global+Slave_Processor(m).Num_X_Node_Local-1;
        % 计算Y方向的全局节点标号的起始位置
        if i<=Num_Slave_Processor_Row_Using_Ceil
            Slave_Processor(m).Num_Y_Node_Start_Global=(i-1)*Num_Y_Node_Local_Ceil+1;
        else
            Slave_Processor(m).Num_Y_Node_Start_Global=Num_Slave_Processor_Row_Using_Ceil*Num_Y_Node_Local_Ceil+(i-Num_Slave_Processor_Row_Using_Ceil-1)*Num_Y_Node_Local_Floor+1;
        end
        % 计算Y方向的全局节点标号的终止位置
        Slave_Processor(m).Num_Y_Node_End_Global=Slave_Processor(m).Num_Y_Node_Start_Global+Slave_Processor(m).Num_Y_Node_Local-1;
        m=m+1;
    end
end
%% 规模计算
%从处理器获取扩展计算区域
% 每个从处理器得到计算所需的节点信息,建立缓存空间
M=1;
for I=1:1:Num_Slave_Processor_Row
    for J=1:1:Num_Slave_Processor_Col
        if  (Slave_Processor(M).Processor_Type ==1) || (Slave_Processor(M).Processor_Type ==3) || Slave_Processor(M).Processor_Type ==7 || Slave_Processor(M).Processor_Type ==9
            Slave_Processor(M).Num_X_Node_Cal=Num_Node_Extend_X+Slave_Processor(M).Num_X_Node_Local;
            Slave_Processor(M).Num_Y_Node_Cal=Num_Node_Extend_Y+Slave_Processor(M).Num_Y_Node_Local;
        end
        if  (Slave_Processor(M).Processor_Type ==4) || (Slave_Processor(M).Processor_Type ==6)
            Slave_Processor(M).Num_X_Node_Cal=Num_Node_Extend_X+Slave_Processor(M).Num_X_Node_Local;
            Slave_Processor(M).Num_Y_Node_Cal=2*Num_Node_Extend_Y+Slave_Processor(M).Num_Y_Node_Local;
        end
        if  (Slave_Processor(M).Processor_Type ==2) || (Slave_Processor(M).Processor_Type ==8)
            Slave_Processor(M).Num_X_Node_Cal=2*Num_Node_Extend_X+Slave_Processor(M).Num_X_Node_Local;
            Slave_Processor(M).Num_Y_Node_Cal=Num_Node_Extend_Y+Slave_Processor(M).Num_Y_Node_Local;
        end
        if  (Slave_Processor(M).Processor_Type ==5)
            Slave_Processor(M).Num_X_Node_Cal=2*Num_Node_Extend_X+Slave_Processor(M).Num_X_Node_Local;
            Slave_Processor(M).Num_Y_Node_Cal=2*Num_Node_Extend_Y+Slave_Processor(M).Num_Y_Node_Local;
        end
        Slave_Processor(M).Node_Coordinates_List_Calculate=zeros(Slave_Processor(M).Num_X_Node_Cal*Slave_Processor(M).Num_Y_Node_Cal,2);
        Slave_Processor(M).Node_Displacement_List_Calculate=zeros(Slave_Processor(M).Num_X_Node_Cal*Slave_Processor(M).Num_Y_Node_Cal,2);
        M=M+1;
    end
end
% 每一个从处理器标定第一个本地节点在计算节点群中的位置
M=1;
for I=1:1:Num_Slave_Processor_Row
    for J=1:1:Num_Slave_Processor_Col
        if  Slave_Processor(M).Processor_Type ==1
            Slave_Processor(M).Local_Node_Calculation_X_Location=1;
            Slave_Processor(M).Local_Node_Calculation_Y_Location=1;
        elseif Slave_Processor(M).Processor_Type ==2 || Slave_Processor(M).Processor_Type ==3
            Slave_Processor(M).Local_Node_Calculation_X_Location=Num_Node_Extend_X+1;
            Slave_Processor(M).Local_Node_Calculation_Y_Location=1;
        elseif Slave_Processor(M).Processor_Type ==4 || Slave_Processor(M).Processor_Type ==7
            Slave_Processor(M).Local_Node_Calculation_X_Location=1;
            Slave_Processor(M).Local_Node_Calculation_Y_Location=Num_Node_Extend_Y+1;
        else
            Slave_Processor(M).Local_Node_Calculation_X_Location=Num_Node_Extend_X+1;
            Slave_Processor(M).Local_Node_Calculation_Y_Location=Num_Node_Extend_Y+1;
        end
        M=M+1;
    end
end
% 每一个从处理器标记邻居元素到计算列表的索引，用于消息传递使用
M=1;
for I=1:1:Num_Slave_Processor_Row
    for J=1:1:Num_Slave_Processor_Col
        %n 用于标记从处理器计算区域的非边界节点在计算列表中的序号
        %m 用于标记从处理器计算区域的边界节点在计算列表中的序号
        % 标记起始序号
        n=Slave_Processor(M).Num_X_Node_Local*Slave_Processor(M).Num_Y_Node_Local+1;
        if Slave_Processor(M).Processor_Type== 1 || Slave_Processor(M).Processor_Type== 3 || Slave_Processor(M).Processor_Type== 7 || Slave_Processor(M).Processor_Type== 9
            m=(Slave_Processor(M).Num_X_Node_Local+Num_Node_Extend_X-1)*(Slave_Processor(M).Num_Y_Node_Local+Num_Node_Extend_Y-1)+1;
        elseif Slave_Processor(M).Processor_Type== 4 || Slave_Processor(M).Processor_Type== 6
            m=(Slave_Processor(M).Num_X_Node_Local+Num_Node_Extend_X-1)*(Slave_Processor(M).Num_Y_Node_Local+Num_Node_Extend_Y*2-2)+1;
        elseif Slave_Processor(M).Processor_Type== 2 || Slave_Processor(M).Processor_Type== 8
            m=(Slave_Processor(M).Num_X_Node_Local+Num_Node_Extend_X*2-2)*(Slave_Processor(M).Num_Y_Node_Local+Num_Node_Extend_Y-1)+1;
        else
            m=(Slave_Processor(M).Num_X_Node_Local+Num_Node_Extend_X*2-2)*(Slave_Processor(M).Num_Y_Node_Local+Num_Node_Extend_Y*2-2)+1;
        end
        % Neighbour_North_Index_Calculation_List 下保存信息
        % ============================标记邻居 North 的索引============================
        if Slave_Processor(M).Neighbour_North~=-1   %若有北面的邻居
            N=Slave_Processor(M).Neighbour_North;
            Slave_Processor(M).Neighbour_North_Index_Calculation_List=zeros(Num_Node_Extend_Y*Slave_Processor(M).Num_X_Node_Local,3);
            g=1; %用于标记当邻居传递信息的位置
            for j=1:1:Num_Node_Extend_Y
                for i=1:1:Slave_Processor(M).Num_X_Node_Local
                    Slave_Processor(M).Neighbour_North_Index_Calculation_List(g,2)=...
                        (Slave_Processor(N).Num_Y_Node_Local-Num_Node_Extend_Y+j-1)*(Slave_Processor(N).Num_X_Node_Local)+i;
                    if j==1
                        Slave_Processor(M).Neighbour_North_Index_Calculation_List(g,1)=m;
                        Slave_Processor(M).Neighbour_North_Index_Calculation_List(g,3)=1;
                        m=m+1;
                    else
                        Slave_Processor(M).Neighbour_North_Index_Calculation_List(g,1)=n;
                        Slave_Processor(M).Neighbour_North_Index_Calculation_List(g,3)=0;
                        n=n+1;
                    end
                    g=g+1;
                end
            end
        end
        % ============================标记邻居 NorthEast 的索引============================
        if Slave_Processor(M).Neighbour_NorthEast~=-1   %若有东北面的邻居
            N=Slave_Processor(M).Neighbour_NorthEast;
            Slave_Processor(M).Neighbour_NorthEast_Index_Calculation_List=zeros(Num_Node_Extend_Y*Num_Node_Extend_X,3);
            g=1; %用于标记当邻居传递信息的位置
            for j=1:1:Num_Node_Extend_Y
                for i=1:1:Num_Node_Extend_X
                    Slave_Processor(M).Neighbour_NorthEast_Index_Calculation_List(g,2)=...
                        (Slave_Processor(N).Num_Y_Node_Local-Num_Node_Extend_Y+j-1)*(Slave_Processor(N).Num_X_Node_Local)+i;
                    if j==1 || i == Num_Node_Extend_X
                        Slave_Processor(M).Neighbour_NorthEast_Index_Calculation_List(g,1)=m;
                        Slave_Processor(M).Neighbour_NorthEast_Index_Calculation_List(g,3)=1;
                        m=m+1;
                    else
                        Slave_Processor(M).Neighbour_NorthEast_Index_Calculation_List(g,1)=n;
                        Slave_Processor(M).Neighbour_NorthEast_Index_Calculation_List(g,3)=0;
                        n=n+1;
                    end
                    g=g+1;
                end
            end
        end
        % ============================标记邻居 East 的索引============================
        if Slave_Processor(M).Neighbour_East~=-1   %若有东面的邻居
            N=Slave_Processor(M).Neighbour_East;
            Slave_Processor(M).Neighbour_East_Index_Calculation_List=zeros(Slave_Processor(M).Num_Y_Node_Local*Num_Node_Extend_X,3);
            g=1; %用于标记当邻居传递信息的位置
            for j=1:1:Slave_Processor(M).Num_Y_Node_Local
                for i=1:1:Num_Node_Extend_X
                    Slave_Processor(M).Neighbour_East_Index_Calculation_List(g,2)=...
                        (j-1)*(Slave_Processor(N).Num_X_Node_Local)+i;
                    if i == Num_Node_Extend_X
                        Slave_Processor(M).Neighbour_East_Index_Calculation_List(g,1)=m;
                        Slave_Processor(M).Neighbour_East_Index_Calculation_List(g,3)=1;
                        m=m+1;
                    else
                        Slave_Processor(M).Neighbour_East_Index_Calculation_List(g,1)=n;
                        Slave_Processor(M).Neighbour_East_Index_Calculation_List(g,3)=0;
                        n=n+1;
                    end
                    g=g+1;
                end
            end
        end
        % ============================标记邻居 SouthEast 的索引============================
        if Slave_Processor(M).Neighbour_SouthEast~=-1   %若有东面的邻居
            N=Slave_Processor(M).Neighbour_SouthEast;
            Slave_Processor(M).Neighbour_SouthEast_Index_Calculation_List=zeros(Num_Node_Extend_Y*Num_Node_Extend_X,3);
            g=1; %用于标记当邻居传递信息的位置
            for j=1:1:Num_Node_Extend_Y
                for i=1:1:Num_Node_Extend_X
                    Slave_Processor(M).Neighbour_SouthEast_Index_Calculation_List(g,2)=...
                        (j-1)*(Slave_Processor(N).Num_X_Node_Local)+i;
                    if i == Num_Node_Extend_X || j==Num_Node_Extend_Y
                        Slave_Processor(M).Neighbour_SouthEast_Index_Calculation_List(g,1)=m;
                        Slave_Processor(M).Neighbour_SouthEast_Index_Calculation_List(g,3)=1;
                        m=m+1;
                    else
                        Slave_Processor(M).Neighbour_SouthEast_Index_Calculation_List(g,1)=n;
                        Slave_Processor(M).Neighbour_SouthEast_Index_Calculation_List(g,3)=0;
                        n=n+1;
                    end
                    g=g+1;
                end
            end
        end
        % ============================标记邻居 South 的索引============================
        if Slave_Processor(M).Neighbour_South~=-1   %若有东面的邻居
            N=Slave_Processor(M).Neighbour_South;
            Slave_Processor(M).Neighbour_South_Index_Calculation_List=zeros(Num_Node_Extend_Y*Slave_Processor(M).Num_X_Node_Local,3);
            g=1; %用于标记当邻居传递信息的位置
            for j=1:1:Num_Node_Extend_Y
                for i=1:1:Slave_Processor(M).Num_X_Node_Local
                    Slave_Processor(M).Neighbour_South_Index_Calculation_List(g,2)=...
                        (j-1)*(Slave_Processor(N).Num_X_Node_Local)+i;
                    if j==Num_Node_Extend_Y
                        Slave_Processor(M).Neighbour_South_Index_Calculation_List(g,1)=m;
                        Slave_Processor(M).Neighbour_South_Index_Calculation_List(g,3)=1;
                        m=m+1;
                    else
                        Slave_Processor(M).Neighbour_South_Index_Calculation_List(g,1)=n;
                        Slave_Processor(M).Neighbour_South_Index_Calculation_List(g,3)=0;
                        n=n+1;
                    end
                    g=g+1;
                end
            end
        end
        % ============================标记邻居 SouthWest 的索引============================
        if Slave_Processor(M).Neighbour_SouthWest~=-1   %若有东面的邻居
            N=Slave_Processor(M).Neighbour_SouthWest;
            Slave_Processor(M).Neighbour_SouthWest_Index_Calculation_List=zeros(Num_Node_Extend_Y*Num_Node_Extend_X,3);
            g=1; %用于标记当邻居传递信息的位置
            for j=1:1:Num_Node_Extend_Y
                for i=1:1:Num_Node_Extend_X
                    Slave_Processor(M).Neighbour_SouthWest_Index_Calculation_List(g,2)=...
                        (j-1)*(Slave_Processor(N).Num_X_Node_Local)+Slave_Processor(N).Num_X_Node_Local-Num_Node_Extend_X+i;
                    if i == 1 || j==Num_Node_Extend_Y
                        Slave_Processor(M).Neighbour_SouthWest_Index_Calculation_List(g,1)=m;
                        Slave_Processor(M).Neighbour_SouthWest_Index_Calculation_List(g,3)=1;
                        m=m+1;
                    else
                        Slave_Processor(M).Neighbour_SouthWest_Index_Calculation_List(g,1)=n;
                        Slave_Processor(M).Neighbour_SouthWest_Index_Calculation_List(g,3)=0;
                        n=n+1;
                    end
                    g=g+1;
                end
            end
        end
        % ============================标记邻居 West 的索引============================
        if Slave_Processor(M).Neighbour_West~=-1   %若有东面的邻居
            N=Slave_Processor(M).Neighbour_West;
            Slave_Processor(M).Neighbour_West_Index_Calculation_List=zeros(Slave_Processor(M).Num_Y_Node_Local*Num_Node_Extend_X,3);
            g=1; %用于标记当邻居传递信息的位置
            for j=1:1:Slave_Processor(M).Num_Y_Node_Local
                for i=1:1:Num_Node_Extend_X
                    Slave_Processor(M).Neighbour_West_Index_Calculation_List(g,2)=...
                        (j-1)*(Slave_Processor(N).Num_X_Node_Local)+Slave_Processor(N).Num_X_Node_Local-Num_Node_Extend_X+i;
                    if i == 1
                        Slave_Processor(M).Neighbour_West_Index_Calculation_List(g,1)=m;
                        Slave_Processor(M).Neighbour_West_Index_Calculation_List(g,3)=1;
                        m=m+1;
                    else
                        Slave_Processor(M).Neighbour_West_Index_Calculation_List(g,1)=n;
                        Slave_Processor(M).Neighbour_West_Index_Calculation_List(g,3)=0;
                        n=n+1;
                    end
                    g=g+1;
                end
            end
        end
        % ============================标记邻居 NorthWest 的索引============================
        if Slave_Processor(M).Neighbour_NorthWest~=-1   %若有东北面的邻居
            N=Slave_Processor(M).Neighbour_NorthWest;
            Slave_Processor(M).Neighbour_NorthWest_Index_Calculation_List=zeros(Num_Node_Extend_Y*Num_Node_Extend_X,3);
            g=1; %用于标记当邻居传递信息的位置
            for j=1:1:Num_Node_Extend_Y
                for i=1:1:Num_Node_Extend_X
                    Slave_Processor(M).Neighbour_NorthWest_Index_Calculation_List(g,2)=...
                        (Slave_Processor(N).Num_Y_Node_Local-Num_Node_Extend_Y+j-1)*(Slave_Processor(N).Num_X_Node_Local)+Slave_Processor(N).Num_X_Node_Local-Num_Node_Extend_X+i;
                    if j==1 || i == 1
                        Slave_Processor(M).Neighbour_NorthWest_Index_Calculation_List(g,1)=m;
                        Slave_Processor(M).Neighbour_NorthWest_Index_Calculation_List(g,3)=1;
                        m=m+1;
                    else
                        Slave_Processor(M).Neighbour_NorthWest_Index_Calculation_List(g,1)=n;
                        Slave_Processor(M).Neighbour_NorthWest_Index_Calculation_List(g,3)=0;
                        n=n+1;
                    end
                    g=g+1;
                end
            end
        end
        M=M+1;
    end
end
% 每一个从处理器标记计算区域矩阵到计算列表的索引，用于形成单元信息表使用
M=1;
for I=1:1:Num_Slave_Processor_Row
    for J=1:1:Num_Slave_Processor_Col
        % 建立计算矩阵区域矩阵空间
        Slave_Processor(M).Node_Information_Matrix_Calculate=zeros(Slave_Processor(M).Num_Y_Node_Cal,Slave_Processor(M).Num_X_Node_Cal);
        %对从处理器的本底的信息填入区域计算矩阵 Local
        n=1;
        st_x=Slave_Processor(M).Local_Node_Calculation_X_Location;
        st_y=Slave_Processor(M).Local_Node_Calculation_Y_Location;
        for j=1:1:Slave_Processor(M).Num_Y_Node_Local
            for i=1:1:Slave_Processor(M).Num_X_Node_Local
                Slave_Processor(M).Node_Information_Matrix_Calculate(st_y+j-1,st_x+i-1)=n;
                n=n+1;
            end
        end
        
        %将从处理器周围邻居的信息填入区域计算矩阵 North
        if Slave_Processor(M).Neighbour_North~=-1
            n=1;
            st_x=Slave_Processor(M).Local_Node_Calculation_X_Location;
            st_y=Slave_Processor(M).Local_Node_Calculation_Y_Location-Num_Node_Extend_Y;
            for j=1:1:Num_Node_Extend_Y
                for i=1:1:Slave_Processor(M).Num_X_Node_Local
                    Slave_Processor(M).Node_Information_Matrix_Calculate(st_y+j-1,st_x+i-1)=Slave_Processor(M).Neighbour_North_Index_Calculation_List(n);
                    n=n+1;
                end
            end
        end
        %将从处理器周围邻居的信息填入区域计算矩阵 NorthEast
        if Slave_Processor(M).Neighbour_NorthEast~=-1
            n=1;
            st_x=Slave_Processor(M).Local_Node_Calculation_X_Location+Slave_Processor(M).Num_X_Node_Local;
            st_y=Slave_Processor(M).Local_Node_Calculation_Y_Location-Num_Node_Extend_Y;
            for j=1:1:Num_Node_Extend_Y
                for i=1:1:Num_Node_Extend_X
                    Slave_Processor(M).Node_Information_Matrix_Calculate(st_y+j-1,st_x+i-1)=Slave_Processor(M).Neighbour_NorthEast_Index_Calculation_List(n);
                    n=n+1;
                end
            end
        end
        %将从处理器周围邻居的信息填入区域计算矩阵 East
        if Slave_Processor(M).Neighbour_East~=-1
            n=1;
            st_x=Slave_Processor(M).Local_Node_Calculation_X_Location+Slave_Processor(M).Num_X_Node_Local;
            st_y=Slave_Processor(M).Local_Node_Calculation_Y_Location;
            for j=1:1:Slave_Processor(M).Num_Y_Node_Local
                for i=1:1:Num_Node_Extend_X
                    Slave_Processor(M).Node_Information_Matrix_Calculate(st_y+j-1,st_x+i-1)=Slave_Processor(M).Neighbour_East_Index_Calculation_List(n);
                    n=n+1;
                end
            end
        end
        %将从处理器周围邻居的信息填入区域计算矩阵 SouthEast
        if Slave_Processor(M).Neighbour_SouthEast~=-1
            n=1;
            st_x=Slave_Processor(M).Local_Node_Calculation_X_Location+Slave_Processor(M).Num_X_Node_Local;
            st_y=Slave_Processor(M).Local_Node_Calculation_Y_Location+Slave_Processor(M).Num_Y_Node_Local;
            for j=1:1:Num_Node_Extend_Y
                for i=1:1:Num_Node_Extend_X
                    Slave_Processor(M).Node_Information_Matrix_Calculate(st_y+j-1,st_x+i-1)=Slave_Processor(M).Neighbour_SouthEast_Index_Calculation_List(n);
                    n=n+1;
                end
            end
        end
        %将从处理器周围邻居的信息填入区域计算矩阵 South
        if Slave_Processor(M).Neighbour_South~=-1
            n=1;
            st_x=Slave_Processor(M).Local_Node_Calculation_X_Location;
            st_y=Slave_Processor(M).Local_Node_Calculation_Y_Location+Slave_Processor(M).Num_Y_Node_Local;
            for j=1:1:Num_Node_Extend_Y
                for i=1:1:Slave_Processor(M).Num_X_Node_Local
                    Slave_Processor(M).Node_Information_Matrix_Calculate(st_y+j-1,st_x+i-1)=Slave_Processor(M).Neighbour_South_Index_Calculation_List(n);
                    n=n+1;
                end
            end
        end
        %将从处理器周围邻居的信息填入区域计算矩阵 SouthWest
        if Slave_Processor(M).Neighbour_SouthWest~=-1
            n=1;
            st_x=Slave_Processor(M).Local_Node_Calculation_X_Location-Num_Node_Extend_X;
            st_y=Slave_Processor(M).Local_Node_Calculation_Y_Location+Slave_Processor(M).Num_Y_Node_Local;
            for j=1:1:Num_Node_Extend_Y
                for i=1:1:Num_Node_Extend_X
                    Slave_Processor(M).Node_Information_Matrix_Calculate(st_y+j-1,st_x+i-1)=Slave_Processor(M).Neighbour_SouthWest_Index_Calculation_List(n);
                    n=n+1;
                end
            end
        end
        %将从处理器周围邻居的信息填入区域计算矩阵 West
        if Slave_Processor(M).Neighbour_West~=-1
            n=1;
            st_x=Slave_Processor(M).Local_Node_Calculation_X_Location-Num_Node_Extend_X;
            st_y=Slave_Processor(M).Local_Node_Calculation_Y_Location;
            for j=1:1:Slave_Processor(M).Num_Y_Node_Local
                for i=1:1:Num_Node_Extend_X
                    Slave_Processor(M).Node_Information_Matrix_Calculate(st_y+j-1,st_x+i-1)=Slave_Processor(M).Neighbour_West_Index_Calculation_List(n);
                    n=n+1;
                end
            end
        end
        %将从处理器周围邻居的信息填入区域计算矩阵 NorthWest
        if Slave_Processor(M).Neighbour_NorthWest~=-1
            n=1;
            st_x=Slave_Processor(M).Local_Node_Calculation_X_Location-Num_Node_Extend_X;
            st_y=Slave_Processor(M).Local_Node_Calculation_Y_Location-Num_Node_Extend_Y;
            for j=1:1:Num_Node_Extend_Y
                for i=1:1:Num_Node_Extend_X
                    Slave_Processor(M).Node_Information_Matrix_Calculate(st_y+j-1,st_x+i-1)=Slave_Processor(M).Neighbour_NorthWest_Index_Calculation_List(n);
                    n=n+1;
                end
            end
        end
        M=M+1;
    end
end
% 每一个从处理器得到计算区域的内部节点和边界节点
M=1;
for I=1:1:Num_Slave_Processor_Row
    for J=1:1:Num_Slave_Processor_Col
        if Slave_Processor(M).Neighbour_East ==-1 || Slave_Processor(M).Neighbour_West ==-1
            n=Slave_Processor(M).Num_X_Node_Cal-1;
        else
            n=Slave_Processor(M).Num_X_Node_Cal-2;
        end
        if Slave_Processor(M).Neighbour_South ==-1 || Slave_Processor(M).Neighbour_North ==-1
            m=Slave_Processor(M).Num_Y_Node_Cal-1;
        else
            m=Slave_Processor(M).Num_Y_Node_Cal-2;
        end
        Slave_Processor(M).Num_Node_Cal_Inner=n*m;
        Slave_Processor(M).Num_Node_Cal_Outer=(Slave_Processor(M).Num_X_Node_Cal)*(Slave_Processor(M).Num_Y_Node_Cal)-Slave_Processor(M).Num_Node_Cal_Inner;
        M=M+1;
    end
end
%% 节点信息计算
% 得到从处理器下所有计算节点的横纵坐标
M=1;
for I=1:1:Num_Slave_Processor_Row
    for J=1:1:Num_Slave_Processor_Col
        %计算当前从处理器的最西北端计算节点的全局坐标
        st_x=Slave_Processor(M).Num_X_Node_Start_Global;
        st_y=Slave_Processor(M).Num_Y_Node_Start_Global;
        if Slave_Processor(M).Neighbour_North~=-1
            st_y=st_y-Num_Node_Extend_Y;
        end
        if Slave_Processor(M).Neighbour_West~=-1
            st_x=st_x-Num_Node_Extend_X;
        end
        for i=1:1:Slave_Processor(M).Num_Y_Node_Cal
            for j=1:1:Slave_Processor(M).Num_X_Node_Cal
                % 获取目标点序号
                n=Slave_Processor(M).Node_Information_Matrix_Calculate(i,j);
                % 保存目标点横坐标
                Slave_Processor(M).Node_Coordinates_List_Calculate(n,1)=(st_x+j-2)*dX;
                % 保存目标点纵坐标
                Slave_Processor(M).Node_Coordinates_List_Calculate(n,2)=Y_Range_Model-(st_y+i-2)*dY;
            end
        end
        M=M+1;
    end
end
% 得到从处理器的中心坐标
M=1;
for I=1:1:Num_Slave_Processor_Row
    for J=1:1:Num_Slave_Processor_Col
        %计算当前从处理器的最西北端计算节点的全局坐标
        c_x=0;
        c_y=0;
        for i=1:1:Slave_Processor(M).Num_Node_Local
            c_x=c_x+Slave_Processor(M).Node_Coordinates_List_Calculate(i,1);
            c_y=c_y+Slave_Processor(M).Node_Coordinates_List_Calculate(i,2);
        end
        c_x=c_x/Slave_Processor(M).Num_Node_Local;
        c_y=c_y/Slave_Processor(M).Num_Node_Local;
        Slave_Processor(M).Center_X_Coordinate=c_x;
        Slave_Processor(M).Center_Y_Coordinate=c_y;
        M=M+1;
    end
end
%% 单元信息计算
% 每一个从处理器形成单元信息列表
M=1;
for I=1:1:Num_Slave_Processor_Row
    for J=1:1:Num_Slave_Processor_Col
        Num_Element_X=Slave_Processor(M).Num_X_Node_Cal-1;
        Num_Element_Y=Slave_Processor(M).Num_Y_Node_Cal-1;
        % 形成单元信息表
        % 第一个元素用于存放单元中第一个节点的信息
        % 第二个元素用于存放单元中第二个节点的信息
        % 第三个元素用于存放单元中第三个节点的信息
        % 第四个元素用于存放单元类型 1.全是区域内部点组成的单元 2.边界节点和内部节点组成的混合单元 3.全都由边界节点形成的单元
        Slave_Processor(M).Num_Element_Calculate=Num_Element_X*Num_Element_Y*2;
        Slave_Processor(M).Element_Information_List=zeros(Num_Element_X*Num_Element_Y*2,4);
        n=1;
        for i=1:1:Num_Element_Y
            for j=1:1:Num_Element_X
                
                % =========================保存上三角单元==============================
                % 从二维节点表中获取单元的节点
                Node_A_ID=Slave_Processor(M).Node_Information_Matrix_Calculate(i,j);
                Node_B_ID=Slave_Processor(M).Node_Information_Matrix_Calculate(i+1,j);
                Node_C_ID=Slave_Processor(M).Node_Information_Matrix_Calculate(i,j+1);
                % 判断单元类型
                inner=Slave_Processor(M).Num_Node_Cal_Inner;
                Flag_A=Node_A_ID>inner;
                Flag_B=Node_B_ID>inner;
                Flag_C=Node_C_ID>inner;
                if Flag_A*Flag_B*Flag_C==1            %所有节点的序号都大于内部节点数，则为都是外部节点组成的单元
                    Slave_Processor(M).Element_Information_List(n,4)=3;
                elseif  Flag_A+Flag_B+Flag_C==0       %所有节点的序号都小于内部节点数，则为都是内部节点组成的单元
                    Slave_Processor(M).Element_Information_List(n,4)=1;
                else %其余情况为混合单元
                    Slave_Processor(M).Element_Information_List(n,4)=2;
                end
                % 把提取的节点保存到单元中
                Slave_Processor(M).Element_Information_List(n,1)=Node_A_ID;
                Slave_Processor(M).Element_Information_List(n,2)=Node_B_ID;
                Slave_Processor(M).Element_Information_List(n,3)=Node_C_ID;
                n=n+1;
                % =========================保存下三角单元==============================
                Node_A_ID=Slave_Processor(M).Node_Information_Matrix_Calculate(i,j+1);
                Node_B_ID=Slave_Processor(M).Node_Information_Matrix_Calculate(i+1,j);
                Node_C_ID=Slave_Processor(M).Node_Information_Matrix_Calculate(i+1,j+1);
                % 判断单元类型
                inner=Slave_Processor(M).Num_Node_Cal_Inner;
                Flag_A=Node_A_ID>inner;
                Flag_B=Node_B_ID>inner;
                Flag_C=Node_C_ID>inner;
                if Flag_A*Flag_B*Flag_C==1            %所有节点的序号都大于内部节点数，则为都是外部节点组成的单元
                    Slave_Processor(M).Element_Information_List(n,4)=3;
                elseif  Flag_A+Flag_B+Flag_C==0       %所有节点的序号都小于内部节点数，则为都是内部节点组成的单元
                    Slave_Processor(M).Element_Information_List(n,4)=1;
                else %其余情况为混合单元
                    Slave_Processor(M).Element_Information_List(n,4)=2;
                end
                % 把提取的节点保存到单元中
                Slave_Processor(M).Element_Information_List(n,1)=Node_A_ID;
                Slave_Processor(M).Element_Information_List(n,2)=Node_B_ID;
                Slave_Processor(M).Element_Information_List(n,3)=Node_C_ID;
                n=n+1;
            end
        end
        M=M+1;
    end
end
%% 载荷向量
% 定义本地的载荷向量
M=1;
for I=1:1:Num_Slave_Processor_Row
    for J=1:1:Num_Slave_Processor_Col
        n=Slave_Processor(M).Num_Node_Cal_Inner;
        m=Slave_Processor(M).Num_Node_Cal_Outer;
        % 对施加随机力载荷
        Slave_Processor(M).P=ones(2*n,1);
        M=M+1;
    end
end
M=1;
% 获取邻居载荷
for I=1:1:Num_Slave_Processor_Row
    for J=1:1:Num_Slave_Processor_Col
        % 更新邻居信息 North
        if Slave_Processor(M).Neighbour_North~=-1
            N=Slave_Processor(M).Neighbour_North;
            g=1;
            for j=1:1:Num_Node_Extend_Y
                for i=1:1:Slave_Processor(M).Num_X_Node_Local %遍历邻居消息传递的每一个点
                    neigh_id_in_local=Slave_Processor(M).Neighbour_North_Index_Calculation_List(g,1);
                    neigh_id_in_neigh=Slave_Processor(M).Neighbour_North_Index_Calculation_List(g,2);
                    if neigh_id_in_local>Slave_Processor(M).Num_Node_Cal_Inner
                        %若属于边界节点，边界节点的载荷信息没有作用，直接忽略
                        g=g+1;
                        continue;
                    else
                        %若属于内部节点，则将信息更新到载荷向量中
                        for i1=1:2
                            Slave_Processor(M).P((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).P((neigh_id_in_neigh-1)*2+i1);
                        end
                    end
                    g=g+1;
                end
            end
        end
        % 更新邻居信息 NorthEast
        if Slave_Processor(M).Neighbour_NorthEast~=-1
            N=Slave_Processor(M).Neighbour_NorthEast;
            g=1;
            for j=1:1:Num_Node_Extend_Y
                for i=1:1:Num_Node_Extend_X %遍历邻居消息传递的每一个点
                    neigh_id_in_local=Slave_Processor(M).Neighbour_NorthEast_Index_Calculation_List(g,1);
                    neigh_id_in_neigh=Slave_Processor(M).Neighbour_NorthEast_Index_Calculation_List(g,2);
                    if neigh_id_in_local>Slave_Processor(M).Num_Node_Cal_Inner
                        %若属于边界节点，边界节点的载荷信息没有作用，直接忽略
                        g=g+1;
                        continue;
                    else
                        %若属于内部节点，则将信息更新到载荷向量中
                        for i1=1:2
                            Slave_Processor(M).P((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).P((neigh_id_in_neigh-1)*2+i1);
                        end
                    end
                    g=g+1;
                end
            end
        end
        % 更新邻居信息 East
        if Slave_Processor(M).Neighbour_East~=-1
            N=Slave_Processor(M).Neighbour_East;
            g=1;
            for j=1:1:Slave_Processor(M).Num_Y_Node_Local
                for i=1:1:Num_Node_Extend_X %遍历邻居消息传递的每一个点
                    neigh_id_in_local=Slave_Processor(M).Neighbour_East_Index_Calculation_List(g,1);
                    neigh_id_in_neigh=Slave_Processor(M).Neighbour_East_Index_Calculation_List(g,2);
                    if neigh_id_in_local>Slave_Processor(M).Num_Node_Cal_Inner
                        %若属于边界节点，边界节点的载荷信息没有作用，直接忽略
                        g=g+1;
                        continue;
                    else
                        %若属于内部节点，则将信息更新到载荷向量中
                        for i1=1:2
                            Slave_Processor(M).P((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).P((neigh_id_in_neigh-1)*2+i1);
                        end
                    end
                    g=g+1;
                end
            end
        end
        % 更新邻居信息 SouthEast
        if Slave_Processor(M).Neighbour_SouthEast~=-1
            N=Slave_Processor(M).Neighbour_SouthEast;
            g=1;
            for j=1:1:Num_Node_Extend_Y
                for i=1:1:Num_Node_Extend_X %遍历邻居消息传递的每一个点
                    neigh_id_in_local=Slave_Processor(M).Neighbour_SouthEast_Index_Calculation_List(g,1);
                    neigh_id_in_neigh=Slave_Processor(M).Neighbour_SouthEast_Index_Calculation_List(g,2);
                    if neigh_id_in_local>Slave_Processor(M).Num_Node_Cal_Inner
                        %若属于边界节点，边界节点的载荷信息没有作用，直接忽略
                        g=g+1;
                        continue;
                    else
                        %若属于内部节点，则将信息更新到载荷向量中
                        for i1=1:2
                            Slave_Processor(M).P((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).P((neigh_id_in_neigh-1)*2+i1);
                        end
                    end
                    g=g+1;
                end
            end
        end
        % 更新邻居信息 South
        if Slave_Processor(M).Neighbour_South~=-1
            N=Slave_Processor(M).Neighbour_South;
            g=1;
            for j=1:1:Num_Node_Extend_Y
                for i=1:1:Slave_Processor(M).Num_X_Node_Local %遍历邻居消息传递的每一个点
                    neigh_id_in_local=Slave_Processor(M).Neighbour_South_Index_Calculation_List(g,1);
                    neigh_id_in_neigh=Slave_Processor(M).Neighbour_South_Index_Calculation_List(g,2);
                    if neigh_id_in_local>Slave_Processor(M).Num_Node_Cal_Inner
                        %若属于边界节点，边界节点的载荷信息没有作用，直接忽略
                        g=g+1;
                        continue;
                    else
                        %若属于内部节点，则将信息更新到载荷向量中
                        for i1=1:2
                            Slave_Processor(M).P((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).P((neigh_id_in_neigh-1)*2+i1);
                        end
                    end
                    g=g+1;
                end
            end
        end
        % 更新邻居信息 SouthWest
        if Slave_Processor(M).Neighbour_SouthWest~=-1
            N=Slave_Processor(M).Neighbour_SouthWest;
            g=1;
            for j=1:1:Num_Node_Extend_Y
                for i=1:1:Num_Node_Extend_X %遍历邻居消息传递的每一个点
                    neigh_id_in_local=Slave_Processor(M).Neighbour_SouthWest_Index_Calculation_List(g,1);
                    neigh_id_in_neigh=Slave_Processor(M).Neighbour_SouthWest_Index_Calculation_List(g,2);
                    if neigh_id_in_local>Slave_Processor(M).Num_Node_Cal_Inner
                        %若属于边界节点，边界节点的载荷信息没有作用，直接忽略
                        g=g+1;
                        continue;
                    else
                        %若属于内部节点，则将信息更新到载荷向量中
                        for i1=1:2
                            Slave_Processor(M).P((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).P((neigh_id_in_neigh-1)*2+i1);
                        end
                    end
                    g=g+1;
                end
            end
        end
        % 更新邻居信息 West
        if Slave_Processor(M).Neighbour_West~=-1
            N=Slave_Processor(M).Neighbour_West;
            g=1;
            for j=1:1:Slave_Processor(M).Num_Y_Node_Local
                for i=1:1:Num_Node_Extend_X %遍历邻居消息传递的每一个点
                    neigh_id_in_local=Slave_Processor(M).Neighbour_West_Index_Calculation_List(g,1);
                    neigh_id_in_neigh=Slave_Processor(M).Neighbour_West_Index_Calculation_List(g,2);
                    if neigh_id_in_local>Slave_Processor(M).Num_Node_Cal_Inner
                        %若属于边界节点，边界节点的载荷信息没有作用，直接忽略
                        g=g+1;
                        continue;
                    else
                        %若属于内部节点，则将信息更新到载荷向量中
                        for i1=1:2
                            Slave_Processor(M).P((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).P((neigh_id_in_neigh-1)*2+i1);
                        end
                    end
                    g=g+1;
                end
            end
        end
        % 更新邻居信息 NorthWest
        if Slave_Processor(M).Neighbour_NorthWest~=-1
            N=Slave_Processor(M).Neighbour_NorthWest;
            g=1;
            for j=1:1:Num_Node_Extend_Y
                for i=1:1:Num_Node_Extend_X %遍历邻居消息传递的每一个点
                    neigh_id_in_local=Slave_Processor(M).Neighbour_NorthWest_Index_Calculation_List(g,1);
                    neigh_id_in_neigh=Slave_Processor(M).Neighbour_NorthWest_Index_Calculation_List(g,2);
                    if neigh_id_in_local>Slave_Processor(M).Num_Node_Cal_Inner
                        %若属于边界节点，边界节点的载荷信息没有作用，直接忽略
                        g=g+1;
                        continue;
                    else
                        %若属于内部节点，则将信息更新到载荷向量中
                        for i1=1:2
                            Slave_Processor(M).P((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).P((neigh_id_in_neigh-1)*2+i1);
                        end
                    end
                    g=g+1;
                end
            end
        end
        M=M+1;
    end
end

%% 自由度约束
%每一个从处理器定义本地自由度约束
M=1;
for I=1:1:Num_Slave_Processor_Row
    for J=1:1:Num_Slave_Processor_Col
        n=Slave_Processor(M).Num_X_Node_Cal*Slave_Processor(M).Num_Y_Node_Cal;
        Slave_Processor(M).Fix_DoF=zeros(n*2,2);
        if I==1
            % 约束最北方节点的自由度
            for i=1:1:Slave_Processor(M).Num_X_Node_Local*2
                Slave_Processor(M).Fix_DoF(i,1)=1;  %标记需要进行位移约束
                Slave_Processor(M).Fix_DoF(i,2)=0;  %标记位移约束的数值
            end
        end
        M=M+1;
    end
end
%每一个从处理器从邻居节点获取邻居自由度约束
M=1;
for I=1:1:Num_Slave_Processor_Row
    for J=1:1:Num_Slave_Processor_Col
        % 更新邻居信息 North
        if Slave_Processor(M).Neighbour_North~=-1
            N=Slave_Processor(M).Neighbour_North;
            g=1;
            for j=1:1:Num_Node_Extend_Y
                for i=1:1:Slave_Processor(M).Num_X_Node_Local %遍历邻居消息传递的每一个点
                    neigh_id_in_local=Slave_Processor(M).Neighbour_North_Index_Calculation_List(g,1);
                    neigh_id_in_neigh=Slave_Processor(M).Neighbour_North_Index_Calculation_List(g,2);
%                     if neigh_id_in_local<Slave_Processor(M).Num_Node_Cal_Inner
                        % 获取该邻居的自由度信息
                        for i2=1:1:2
                            dof_id_neighbour=2*neigh_id_in_neigh-2+i2;
                            dof_id_local=2*neigh_id_in_local-2+i2;
                            if Slave_Processor(N).Fix_DoF(dof_id_neighbour,1)==1
                                Slave_Processor(M).Fix_DoF(dof_id_local,1)=1;
                                Slave_Processor(M).Fix_DoF(dof_id_local,2)=...
                                    Slave_Processor(N).Fix_DoF(dof_id_neighbour,2);
                            end
                        end
%                     end
                    g=g+1;
                end
            end
        end
        % 更新邻居信息 NorthEast
        if Slave_Processor(M).Neighbour_NorthEast~=-1
            N=Slave_Processor(M).Neighbour_NorthEast;
            g=1;
            for j=1:1:Num_Node_Extend_Y
                for i=1:1:Num_Node_Extend_X %遍历邻居消息传递的每一个点
                    neigh_id_in_local=Slave_Processor(M).Neighbour_NorthEast_Index_Calculation_List(g,1);
                    neigh_id_in_neigh=Slave_Processor(M).Neighbour_NorthEast_Index_Calculation_List(g,2);
%                     if neigh_id_in_local<Slave_Processor(M).Num_Node_Cal_Inner
                        % 获取该邻居的自由度信息
                        for i2=1:1:2
                            dof_id_neighbour=2*neigh_id_in_neigh-2+i2;
                            dof_id_local=2*neigh_id_in_local-2+i2;
                            if Slave_Processor(N).Fix_DoF(dof_id_neighbour,1)==1
                                Slave_Processor(M).Fix_DoF(dof_id_local,1)=1;
                                Slave_Processor(M).Fix_DoF(dof_id_local,2)=...
                                    Slave_Processor(N).Fix_DoF(dof_id_neighbour,2);
                            end
                        end
%                     end
                    g=g+1;
                end
            end
        end
        % 更新邻居信息 East
        if Slave_Processor(M).Neighbour_East~=-1
            N=Slave_Processor(M).Neighbour_East;
            g=1;
            for j=1:1:Slave_Processor(M).Num_Y_Node_Local
                for i=1:1:Num_Node_Extend_X %遍历邻居消息传递的每一个点
                    neigh_id_in_local=Slave_Processor(M).Neighbour_East_Index_Calculation_List(g,1);
                    neigh_id_in_neigh=Slave_Processor(M).Neighbour_East_Index_Calculation_List(g,2);
%                     if neigh_id_in_local<Slave_Processor(M).Num_Node_Cal_Inner
                        % 获取该邻居的自由度信息
                        for i2=1:1:2
                            dof_id_neighbour=2*neigh_id_in_neigh-2+i2;
                            dof_id_local=2*neigh_id_in_local-2+i2;
                            if Slave_Processor(N).Fix_DoF(dof_id_neighbour,1)==1
                                Slave_Processor(M).Fix_DoF(dof_id_local,1)=1;
                                Slave_Processor(M).Fix_DoF(dof_id_local,2)=...
                                    Slave_Processor(N).Fix_DoF(dof_id_neighbour,2);
                            end
                        end
%                     end
                    g=g+1;
                end
            end
        end
        % 更新邻居信息 SouthEast
        if Slave_Processor(M).Neighbour_SouthEast~=-1
            N=Slave_Processor(M).Neighbour_SouthEast;
            g=1;
            for j=1:1:Num_Node_Extend_Y
                for i=1:1:Num_Node_Extend_X %遍历邻居消息传递的每一个点
                    neigh_id_in_local=Slave_Processor(M).Neighbour_SouthEast_Index_Calculation_List(g,1);
                    neigh_id_in_neigh=Slave_Processor(M).Neighbour_SouthEast_Index_Calculation_List(g,2);
%                     if neigh_id_in_local<Slave_Processor(M).Num_Node_Cal_Inner
                        % 获取该邻居的自由度信息
                        for i2=1:1:2
                            dof_id_neighbour=2*neigh_id_in_neigh-2+i2;
                            dof_id_local=2*neigh_id_in_local-2+i2;
                            if Slave_Processor(N).Fix_DoF(dof_id_neighbour,1)==1
                                Slave_Processor(M).Fix_DoF(dof_id_local,1)=1;
                                Slave_Processor(M).Fix_DoF(dof_id_local,2)=...
                                    Slave_Processor(N).Fix_DoF(dof_id_neighbour,2);
                            end
                        end
%                     end
                    g=g+1;
                end
            end
        end
        % 更新邻居信息 South
        if Slave_Processor(M).Neighbour_South~=-1
            N=Slave_Processor(M).Neighbour_South;
            g=1;
            for j=1:1:Num_Node_Extend_Y
                for i=1:1:Slave_Processor(M).Num_X_Node_Local %遍历邻居消息传递的每一个点
                    neigh_id_in_local=Slave_Processor(M).Neighbour_South_Index_Calculation_List(g,1);
                    neigh_id_in_neigh=Slave_Processor(M).Neighbour_South_Index_Calculation_List(g,2);
%                     if neigh_id_in_local<Slave_Processor(M).Num_Node_Cal_Inner
                        % 获取该邻居的自由度信息
                        for i2=1:1:2
                            dof_id_neighbour=2*neigh_id_in_neigh-2+i2;
                            dof_id_local=2*neigh_id_in_local-2+i2;
                            if Slave_Processor(N).Fix_DoF(dof_id_neighbour,1)==1
                                Slave_Processor(M).Fix_DoF(dof_id_local,1)=1;
                                Slave_Processor(M).Fix_DoF(dof_id_local,2)=...
                                    Slave_Processor(N).Fix_DoF(dof_id_neighbour,2);
                            end
                        end
%                     end
                    g=g+1;
                end
            end
        end
        % 更新邻居信息 SouthWest
        if Slave_Processor(M).Neighbour_SouthWest~=-1
            N=Slave_Processor(M).Neighbour_SouthWest;
            g=1;
            for j=1:1:Num_Node_Extend_Y
                for i=1:1:Num_Node_Extend_X %遍历邻居消息传递的每一个点
                    neigh_id_in_local=Slave_Processor(M).Neighbour_SouthWest_Index_Calculation_List(g,1);
                    neigh_id_in_neigh=Slave_Processor(M).Neighbour_SouthWest_Index_Calculation_List(g,2);
%                     if neigh_id_in_local<Slave_Processor(M).Num_Node_Cal_Inner
                        % 获取该邻居的自由度信息
                        for i2=1:1:2
                            dof_id_neighbour=2*neigh_id_in_neigh-2+i2;
                            dof_id_local=2*neigh_id_in_local-2+i2;
                            if Slave_Processor(N).Fix_DoF(dof_id_neighbour,1)==1
                                Slave_Processor(M).Fix_DoF(dof_id_local,1)=1;
                                Slave_Processor(M).Fix_DoF(dof_id_local,2)=...
                                    Slave_Processor(N).Fix_DoF(dof_id_neighbour,2);
                            end
                        end
%                     end
                    g=g+1;
                end
            end
        end
        % 更新邻居信息 West
        if Slave_Processor(M).Neighbour_West~=-1
            N=Slave_Processor(M).Neighbour_West;
            g=1;
            for j=1:1:Slave_Processor(M).Num_Y_Node_Local
                for i=1:1:Num_Node_Extend_X %遍历邻居消息传递的每一个点
                    neigh_id_in_local=Slave_Processor(M).Neighbour_West_Index_Calculation_List(g,1);
                    neigh_id_in_neigh=Slave_Processor(M).Neighbour_West_Index_Calculation_List(g,2);
%                     if neigh_id_in_local<Slave_Processor(M).Num_Node_Cal_Inner
                        % 获取该邻居的自由度信息
                        for i2=1:1:2
                            dof_id_neighbour=2*neigh_id_in_neigh-2+i2;
                            dof_id_local=2*neigh_id_in_local-2+i2;
                            if Slave_Processor(N).Fix_DoF(dof_id_neighbour,1)==1
                                Slave_Processor(M).Fix_DoF(dof_id_local,1)=1;
                                Slave_Processor(M).Fix_DoF(dof_id_local,2)=...
                                    Slave_Processor(N).Fix_DoF(dof_id_neighbour,2);
                            end
                        end
%                     end
                    g=g+1;
                end
            end
        end
        % 更新邻居信息 NorthWest
        if Slave_Processor(M).Neighbour_NorthWest~=-1
            N=Slave_Processor(M).Neighbour_NorthWest;
            g=1;
            for j=1:1:Num_Node_Extend_Y
                for i=1:1:Num_Node_Extend_X %遍历邻居消息传递的每一个点
                    neigh_id_in_local=Slave_Processor(M).Neighbour_NorthWest_Index_Calculation_List(g,1);
                    neigh_id_in_neigh=Slave_Processor(M).Neighbour_NorthWest_Index_Calculation_List(g,2);
%                     if neigh_id_in_local<Slave_Processor(M).Num_Node_Cal_Inner
                        % 获取该邻居的自由度信息
                        for i2=1:1:2
                            dof_id_neighbour=2*neigh_id_in_neigh-2+i2;
                            dof_id_local=2*neigh_id_in_local-2+i2;
                            if Slave_Processor(N).Fix_DoF(dof_id_neighbour,1)==1
                                Slave_Processor(M).Fix_DoF(dof_id_local,1)=1;
                                Slave_Processor(M).Fix_DoF(dof_id_local,2)=...
                                    Slave_Processor(N).Fix_DoF(dof_id_neighbour,2);
                            end
                        end
%                     end
                    g=g+1;
                end
            end
        end
        M=M+1;
    end
end
%% 刚度信息计算
% 每一个从处理器计算单元刚度矩阵
M=1;
for I=1:1:Num_Slave_Processor_Row
    for J=1:1:Num_Slave_Processor_Col
        % 建立单元刚度矩阵的储存空间
        Slave_Processor(M).Element_Stiffness_Matrix=cell(Slave_Processor(M).Num_Element_Calculate,1);
        % 对每一个单元循环 保存信息
        for e=1:1:Slave_Processor(M).Num_Element_Calculate
            %单元中节点 1 2 3
            % 得到每个节点横纵坐标
            x=zeros(3,1);
            y=zeros(3,1);
            for i=1:1:3
                Node_id=Slave_Processor(M).Element_Information_List(e,i);
                x(i)=Slave_Processor(M).Node_Coordinates_List_Calculate(Node_id,1);
                y(i)=Slave_Processor(M).Node_Coordinates_List_Calculate(Node_id,2);
            end
            a=zeros(3,1);
            b=zeros(3,1);
            c=zeros(3,1);
            for i=1:1:3
                if(i==1);j=2;m=3;end
                if(i==2);j=3;m=1;end
                if(i==3);j=1;m=2;end
                a(i)=x(j)*y(m)-x(m)*y(j);
                b(i)=y(j)-y(m);
                c(i)=-x(j)+x(m);
            end
            A=1/2*(b(i)*c(j)-b(j)*c(i));
            K0=zeros(6,6);
            for r=1:1:3
                for s=1:1:3
                    K0(2*r-1,2*s-1)=b(r)*b(s)+(1-nu_0)/2*c(r)*c(s);
                    K0(2*r-1,2*s)=nu_0*b(r)*c(s)+(1-nu_0)/2*c(r)*b(s);
                    K0(2*r,2*s-1)=nu_0*c(r)*b(s)+(1-nu_0)/2*b(r)*c(s);
                    K0(2*r,2*s)=c(r)*c(s)+(1-nu_0)/2*b(r)*b(s);
                end
            end
            K0=K0*E_0*Thickness/(4*A*(1-nu_0^2));
            Slave_Processor(M).Element_Stiffness_Matrix{e}=K0;
        end
        M=M+1;
    end
end
% 从处理器对每一个单元刚度矩阵进行叠加 得到K_nxn Inner 和 K_nxn Outer
M=1;
for I=1:1:Num_Slave_Processor_Row
    for J=1:1:Num_Slave_Processor_Col
        % 建立保存空间
        Slave_Processor(M).Global_Stiffness_Matrix_nxn=zeros(Slave_Processor(M).Num_Node_Cal_Inner*2,Slave_Processor(M).Num_Node_Cal_Inner*2);
        Slave_Processor(M).Global_Stiffness_Matrix_nxn_Inner=zeros(Slave_Processor(M).Num_Node_Cal_Inner*2,Slave_Processor(M).Num_Node_Cal_Inner*2);
        Slave_Processor(M).Global_Stiffness_Matrix_nxn_Outer=zeros(Slave_Processor(M).Num_Node_Cal_Inner*2,Slave_Processor(M).Num_Node_Cal_Inner*2);
        Slave_Processor(M).Global_Stiffness_Matrix_mxn=zeros(Slave_Processor(M).Num_Node_Cal_Inner*2,Slave_Processor(M).Num_Node_Cal_Outer*2);
        % 从单元刚度矩阵进行叠加 得到 K_nxn_Inner K_nxn_Outer K_nxm
        for e=1:1:Slave_Processor(M).Num_Element_Calculate
            % ===================1.获取单元类型===================
            element_type=Slave_Processor(M).Element_Information_List(e,4);
            
            % 单元节点 A B C
            Node_id=zeros(3,1);
            Node_id(1)=Slave_Processor(M).Element_Information_List(e,1);
            Node_id(2)=Slave_Processor(M).Element_Information_List(e,2);
            Node_id(3)=Slave_Processor(M).Element_Information_List(e,3);
            % 单元刚度矩阵
            %  K=[K_AA K_AB K_AC
            %     K_BA K_BB K_BC
            %     K_CA K_CB K_CC
            % ]; 其中每个元素都是2x2的
            
            % ===================2.从单元刚度矩阵列表中得到单元刚度矩阵===================
            K0=Slave_Processor(M).Element_Stiffness_Matrix{e};
            % ===================3.根据单元类型进行叠加===================
            % 若属于第三种单元类型，则不需要叠加
            if element_type==3
                continue;
                % 若属于第一种单元类型，则叠加在 K_nxn_Inner上
            elseif element_type==1
                for r=1:1:3   % 循环叠加单元刚度矩阵中9个2x2的矩阵
                    for s=1:1:3
                        Node_R=Node_id(r);
                        Node_S=Node_id(s);
                        % 分别叠加每个2x2矩阵中的元素
                        for i=1:1:2
                            for j=1:1:2
                                % 计算得到对应元素在总体刚度矩阵中的行列
                                row=(Node_R-1)*2+i;
                                col=(Node_S-1)*2+j;
                                Slave_Processor(M).Global_Stiffness_Matrix_nxn_Inner(row,col)=Slave_Processor(M).Global_Stiffness_Matrix_nxn_Inner(row,col)+K0(2*(r-1)+i,2*(s-1)+j);
                            end
                        end
                    end
                end
            else % 如果属于混合单元，则叠加在K_nxn_Outer 和 K_nxm 上
                n=Slave_Processor(M).Num_Node_Cal_Inner;
                for r=1:1:3   % 循环叠加单元刚度矩阵中9个2x2的矩阵
                    for s=1:1:3
                        Node_R=Node_id(r);
                        Node_S=Node_id(s);
                        %分四种情况讨论：R外S外，R外S内，R内S外，R内S内
                        if Node_R>n %若R外，则不叠加
                            continue;
                        else
                            if Node_S>n  %在R内S外，则叠加在K_nxm_上
                                Node_S=Node_S-n;
                                for i=1:1:2
                                    for j=1:1:2
                                        % 计算得到对应元素在总体刚度矩阵中的行列
                                        row=(Node_R-1)*2+i;
                                        col=(Node_S-1)*2+j;
                                        Slave_Processor(M).Global_Stiffness_Matrix_mxn(row,col)=Slave_Processor(M).Global_Stiffness_Matrix_mxn(row,col)+K0(2*(r-1)+i,2*(s-1)+j);
                                    end
                                end
                            else  %在R内S内，则叠加在K_nxx_Outer上
                                for i=1:1:2
                                    for j=1:1:2
                                        % 计算得到对应元素在总体刚度矩阵中的行列
                                        row=(Node_R-1)*2+i;
                                        col=(Node_S-1)*2+j;
                                        Slave_Processor(M).Global_Stiffness_Matrix_nxn_Outer(row,col)=Slave_Processor(M).Global_Stiffness_Matrix_nxn_Outer(row,col)+K0(2*(r-1)+i,2*(s-1)+j);
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
        % 从K_nxn_Inner 和 K_nxn_Outer 得到 K_nxn
        Slave_Processor(M).Global_Stiffness_Matrix_nxn=Slave_Processor(M).Global_Stiffness_Matrix_nxn_Inner+Slave_Processor(M).Global_Stiffness_Matrix_nxn_Outer;
        M=M+1;
    end
end
% 基于位移约束，对刚度矩阵进行修正
M=1;
for I=1:1:Num_Slave_Processor_Row
    for J=1:1:Num_Slave_Processor_Col
        for n=1:1:Slave_Processor(M).Num_Node_Cal_Inner*2
            %基于置大数法-------------------------------
            %约束自由度
            if Slave_Processor(M).Fix_DoF(n,1)~=1
                continue;
            end
            u_constrain=Slave_Processor(M).Fix_DoF(n,2);
%             Slave_Processor(M).Global_Stiffness_Matrix_nxn_Inner(n,n)=INF*Slave_Processor(M).Global_Stiffness_Matrix_nxn_Inner(n,n);
%             Slave_Processor(M).Global_Stiffness_Matrix_nxn_Outer(n,n)=INF*Slave_Processor(M).Global_Stiffness_Matrix_nxn_Outer(n,n);
%             Slave_Processor(M).Global_Stiffness_Matrix_nxn(n,n)=INF*Slave_Processor(M).Global_Stiffness_Matrix_nxn(n,n);
%             Slave_Processor(M).P(n)=INF*u_constrain;
            Slave_Processor(M).Global_Stiffness_Matrix_nxn_Inner(n,:)=0;
            Slave_Processor(M).Global_Stiffness_Matrix_nxn_Outer(n,:)=0;
            Slave_Processor(M).Global_Stiffness_Matrix_nxn(n,:)=0;
            Slave_Processor(M).Global_Stiffness_Matrix_nxn_Inner(:,n)=0;
            Slave_Processor(M).Global_Stiffness_Matrix_nxn_Outer(:,n)=0;
            Slave_Processor(M).Global_Stiffness_Matrix_nxn(:,n)=0;
            Slave_Processor(M).Global_Stiffness_Matrix_nxn_Inner(n,n)=1;
            Slave_Processor(M).Global_Stiffness_Matrix_nxn_Outer(n,n)=1;
            Slave_Processor(M).Global_Stiffness_Matrix_nxn(n,n)=1;
            Slave_Processor(M).Global_Stiffness_Matrix_mxn(n,:)=0;
            Slave_Processor(M).P(n)=0;
           
        end
        for n=1:1:Slave_Processor(M).Num_Node_Cal_Outer*2
            if Slave_Processor(M).Fix_DoF(n+Slave_Processor(M).Num_Node_Cal_Inner*2,1)~=1
                continue;
            end
            Slave_Processor(M).Global_Stiffness_Matrix_mxn(:,n)=0;
        end
        M=M+1;
    end
end
%建立邻居相连表，表中包含两组数
%本地处理器从属点，邻居处理器从属点，相连刚度
M=1;
for I=1:1:Num_Slave_Processor_Row
    for J=1:1:Num_Slave_Processor_Col
        nci=Slave_Processor(M).Num_Node_Cal_Inner;
        % 建立邻居相连表的空间
        n=Slave_Processor(M).Num_X_Node_Local;
        m=Slave_Processor(M).Num_Y_Node_Local;
        switch(Slave_Processor(M).Processor_Type)
            case 1
                g=(2*n-1)+(2*m-1);
            case 2
                g=(2*m-1)*2+(2*n-1)+1;
            case 3
                g=(2*m-1)+(2*n-1)+1;
            case 4
                g=(2*m-1)+(2*n-1)*2+1;
            case 5
                g=(2*m-1)*2+(2*n-1)*2+2;
            case 6
                g=(2*n-1)*2+(2*m-1)+1;
            case 7
                g=(2*m-1)+(2*n-1)+1;
            case 8
                g=(2*m-1)*2+(2*n-1)+1;
            case 9
                g=(2*m-1)+(2*n-1);
        end
        Slave_Processor(M).Neighbour_Link_List=zeros(g,3);
        Slave_Processor(M).Neighbour_Link_Stiffness=cell(g,1);
        g=1;
        %======================================================
        %       获取邻居的从属点，获取相连刚度 North
        %======================================================
        if Slave_Processor(M).Neighbour_North~=-1
            %获取目标点正北方点的直接相连点的连接关系
            N=Slave_Processor(M).Neighbour_North;
            for i=1:1:Slave_Processor(M).Num_X_Node_Local
                % 获取连接点的序号
                col_local=Slave_Processor(M).Local_Node_Calculation_X_Location+i-1;
                row_local=Slave_Processor(M).Local_Node_Calculation_Y_Location;
                row_neighbour=row_local-1;
                col_neighbour=col_local;
                local_id=Slave_Processor(M).Node_Information_Matrix_Calculate(row_local,col_local);
                neighbour_id=Slave_Processor(M).Node_Information_Matrix_Calculate(row_neighbour,col_neighbour);
                neighbour_id_other_processor=Slave_Processor(N).Num_Node_Local-Slave_Processor(N).Num_X_Node_Local+i;
                % 序号保存到表中
                Slave_Processor(M).Neighbour_Link_List(g,1)=local_id;
                Slave_Processor(M).Neighbour_Link_List(g,2)=neighbour_id;
                Slave_Processor(M).Neighbour_Link_List(g,3)=neighbour_id_other_processor;
                % 获取该目标点对应的刚度矩阵分量2x2
                k=zeros(2,2);
                if neighbour_id > nci
                    neighbour_id=neighbour_id-nci; %此处应该减去的是非边界点
                    for i1=1:1:2
                        for i2=1:1:2
                            k(i1,i2)=Slave_Processor(M).Global_Stiffness_Matrix_mxn(local_id*2-2+i1,neighbour_id*2-2+i2);
                        end
                    end
                else
                    for i1=1:1:2
                        for i2=1:1:2
                            k(i1,i2)=Slave_Processor(M).Global_Stiffness_Matrix_nxn(local_id*2-2+i1,neighbour_id*2-2+i2);
                        end
                    end
                end
                % 刚度分量保存到表中
                Slave_Processor(M).Neighbour_Link_Stiffness{g}=k;
                g=g+1;
            end
            %获取目标点东北方点的直接相连点在该从处理器中的序号
            for i=1:1:Slave_Processor(M).Num_X_Node_Local-1
                % 获取连接点的序号
                col_local=Slave_Processor(M).Local_Node_Calculation_X_Location+i-1;
                row_local=Slave_Processor(M).Local_Node_Calculation_Y_Location;
                row_neighbour=row_local-1;
                col_neighbour=col_local+1;
                local_id=Slave_Processor(M).Node_Information_Matrix_Calculate(row_local,col_local);
                neighbour_id=Slave_Processor(M).Node_Information_Matrix_Calculate(row_neighbour,col_neighbour);
                neighbour_id_other_processor=Slave_Processor(N).Num_Node_Local-Slave_Processor(N).Num_X_Node_Local+i+1;
                % 序号保存到表中
                Slave_Processor(M).Neighbour_Link_List(g,1)=local_id;
                Slave_Processor(M).Neighbour_Link_List(g,2)=neighbour_id;
                Slave_Processor(M).Neighbour_Link_List(g,3)=neighbour_id_other_processor;
                % 获取该目标点对应的刚度矩阵分量2x2
                k=zeros(2,2);
                if neighbour_id > nci
                    neighbour_id=neighbour_id-nci;
                    for i1=1:1:2
                        for i2=1:1:2
                            k(i1,i2)=Slave_Processor(M).Global_Stiffness_Matrix_mxn(local_id*2-2+i1,neighbour_id*2-2+i2);
                        end
                    end
                else
                    for i1=1:1:2
                        for i2=1:1:2
                            k(i1,i2)=Slave_Processor(M).Global_Stiffness_Matrix_nxn(local_id*2-2+i1,neighbour_id*2-2+i2);
                        end
                    end
                end
                % 刚度分量保存到表中
                Slave_Processor(M).Neighbour_Link_Stiffness{g}=k;
                g=g+1;
            end
        end
        %======================================================
        %       获取邻居的从属点，获取相连刚度 NorthEast
        %======================================================
        N=Slave_Processor(M).Neighbour_NorthEast;
        if Slave_Processor(M).Neighbour_NorthEast~=-1
            %获取目标点东北方点的直接相连点在该从处理器中的序号
            for i=Slave_Processor(M).Num_X_Node_Local
                % 获取连接点的序号
                col_local=Slave_Processor(M).Local_Node_Calculation_X_Location+i-1;
                row_local=Slave_Processor(M).Local_Node_Calculation_Y_Location;
                row_neighbour=row_local-1;
                col_neighbour=col_local+1;
                local_id=Slave_Processor(M).Node_Information_Matrix_Calculate(row_local,col_local);
                neighbour_id=Slave_Processor(M).Node_Information_Matrix_Calculate(row_neighbour,col_neighbour);
                neighbour_id_other_processor=Slave_Processor(N).Num_Node_Local-Slave_Processor(N).Num_X_Node_Local+1;
                % 序号保存到表中
                Slave_Processor(M).Neighbour_Link_List(g,1)=local_id;
                Slave_Processor(M).Neighbour_Link_List(g,2)=neighbour_id;
                Slave_Processor(M).Neighbour_Link_List(g,3)=neighbour_id_other_processor;
                % 获取该目标点对应的刚度矩阵分量2x2
                k=zeros(2,2);
                if neighbour_id > nci
                    neighbour_id=neighbour_id-nci;
                    for i1=1:1:2
                        for i2=1:1:2
                            k(i1,i2)=Slave_Processor(M).Global_Stiffness_Matrix_mxn(local_id*2-2+i1,neighbour_id*2-2+i2);
                        end
                    end
                else
                    for i1=1:1:2
                        for i2=1:1:2
                            k(i1,i2)=Slave_Processor(M).Global_Stiffness_Matrix_nxn(local_id*2-2+i1,neighbour_id*2-2+i2);
                        end
                    end
                end
                % 刚度分量保存到表中
                Slave_Processor(M).Neighbour_Link_Stiffness{g}=k;
                g=g+1;
            end
        end
        %======================================================
        %       获取邻居的从属点，获取相连刚度 East
        %======================================================
        N=Slave_Processor(M).Neighbour_East;
        if Slave_Processor(M).Neighbour_East~=-1
            %获取目标点正东方点的直接相连点的连接关系
            for i=1:1:Slave_Processor(M).Num_Y_Node_Local
                % 获取连接点的序号
                col_local=Slave_Processor(M).Local_Node_Calculation_X_Location+Slave_Processor(M).Num_X_Node_Local-1;
                row_local=Slave_Processor(M).Local_Node_Calculation_Y_Location+i-1;
                row_neighbour=row_local;
                col_neighbour=col_local+1;
                local_id=Slave_Processor(M).Node_Information_Matrix_Calculate(row_local,col_local);
                neighbour_id=Slave_Processor(M).Node_Information_Matrix_Calculate(row_neighbour,col_neighbour);
                neighbour_id_other_processor=(i-1)*Slave_Processor(N).Num_X_Node_Local+1;
                % 序号保存到表中
                Slave_Processor(M).Neighbour_Link_List(g,1)=local_id;
                Slave_Processor(M).Neighbour_Link_List(g,2)=neighbour_id;
                Slave_Processor(M).Neighbour_Link_List(g,3)=neighbour_id_other_processor;
                % 获取该目标点对应的刚度矩阵分量2x2
                k=zeros(2,2);
                if neighbour_id > nci
                    neighbour_id=neighbour_id-nci;
                    for i1=1:1:2
                        for i2=1:1:2
                            k(i1,i2)=Slave_Processor(M).Global_Stiffness_Matrix_mxn(local_id*2-2+i1,neighbour_id*2-2+i2);
                        end
                    end
                else
                    for i1=1:1:2
                        for i2=1:1:2
                            k(i1,i2)=Slave_Processor(M).Global_Stiffness_Matrix_nxn(local_id*2-2+i1,neighbour_id*2-2+i2);
                        end
                    end
                end
                % 刚度分量保存到表中
                Slave_Processor(M).Neighbour_Link_Stiffness{g}=k;
                g=g+1;
            end
            %获取目标点东北方点的直接相连点在该从处理器中的序号
            for i=2:1:Slave_Processor(M).Num_Y_Node_Local
                % 获取连接点的序号
                col_local=Slave_Processor(M).Local_Node_Calculation_X_Location+Slave_Processor(M).Num_X_Node_Local-1;
                row_local=Slave_Processor(M).Local_Node_Calculation_Y_Location+i-1;
                row_neighbour=row_local-1;
                col_neighbour=col_local+1;
                local_id=Slave_Processor(M).Node_Information_Matrix_Calculate(row_local,col_local);
                neighbour_id=Slave_Processor(M).Node_Information_Matrix_Calculate(row_neighbour,col_neighbour);
                neighbour_id_other_processor=(i-2)*Slave_Processor(N).Num_X_Node_Local+1;
                % 序号保存到表中
                Slave_Processor(M).Neighbour_Link_List(g,1)=local_id;
                Slave_Processor(M).Neighbour_Link_List(g,2)=neighbour_id;
                Slave_Processor(M).Neighbour_Link_List(g,3)=neighbour_id_other_processor;
                % 获取该目标点对应的刚度矩阵分量2x2
                k=zeros(2,2);
                if neighbour_id > nci
                    neighbour_id=neighbour_id-nci;
                    for i1=1:1:2
                        for i2=1:1:2
                            k(i1,i2)=Slave_Processor(M).Global_Stiffness_Matrix_mxn(local_id*2-2+i1,neighbour_id*2-2+i2);
                        end
                    end
                else
                    for i1=1:1:2
                        for i2=1:1:2
                            k(i1,i2)=Slave_Processor(M).Global_Stiffness_Matrix_nxn(local_id*2-2+i1,neighbour_id*2-2+i2);
                        end
                    end
                end
                % 刚度分量保存到表中
                Slave_Processor(M).Neighbour_Link_Stiffness{g}=k;
                g=g+1;
            end
        end
        %======================================================
        %       获取邻居的从属点，获取相连刚度 SouthEast 无
        %======================================================
        
        %======================================================
        %       获取邻居的从属点，获取相连刚度 South
        %======================================================
        N=Slave_Processor(M).Neighbour_South;
        if Slave_Processor(M).Neighbour_South~=-1
            %获取目标点正南方点的直接相连点的连接关系
            for i=1:1:Slave_Processor(M).Num_X_Node_Local
                % 获取连接点的序号
                col_local=Slave_Processor(M).Local_Node_Calculation_X_Location+i-1;
                row_local=Slave_Processor(M).Local_Node_Calculation_Y_Location+Slave_Processor(M).Num_Y_Node_Local-1;
                row_neighbour=row_local+1;
                col_neighbour=col_local;
                local_id=Slave_Processor(M).Node_Information_Matrix_Calculate(row_local,col_local);
                neighbour_id=Slave_Processor(M).Node_Information_Matrix_Calculate(row_neighbour,col_neighbour);
                neighbour_id_other_processor=i;
                % 序号保存到表中
                Slave_Processor(M).Neighbour_Link_List(g,1)=local_id;
                Slave_Processor(M).Neighbour_Link_List(g,2)=neighbour_id;
                Slave_Processor(M).Neighbour_Link_List(g,3)=neighbour_id_other_processor;
                % 获取该目标点对应的刚度矩阵分量2x2
                k=zeros(2,2);
                if neighbour_id > nci
                    neighbour_id=neighbour_id-nci;
                    for i1=1:1:2
                        for i2=1:1:2
                            k(i1,i2)=Slave_Processor(M).Global_Stiffness_Matrix_mxn(local_id*2-2+i1,neighbour_id*2-2+i2);
                        end
                    end
                else
                    for i1=1:1:2
                        for i2=1:1:2
                            k(i1,i2)=Slave_Processor(M).Global_Stiffness_Matrix_nxn(local_id*2-2+i1,neighbour_id*2-2+i2);
                        end
                    end
                end
                % 刚度分量保存到表中
                Slave_Processor(M).Neighbour_Link_Stiffness{g}=k;
                g=g+1;
            end
            %获取目标点西南方向的直接相连点在该从处理器中的序号
            for i=2:1:Slave_Processor(M).Num_X_Node_Local
                % 获取连接点的序号
                col_local=Slave_Processor(M).Local_Node_Calculation_X_Location+i-1;
                row_local=Slave_Processor(M).Local_Node_Calculation_Y_Location+Slave_Processor(M).Num_Y_Node_Local-1;
                row_neighbour=row_local+1;
                col_neighbour=col_local-1;
                local_id=Slave_Processor(M).Node_Information_Matrix_Calculate(row_local,col_local);
                neighbour_id=Slave_Processor(M).Node_Information_Matrix_Calculate(row_neighbour,col_neighbour);
                neighbour_id_other_processor=i-1;
                % 序号保存到表中
                Slave_Processor(M).Neighbour_Link_List(g,1)=local_id;
                Slave_Processor(M).Neighbour_Link_List(g,2)=neighbour_id;
                Slave_Processor(M).Neighbour_Link_List(g,3)=neighbour_id_other_processor;
                % 获取该目标点对应的刚度矩阵分量2x2
                k=zeros(2,2);
                if neighbour_id > nci
                    neighbour_id=neighbour_id-nci;
                    for i1=1:1:2
                        for i2=1:1:2
                            k(i1,i2)=Slave_Processor(M).Global_Stiffness_Matrix_mxn(local_id*2-2+i1,neighbour_id*2-2+i2);
                        end
                    end
                else
                    for i1=1:1:2
                        for i2=1:1:2
                            k(i1,i2)=Slave_Processor(M).Global_Stiffness_Matrix_nxn(local_id*2-2+i1,neighbour_id*2-2+i2);
                        end
                    end
                end
                % 刚度分量保存到表中
                Slave_Processor(M).Neighbour_Link_Stiffness{g}=k;
                g=g+1;
            end
        end
        
        %======================================================
        %       获取邻居的从属点，获取相连刚度 SouthWest
        %======================================================
        N=Slave_Processor(M).Neighbour_SouthWest;
        if Slave_Processor(M).Neighbour_SouthWest~=-1
            %获取目标点西南方点的直接相连点在该从处理器中的序号
            for i=1
                % 获取连接点的序号
                col_local=Slave_Processor(M).Local_Node_Calculation_X_Location+i-1;
                row_local=Slave_Processor(M).Local_Node_Calculation_Y_Location+Slave_Processor(M).Num_Y_Node_Local-1;
                
                row_neighbour=row_local+1;
                col_neighbour=col_local-1;
                local_id=Slave_Processor(M).Node_Information_Matrix_Calculate(row_local,col_local);
                neighbour_id=Slave_Processor(M).Node_Information_Matrix_Calculate(row_neighbour,col_neighbour);
                neighbour_id_other_processor=Slave_Processor(N).Num_X_Node_Local;
                % 序号保存到表中
                Slave_Processor(M).Neighbour_Link_List(g,1)=local_id;
                Slave_Processor(M).Neighbour_Link_List(g,2)=neighbour_id;
                Slave_Processor(M).Neighbour_Link_List(g,3)=neighbour_id_other_processor;
                % 获取该目标点对应的刚度矩阵分量2x2
                k=zeros(2,2);
                if neighbour_id > nci
                    neighbour_id=neighbour_id-nci;
                    for i1=1:1:2
                        for i2=1:1:2
                            k(i1,i2)=Slave_Processor(M).Global_Stiffness_Matrix_mxn(local_id*2-2+i1,neighbour_id*2-2+i2);
                        end
                    end
                else
                    for i1=1:1:2
                        for i2=1:1:2
                            k(i1,i2)=Slave_Processor(M).Global_Stiffness_Matrix_nxn(local_id*2-2+i1,neighbour_id*2-2+i2);
                        end
                    end
                end
                % 刚度分量保存到表中
                Slave_Processor(M).Neighbour_Link_Stiffness{g}=k;
                g=g+1;
            end
        end
        
        %======================================================
        %       获取邻居的从属点，获取相连刚度 West
        %======================================================
        N=Slave_Processor(M).Neighbour_West;
        if Slave_Processor(M).Neighbour_West~=-1
            %获取目标点正西方点的直接相连点的连接关系
            for i=1:1:Slave_Processor(M).Num_Y_Node_Local
                % 获取连接点的序号
                col_local=Slave_Processor(M).Local_Node_Calculation_X_Location;
                row_local=Slave_Processor(M).Local_Node_Calculation_Y_Location+i-1;
                row_neighbour=row_local;
                col_neighbour=col_local-1;
                local_id=Slave_Processor(M).Node_Information_Matrix_Calculate(row_local,col_local);
                neighbour_id=Slave_Processor(M).Node_Information_Matrix_Calculate(row_neighbour,col_neighbour);
                neighbour_id_other_processor=Slave_Processor(N).Num_X_Node_Local*i;
                % 序号保存到表中
                Slave_Processor(M).Neighbour_Link_List(g,1)=local_id;
                Slave_Processor(M).Neighbour_Link_List(g,2)=neighbour_id;
                Slave_Processor(M).Neighbour_Link_List(g,3)=neighbour_id_other_processor;
                % 获取该目标点对应的刚度矩阵分量2x2
                k=zeros(2,2);
                if neighbour_id > nci
                    neighbour_id=neighbour_id-nci;
                    for i1=1:1:2
                        for i2=1:1:2
                            k(i1,i2)=Slave_Processor(M).Global_Stiffness_Matrix_mxn(local_id*2-2+i1,neighbour_id*2-2+i2);
                        end
                    end
                else
                    for i1=1:1:2
                        for i2=1:1:2
                            k(i1,i2)=Slave_Processor(M).Global_Stiffness_Matrix_nxn(local_id*2-2+i1,neighbour_id*2-2+i2);
                        end
                    end
                end
                % 刚度分量保存到表中
                Slave_Processor(M).Neighbour_Link_Stiffness{g}=k;
                g=g+1;
            end
            %获取目标点西南方点的直接相连点在该从处理器中的序号
            for i=1:1:Slave_Processor(M).Num_Y_Node_Local-1
                % 获取连接点的序号
                col_local=Slave_Processor(M).Local_Node_Calculation_X_Location;
                row_local=Slave_Processor(M).Local_Node_Calculation_Y_Location+i-1;
                row_neighbour=row_local+1;
                col_neighbour=col_local-1;
                local_id=Slave_Processor(M).Node_Information_Matrix_Calculate(row_local,col_local);
                neighbour_id=Slave_Processor(M).Node_Information_Matrix_Calculate(row_neighbour,col_neighbour);
                neighbour_id_other_processor=Slave_Processor(N).Num_X_Node_Local*(i+1);
                % 序号保存到表中
                Slave_Processor(M).Neighbour_Link_List(g,1)=local_id;
                Slave_Processor(M).Neighbour_Link_List(g,2)=neighbour_id;
                Slave_Processor(M).Neighbour_Link_List(g,3)=neighbour_id_other_processor;
                % 获取该目标点对应的刚度矩阵分量2x2
                k=zeros(2,2);
                if neighbour_id > nci
                    neighbour_id=neighbour_id-nci;
                    for i1=1:1:2
                        for i2=1:1:2
                            k(i1,i2)=Slave_Processor(M).Global_Stiffness_Matrix_mxn(local_id*2-2+i1,neighbour_id*2-2+i2);
                        end
                    end
                else
                    for i1=1:1:2
                        for i2=1:1:2
                            k(i1,i2)=Slave_Processor(M).Global_Stiffness_Matrix_nxn(local_id*2-2+i1,neighbour_id*2-2+i2);
                        end
                    end
                end
                % 刚度分量保存到表中
                Slave_Processor(M).Neighbour_Link_Stiffness{g}=k;
                g=g+1;
            end
        end
        %======================================================
        %       获取邻居的从属点，获取相连刚度 NorthWest 无
        %======================================================
        M=M+1;
    end
end
%% 建立各级基本位移模式
M=1;
for I=1:1:Num_Slave_Processor_Row
    for J=1:1:Num_Slave_Processor_Col
        Slave_Processor(M).Displacement_Model=cell(12,1);
        % 建立各级模态的储存空间
        for i=1:1:12
            Slave_Processor(M).Displacement_Model{i}=zeros(Slave_Processor(M).Num_Node_Cal_Inner*2,1);
%             Slave_Processor(M).Displacement_Model{i}(i)=1;
        end
        
        for i=1:1:Slave_Processor(M).Num_Node_Cal_Inner
            % 对初始化第一个模态
            n=2*i-1;
            Slave_Processor(M).Displacement_Model{1}(n)=1;
            % 初始化第二个模态
            n=2*i;
            Slave_Processor(M).Displacement_Model{2}(n)=1;
            % 初始化第三个模态
            n=2*i-1;
            %Slave_Processor(M).Displacement_Model{3}(n)=Slave_Processor(M).Node_Coordinates_List_Calculate(i,1)-Slave_Processor(M).Center_X_Coordinate;
            Slave_Processor(M).Displacement_Model{3}(n)=Slave_Processor(M).Node_Coordinates_List_Calculate(i,1);
            % 初始化第四个模态
            n=2*i-1;
            %Slave_Processor(M).Displacement_Model{4}(n)=Slave_Processor(M).Node_Coordinates_List_Calculate(i,2)-Slave_Processor(M).Center_Y_Coordinate;
            Slave_Processor(M).Displacement_Model{4}(n)=Slave_Processor(M).Node_Coordinates_List_Calculate(i,2);
            % 初始化第五个模态
            n=2*i;
            %Slave_Processor(M).Displacement_Model{5}(n)=Slave_Processor(M).Node_Coordinates_List_Calculate(i,1)-Slave_Processor(M).Center_X_Coordinate;
            Slave_Processor(M).Displacement_Model{5}(n)=Slave_Processor(M).Node_Coordinates_List_Calculate(i,1);
            % 初始化第六个模态
            n=2*i;
            %Slave_Processor(M).Displacement_Model{6}(n)=Slave_Processor(M).Node_Coordinates_List_Calculate(i,2)-Slave_Processor(M).Center_Y_Coordinate;
            Slave_Processor(M).Displacement_Model{6}(n)=Slave_Processor(M).Node_Coordinates_List_Calculate(i,2);
        end
        M=M+1;
    end
end
%% 求解迭代初始化
M=1;
for I=1:1:Num_Slave_Processor_Row
    for J=1:1:Num_Slave_Processor_Col
        n=Slave_Processor(M).Num_Node_Cal_Inner;
        m=Slave_Processor(M).Num_Node_Cal_Outer;
        Slave_Processor(M).P_RD=zeros(2*n,1);
        Slave_Processor(M).U_n_RD=zeros(2*n,1);
        Slave_Processor(M).U_n_old=zeros(2*n,1);
        Slave_Processor(M).U_m_old=zeros(2*m,1);
%         load(['d_' num2str(M) '.mat']);
%         for i=1:1:Slave_Processor(M).Num_Node_Local*2
%             Slave_Processor(M).U_n_old(i)=di(i);
%         end
        M=M+1;
    end
end
M=1;
for I=1:1:Num_Slave_Processor_Row
        for J=1:1:Num_Slave_Processor_Col
            % 更新邻居信息 North
            if Slave_Processor(M).Neighbour_North~=-1
                N=Slave_Processor(M).Neighbour_North;
                g=1;
                for j=1:1:Num_Node_Extend_Y
                    for i=1:1:Slave_Processor(M).Num_X_Node_Local %遍历邻居消息传递的每一个点
                        neigh_id_in_local=Slave_Processor(M).Neighbour_North_Index_Calculation_List(g,1);
                        neigh_id_in_neigh=Slave_Processor(M).Neighbour_North_Index_Calculation_List(g,2);
                        if neigh_id_in_local>Slave_Processor(M).Num_Node_Cal_Inner
                            %若属于边界节点，则将信息更新到U_m_old中,自由度有两个,记得保存两个自由度
                            neigh_id_in_local=neigh_id_in_local-Slave_Processor(M).Num_Node_Cal_Inner;
                            for i1=1:2
                                Slave_Processor(M).U_m_old((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).U_n_old((neigh_id_in_neigh-1)*2+i1);
                            end
                        else
                            %若属于内部节点，则将信息更新到U_n_old中
                            for i1=1:2
                                Slave_Processor(M).U_n_old((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).U_n_old((neigh_id_in_neigh-1)*2+i1);
                            end
                        end
                        g=g+1;
                    end
                end
            end
            % 更新邻居信息 NorthEast
            if Slave_Processor(M).Neighbour_NorthEast~=-1
                N=Slave_Processor(M).Neighbour_NorthEast;
                g=1;
                for j=1:1:Num_Node_Extend_Y
                    for i=1:1:Num_Node_Extend_X %遍历邻居消息传递的每一个点
                        neigh_id_in_local=Slave_Processor(M).Neighbour_NorthEast_Index_Calculation_List(g,1);
                        neigh_id_in_neigh=Slave_Processor(M).Neighbour_NorthEast_Index_Calculation_List(g,2);
                        if neigh_id_in_local>Slave_Processor(M).Num_Node_Cal_Inner
                            %若属于边界节点，则将信息更新到U_m_old中,自由度有两个,记得保存两个自由度
                            neigh_id_in_local=neigh_id_in_local-Slave_Processor(M).Num_Node_Cal_Inner;
                            for i1=1:2
                                Slave_Processor(M).U_m_old((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).U_n_old((neigh_id_in_neigh-1)*2+i1);
                            end
                        else
                            %若属于内部节点，则将信息更新到U_n_old中
                            for i1=1:2
                                Slave_Processor(M).U_n_old((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).U_n_old((neigh_id_in_neigh-1)*2+i1);
                            end
                        end
                        g=g+1;
                    end
                end
            end
            % 更新邻居信息 East
            if Slave_Processor(M).Neighbour_East~=-1
                N=Slave_Processor(M).Neighbour_East;
                g=1;
                for j=1:1:Slave_Processor(M).Num_Y_Node_Local
                    for i=1:1:Num_Node_Extend_X %遍历邻居消息传递的每一个点
                        neigh_id_in_local=Slave_Processor(M).Neighbour_East_Index_Calculation_List(g,1);
                        neigh_id_in_neigh=Slave_Processor(M).Neighbour_East_Index_Calculation_List(g,2);
                        if neigh_id_in_local>Slave_Processor(M).Num_Node_Cal_Inner
                            %若属于边界节点，则将信息更新到U_m_old中,自由度有两个,记得保存两个自由度
                            neigh_id_in_local=neigh_id_in_local-Slave_Processor(M).Num_Node_Cal_Inner;
                            for i1=1:2
                                Slave_Processor(M).U_m_old((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).U_n_old((neigh_id_in_neigh-1)*2+i1);
                            end
                        else
                            %若属于内部节点，则将信息更新到U_n_old中
                            for i1=1:2
                                Slave_Processor(M).U_n_old((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).U_n_old((neigh_id_in_neigh-1)*2+i1);
                            end
                        end
                        g=g+1;
                    end
                end
            end
            % 更新邻居信息 SouthEast
            if Slave_Processor(M).Neighbour_SouthEast~=-1
                N=Slave_Processor(M).Neighbour_SouthEast;
                g=1;
                for j=1:1:Num_Node_Extend_Y
                    for i=1:1:Num_Node_Extend_X %遍历邻居消息传递的每一个点
                        neigh_id_in_local=Slave_Processor(M).Neighbour_SouthEast_Index_Calculation_List(g,1);
                        neigh_id_in_neigh=Slave_Processor(M).Neighbour_SouthEast_Index_Calculation_List(g,2);
                        if neigh_id_in_local>Slave_Processor(M).Num_Node_Cal_Inner
                            %若属于边界节点，则将信息更新到U_m_old中,自由度有两个,记得保存两个自由度
                            neigh_id_in_local=neigh_id_in_local-Slave_Processor(M).Num_Node_Cal_Inner;
                            for i1=1:2
                                Slave_Processor(M).U_m_old((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).U_n_old((neigh_id_in_neigh-1)*2+i1);
                            end
                        else
                            %若属于内部节点，则将信息更新到U_n_old中
                            for i1=1:2
                                Slave_Processor(M).U_n_old((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).U_n_old((neigh_id_in_neigh-1)*2+i1);
                            end
                        end
                        g=g+1;
                    end
                end
            end
            % 更新邻居信息 South
            if Slave_Processor(M).Neighbour_South~=-1
                N=Slave_Processor(M).Neighbour_South;
                g=1;
                for j=1:1:Num_Node_Extend_Y
                    for i=1:1:Slave_Processor(M).Num_X_Node_Local %遍历邻居消息传递的每一个点
                        neigh_id_in_local=Slave_Processor(M).Neighbour_South_Index_Calculation_List(g,1);
                        neigh_id_in_neigh=Slave_Processor(M).Neighbour_South_Index_Calculation_List(g,2);
                        if neigh_id_in_local>Slave_Processor(M).Num_Node_Cal_Inner
                            %若属于边界节点，则将信息更新到U_m_old中,自由度有两个,记得保存两个自由度
                            neigh_id_in_local=neigh_id_in_local-Slave_Processor(M).Num_Node_Cal_Inner;
                            for i1=1:2
                                Slave_Processor(M).U_m_old((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).U_n_old((neigh_id_in_neigh-1)*2+i1);
                            end
                        else
                            %若属于内部节点，则将信息更新到U_n_old中
                            for i1=1:2
                                Slave_Processor(M).U_n_old((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).U_n_old((neigh_id_in_neigh-1)*2+i1);
                            end
                        end
                        g=g+1;
                    end
                end
            end
            % 更新邻居信息 SouthWest
            if Slave_Processor(M).Neighbour_SouthWest~=-1
                N=Slave_Processor(M).Neighbour_SouthWest;
                g=1;
                for j=1:1:Num_Node_Extend_Y
                    for i=1:1:Num_Node_Extend_X %遍历邻居消息传递的每一个点
                        neigh_id_in_local=Slave_Processor(M).Neighbour_SouthWest_Index_Calculation_List(g,1);
                        neigh_id_in_neigh=Slave_Processor(M).Neighbour_SouthWest_Index_Calculation_List(g,2);
                        if neigh_id_in_local>Slave_Processor(M).Num_Node_Cal_Inner
                            %若属于边界节点，则将信息更新到U_m_old中,自由度有两个,记得保存两个自由度
                            neigh_id_in_local=neigh_id_in_local-Slave_Processor(M).Num_Node_Cal_Inner;
                            for i1=1:2
                                Slave_Processor(M).U_m_old((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).U_n_old((neigh_id_in_neigh-1)*2+i1);
                            end
                        else
                            %若属于内部节点，则将信息更新到U_n_old中
                            for i1=1:2
                                Slave_Processor(M).U_n_old((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).U_n_old((neigh_id_in_neigh-1)*2+i1);
                            end
                        end
                        g=g+1;
                    end
                end
            end
            % 更新邻居信息 West
            if Slave_Processor(M).Neighbour_West~=-1
                N=Slave_Processor(M).Neighbour_West;
                g=1;
                for j=1:1:Slave_Processor(M).Num_Y_Node_Local
                    for i=1:1:Num_Node_Extend_X %遍历邻居消息传递的每一个点
                        neigh_id_in_local=Slave_Processor(M).Neighbour_West_Index_Calculation_List(g,1);
                        neigh_id_in_neigh=Slave_Processor(M).Neighbour_West_Index_Calculation_List(g,2);
                        if neigh_id_in_local>Slave_Processor(M).Num_Node_Cal_Inner
                            %若属于边界节点，则将信息更新到U_m_old中,自由度有两个,记得保存两个自由度
                            neigh_id_in_local=neigh_id_in_local-Slave_Processor(M).Num_Node_Cal_Inner;
                            for i1=1:2
                                Slave_Processor(M).U_m_old((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).U_n_old((neigh_id_in_neigh-1)*2+i1);
                            end
                        else
                            %若属于内部节点，则将信息更新到U_n_old中
                            for i1=1:2
                                Slave_Processor(M).U_n_old((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).U_n_old((neigh_id_in_neigh-1)*2+i1);
                            end
                        end
                        g=g+1;
                    end
                end
            end
            % 更新邻居信息 NorthWest
            if Slave_Processor(M).Neighbour_NorthWest~=-1
                N=Slave_Processor(M).Neighbour_NorthWest;
                g=1;
                for j=1:1:Num_Node_Extend_Y
                    for i=1:1:Num_Node_Extend_X %遍历邻居消息传递的每一个点
                        neigh_id_in_local=Slave_Processor(M).Neighbour_NorthWest_Index_Calculation_List(g,1);
                        neigh_id_in_neigh=Slave_Processor(M).Neighbour_NorthWest_Index_Calculation_List(g,2);
                        if neigh_id_in_local>Slave_Processor(M).Num_Node_Cal_Inner
                            %若属于边界节点，则将信息更新到U_m_old中,自由度有两个,记得保存两个自由度
                            neigh_id_in_local=neigh_id_in_local-Slave_Processor(M).Num_Node_Cal_Inner;
                            for i1=1:2
                                Slave_Processor(M).U_m_old((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).U_n_old((neigh_id_in_neigh-1)*2+i1);
                            end
                        else
                            %若属于内部节点，则将信息更新到U_n_old中
                            for i1=1:2
                                Slave_Processor(M).U_n_old((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).U_n_old((neigh_id_in_neigh-1)*2+i1);
                            end
                        end
                        g=g+1;
                    end
                end
            end
            M=M+1;
        end
    end
%% 核心求解主循环
figure
Master_Processor.Res=zeros(Max_Iter,1);
for Iter=1:1:Max_Iter
    %=========================================
    %   从处理器得到计算误差
    %=========================================
    M=1;
    for I=1:1:Num_Slave_Processor_Row
        for J=1:1:Num_Slave_Processor_Col   %对每一个从处理器进行遍历
            % 从处理器下为凝聚自由度的载荷向量分配内存空间
            Slave_Processor(M).Res=zeros(Slave_Processor(M).Num_Node_Local*2,1);
            for i=1:1:Slave_Processor(M).Num_Node_Local % 对每一个下属的节点的每一个自由度进行累加
                for i1=1:2
                    dof_id=(i-1)*2+i1;
                    p0=0;
                    % 对所有的相连处理器进行遍历
                    % 当目标处理器是本地处理器时
                    for j=1:1:Slave_Processor(M).Num_Node_Local*2
                        p0=p0-Slave_Processor(M).Global_Stiffness_Matrix_nxn(dof_id,j)*Slave_Processor(M).U_n_old(j);
                    end
                    g=1;
                    % 遍历邻居处理器North
                    if Slave_Processor(M).Neighbour_North~=-1
                        N=Slave_Processor(M).Neighbour_North;
                        for q=1:1:2*Slave_Processor(M).Num_X_Node_Local-1
                            local_id=Slave_Processor(M).Neighbour_Link_List(g,1);
                            
                            if local_id ~= i
                                g=g+1;
                                continue;
                            end
                            neigh_id=Slave_Processor(M).Neighbour_Link_List(g,3);
                            
                            K0=Slave_Processor(M).Neighbour_Link_Stiffness{g};
                            for i2=1:1:2
                                p0=p0-K0(i1,i2)*Slave_Processor(N).U_n_old(2*neigh_id-2+i2);
                                
                            end
                            g=g+1;
                        end
                    end
                    
                    % 遍历邻居处理器NorthEast
                    if Slave_Processor(M).Neighbour_NorthEast~=-1
                        N=Slave_Processor(M).Neighbour_NorthEast;
                        for q=1
                            local_id=Slave_Processor(M).Neighbour_Link_List(g,1);
                            if local_id ~= i
                                g=g+1;
                                continue;
                            end
                            neigh_id=Slave_Processor(M).Neighbour_Link_List(g,3);
                            K0=Slave_Processor(M).Neighbour_Link_Stiffness{g};
                            for i2=1:1:2
                                p0=p0-K0(i1,i2)*Slave_Processor(N).U_n_old(2*neigh_id-2+i2);
                            end
                            g=g+1;
                        end
                    end
                    % 遍历邻居处理器East
                    if Slave_Processor(M).Neighbour_East~=-1
                        N=Slave_Processor(M).Neighbour_East;
                        for q=1:1:2*Slave_Processor(M).Num_Y_Node_Local-1
                            local_id=Slave_Processor(M).Neighbour_Link_List(g,1);
                            if local_id ~= i
                                g=g+1;
                                continue;
                            end
                            neigh_id=Slave_Processor(M).Neighbour_Link_List(g,3);
                            K0=Slave_Processor(M).Neighbour_Link_Stiffness{g};
                            for i2=1:1:2
                                p0=p0-K0(i1,i2)*Slave_Processor(N).U_n_old(2*neigh_id-2+i2);
                            end
                            g=g+1;
                        end
                    end
                     
                    % 遍历邻居处理器SouthEast
                    % 无
                    % 遍历令狐处理器South
                    if Slave_Processor(M).Neighbour_South~=-1
                        N=Slave_Processor(M).Neighbour_South;
                        for q=1:1:2*Slave_Processor(M).Num_X_Node_Local-1
                            local_id=Slave_Processor(M).Neighbour_Link_List(g,1);
                            if local_id ~= i
                                g=g+1;
                                continue;
                            end
                            neigh_id=Slave_Processor(M).Neighbour_Link_List(g,3);
                            K0=Slave_Processor(M).Neighbour_Link_Stiffness{g};
                            for i2=1:1:2
                                p0=p0-K0(i1,i2)*Slave_Processor(N).U_n_old(2*neigh_id-2+i2);
                            end
                            g=g+1;
                        end
                    end
                    % 遍历邻居处理器SouthWest
                    if Slave_Processor(M).Neighbour_SouthWest~=-1
                        N=Slave_Processor(M).Neighbour_SouthWest;
                        for q=1
                            local_id=Slave_Processor(M).Neighbour_Link_List(g,1);
                            if local_id ~= i
                                g=g+1;
                                continue;
                            end
                            neigh_id=Slave_Processor(M).Neighbour_Link_List(g,3);
                            K0=Slave_Processor(M).Neighbour_Link_Stiffness{g};
                            for i2=1:1:2
                                p0=p0-K0(i1,i2)*Slave_Processor(N).U_n_old(2*neigh_id-2+i2);
                            end
                            g=g+1;
                        end
                    end
                    % 遍历邻居处理器West
                    if Slave_Processor(M).Neighbour_West~=-1
                        N=Slave_Processor(M).Neighbour_West;
                        for q=1:1:2*Slave_Processor(M).Num_Y_Node_Local-1
                            local_id=Slave_Processor(M).Neighbour_Link_List(g,1);
                            if local_id ~= i
                                g=g+1;
                                continue;
                            end
                            neigh_id=Slave_Processor(M).Neighbour_Link_List(g,3);
                            K0=Slave_Processor(M).Neighbour_Link_Stiffness{g};
                            for i2=1:1:2
                                p0=p0-K0(i1,i2)*Slave_Processor(N).U_n_old(2*neigh_id-2+i2);
                            end
                            g=g+1;
                        end
                    end
                   
                    % 遍历邻居处理器NoethWest
                    % 无
                    p0=p0+Slave_Processor(M).P(dof_id);
                    Slave_Processor(M).Res(dof_id)=p0;
                end
            end
            M=M+1;
        end
    end
    
    %=======================================
    %从处理器计算能量
    %=======================================
    Energy=0;
    M=1;
    for I=1:1:Num_Slave_Processor_Row
        for J=1:1:Num_Slave_Processor_Col   %对每一个从处理器进行遍历
            % 从处理器下为凝聚自由度的载荷向量分配内存空间
            
            for i=1:1:Slave_Processor(M).Num_Node_Local % 对每一个下属的节点的每一个自由度进行累加
                for i1=1:2
                    dof_id=(i-1)*2+i1;
                    Energy=Energy-Slave_Processor(M).U_n_old(dof_id)*Slave_Processor(M).P(dof_id);
                    % 对所有的相连处理器进行遍历
                    % 当目标处理器是本地处理器时
                    for j=1:1:Slave_Processor(M).Num_Node_Local*2
                        Energy=Energy+0.5*Slave_Processor(M).U_n_old(dof_id)*Slave_Processor(M).Global_Stiffness_Matrix_nxn(dof_id,j)*Slave_Processor(M).U_n_old(j);
                        
                    end
                     g=1;
                    % 遍历邻居处理器North
                    if Slave_Processor(M).Neighbour_North~=-1
                        N=Slave_Processor(M).Neighbour_North;
                        for q=1:1:2*Slave_Processor(M).Num_X_Node_Local-1
                            local_id=Slave_Processor(M).Neighbour_Link_List(g,1);
                            
                            if local_id ~= i
                                g=g+1;
                                continue;
                            end
                            neigh_id=Slave_Processor(M).Neighbour_Link_List(g,3);
                            K0=Slave_Processor(M).Neighbour_Link_Stiffness{g};
                            for i2=1:1:2
                                Energy=Energy+0.5*Slave_Processor(M).U_n_old(dof_id)*K0(i1,i2)*Slave_Processor(N).U_n_old(2*neigh_id-2+i2);
                            end
                            g=g+1;
                        end
                    end
                    
                    % 遍历邻居处理器NorthEast
                    if Slave_Processor(M).Neighbour_NorthEast~=-1
                        N=Slave_Processor(M).Neighbour_NorthEast;
                        for q=1
                            local_id=Slave_Processor(M).Neighbour_Link_List(g,1);
                            if local_id ~= i
                                g=g+1;
                                continue;
                            end
                            neigh_id=Slave_Processor(M).Neighbour_Link_List(g,3);
                            K0=Slave_Processor(M).Neighbour_Link_Stiffness{g};
                            for i2=1:1:2
                                Energy=Energy+0.5*Slave_Processor(M).U_n_old(dof_id)*K0(i1,i2)*Slave_Processor(N).U_n_old(2*neigh_id-2+i2);
                            end
                            g=g+1;
                        end
                    end
                    % 遍历邻居处理器East
                    if Slave_Processor(M).Neighbour_East~=-1
                        N=Slave_Processor(M).Neighbour_East;
                        for q=1:1:2*Slave_Processor(M).Num_Y_Node_Local-1
                            local_id=Slave_Processor(M).Neighbour_Link_List(g,1);
                            if local_id ~= i
                                g=g+1;
                                continue;
                            end
                            neigh_id=Slave_Processor(M).Neighbour_Link_List(g,3);
                            K0=Slave_Processor(M).Neighbour_Link_Stiffness{g};
                            for i2=1:1:2
                                Energy=Energy+0.5*Slave_Processor(M).U_n_old(dof_id)*K0(i1,i2)*Slave_Processor(N).U_n_old(2*neigh_id-2+i2);
                            end
                            g=g+1;
                        end
                    end
                     
                    % 遍历邻居处理器SouthEast
                    % 无
                    % 遍历令狐处理器South
                    if Slave_Processor(M).Neighbour_South~=-1
                        N=Slave_Processor(M).Neighbour_South;
                        for q=1:1:2*Slave_Processor(M).Num_X_Node_Local-1
                            local_id=Slave_Processor(M).Neighbour_Link_List(g,1);
                            if local_id ~= i
                                g=g+1;
                                continue;
                            end
                            neigh_id=Slave_Processor(M).Neighbour_Link_List(g,3);
                            K0=Slave_Processor(M).Neighbour_Link_Stiffness{g};
                            for i2=1:1:2
                                Energy=Energy+0.5*Slave_Processor(M).U_n_old(dof_id)*K0(i1,i2)*Slave_Processor(N).U_n_old(2*neigh_id-2+i2);
                            end
                            g=g+1;
                        end
                    end
                    % 遍历邻居处理器SouthWest
                    if Slave_Processor(M).Neighbour_SouthWest~=-1
                        N=Slave_Processor(M).Neighbour_SouthWest;
                        for q=1
                            local_id=Slave_Processor(M).Neighbour_Link_List(g,1);
                            if local_id ~= i
                                g=g+1;
                                continue;
                            end
                            neigh_id=Slave_Processor(M).Neighbour_Link_List(g,3);
                            K0=Slave_Processor(M).Neighbour_Link_Stiffness{g};
                            for i2=1:1:2
                                Energy=Energy+0.5*Slave_Processor(M).U_n_old(dof_id)*K0(i1,i2)*Slave_Processor(N).U_n_old(2*neigh_id-2+i2);
                            end
                            g=g+1;
                        end
                    end
                    % 遍历邻居处理器West
                    if Slave_Processor(M).Neighbour_West~=-1
                        N=Slave_Processor(M).Neighbour_West;
                        for q=1:1:2*Slave_Processor(M).Num_Y_Node_Local-1
                            local_id=Slave_Processor(M).Neighbour_Link_List(g,1);
                            if local_id ~= i
                                g=g+1;
                                continue;
                            end
                            neigh_id=Slave_Processor(M).Neighbour_Link_List(g,3);
                            K0=Slave_Processor(M).Neighbour_Link_Stiffness{g};
                            for i2=1:1:2
                                Energy=Energy+0.5*Slave_Processor(M).U_n_old(dof_id)*K0(i1,i2)*Slave_Processor(N).U_n_old(2*neigh_id-2+i2);
                            end
                            g=g+1;
                        end
                    end
                   
                    % 遍历邻居处理器NoethWest
                end
            end
            M=M+1;
        end
    end
    
    %=======================================
    %   从处理器计算各自残差，进行收敛性判断
    %=======================================
    M=1;
    for I=1:1:Num_Slave_Processor_Row
        for J=1:1:Num_Slave_Processor_Col
            for i=1:1:Slave_Processor(M).Num_Node_Local*2
                %                 Slave_Processor(M).Res(i)=Slave_Processor(M).Res(i)/Slave_Processor(M).P(i);
            end
            Slave_Processor(M).Sum_Res=sum(abs(Slave_Processor(M).Res));
            M=M+1;
        end
    end
    
    %=====================================
    % 主处理器计算残差
    %=====================================
    M=1;
    
    for I=1:1:Num_Slave_Processor_Row
        for J=1:1:Num_Slave_Processor_Col
            Master_Processor.Res(Iter)=Master_Processor.Res(Iter)+Slave_Processor(M).Sum_Res;
            M=M+1;
        end
    end
    disp(['第 ' num2str(Iter) '次计算： 能量' num2str(Energy) '误差' num2str(Master_Processor.Res(Iter)) ]);
    
    
    %===================================
    %   从处理器计算得到各阶位移模式
    %===================================
    M=1;
    for I=1:1:Num_Slave_Processor_Row
        for J=1:1:Num_Slave_Processor_Col
            % -----------------计算位移松弛模态----------------------------
            Slave_Processor(M).P_RD=Slave_Processor(M).P-Slave_Processor(M).Global_Stiffness_Matrix_mxn*Slave_Processor(M).U_m_old;
            Slave_Processor(M).U_RD=Linear_Equations_Pivot_Gauss_Solver(Slave_Processor(M).Global_Stiffness_Matrix_nxn,Slave_Processor(M).P_RD);
            % 计算得到第七个位移松弛模式
            Slave_Processor(M).Displacement_Model{7}=Slave_Processor(M).U_RD-Slave_Processor(M).U_n_old;
            % 计算得到第八个位移松弛模式
            for i=1:1:Slave_Processor(M).Num_Node_Cal_Inner
                %Slave_Processor(M).Displacement_Model{8}(2*i-1)=Slave_Processor(M).Displacement_Model{7}(2*i-1).*(Slave_Processor(M).Node_Coordinates_List_Calculate(i,1)-Slave_Processor(M).Center_X_Coordinate);
                %Slave_Processor(M).Displacement_Model{8}(2*i)=Slave_Processor(M).Displacement_Model{7}(2*i).*(Slave_Processor(M).Node_Coordinates_List_Calculate(i,1)-Slave_Processor(M).Center_X_Coordinate);
                Slave_Processor(M).Displacement_Model{8}(2*i-1)=Slave_Processor(M).Displacement_Model{7}(2*i-1).*Slave_Processor(M).Node_Coordinates_List_Calculate(i,1);
                Slave_Processor(M).Displacement_Model{8}(2*i)=Slave_Processor(M).Displacement_Model{7}(2*i).*Slave_Processor(M).Node_Coordinates_List_Calculate(i,1);
                
            end
            % 计算得到第九个位移松弛模式
            for i=1:1:Slave_Processor(M).Num_Node_Cal_Inner
                %                 Slave_Processor(M).Displacement_Model{9}(2*i-1)=Slave_Processor(M).Displacement_Model{7}(2*i-1).*(Slave_Processor(M).Node_Coordinates_List_Calculate(i,2)-Slave_Processor(M).Center_Y_Coordinate);
                %                 Slave_Processor(M).Displacement_Model{9}(2*i)=Slave_Processor(M).Displacement_Model{7}(2*i).*(Slave_Processor(M).Node_Coordinates_List_Calculate(i,2)-Slave_Processor(M).Center_Y_Coordinate);
                Slave_Processor(M).Displacement_Model{9}(2*i-1)=Slave_Processor(M).Displacement_Model{7}(2*i-1).*Slave_Processor(M).Node_Coordinates_List_Calculate(i,2);
                Slave_Processor(M).Displacement_Model{9}(2*i)=Slave_Processor(M).Displacement_Model{7}(2*i).*Slave_Processor(M).Node_Coordinates_List_Calculate(i,2);
                
            end
            % -----------------计算最速下降松弛模态----------------------------
            % 最速下降松弛模态：
            % 计算得到第十个位移松弛模式
            
            for i=1:1:Slave_Processor(M).Num_Node_Local*2
                Slave_Processor(M).Displacement_Model{10}(i)=Slave_Processor(M).Res(i);
            end
            % 计算得到第十一个位移松弛模式
            for i=1:1:Slave_Processor(M).Num_Node_Cal_Inner
                %                 Slave_Processor(M).Displacement_Model{11}(2*i-1)=Slave_Processor(M).Displacement_Model{10}(2*i-1).*(Slave_Processor(M).Node_Coordinates_List_Calculate(i,1)-Slave_Processor(M).Center_X_Coordinate);
                %                 Slave_Processor(M).Displacement_Model{11}(2*i)=Slave_Processor(M).Displacement_Model{10}(2*i).*(Slave_Processor(M).Node_Coordinates_List_Calculate(i,1)-Slave_Processor(M).Center_X_Coordinate);
                Slave_Processor(M).Displacement_Model{11}(2*i-1)=Slave_Processor(M).Displacement_Model{10}(2*i-1).*Slave_Processor(M).Node_Coordinates_List_Calculate(i,1);
                Slave_Processor(M).Displacement_Model{11}(2*i)=Slave_Processor(M).Displacement_Model{10}(2*i).*Slave_Processor(M).Node_Coordinates_List_Calculate(i,1);
                
            end
            % 计算得到第十二个位移松弛模式
            for i=1:1:Slave_Processor(M).Num_Node_Cal_Inner
                %                 Slave_Processor(M).Displacement_Model{12}(2*i-1)=Slave_Processor(M).Displacement_Model{10}(2*i-1).*(Slave_Processor(M).Node_Coordinates_List_Calculate(i,2)-Slave_Processor(M).Center_Y_Coordinate);
                %                 Slave_Processor(M).Displacement_Model{12}(2*i)=Slave_Processor(M).Displacement_Model{10}(2*i).*(Slave_Processor(M).Node_Coordinates_List_Calculate(i,2)-Slave_Processor(M).Center_Y_Coordinate);
                Slave_Processor(M).Displacement_Model{12}(2*i-1)=Slave_Processor(M).Displacement_Model{10}(2*i-1).*Slave_Processor(M).Node_Coordinates_List_Calculate(i,2);
                Slave_Processor(M).Displacement_Model{12}(2*i)=Slave_Processor(M).Displacement_Model{10}(2*i).*Slave_Processor(M).Node_Coordinates_List_Calculate(i,2);
                
            end
            
            % -------------------------随机松弛模态-------------------------------
            %             Slave_Processor(M).Displacement_Model{7}=rand(Slave_Processor(M).Num_Node_Cal_Inner*2,1);
%                          for i=10:1:11
%                              Slave_Processor(M).Displacement_Model{i}=rand(Slave_Processor(M).Num_Node_Cal_Inner*2,1);
%                          end
            M=M+1;
        end
    end    
    %=========================================
    %   从处理器计算得到各个凝聚方程的系数矩阵  校核无误
    %=========================================
    M=1;
    N=1;
    for I=1:1:Num_Slave_Processor_Row
        for J=1:1:Num_Slave_Processor_Col   %这两重循环是对每一个处理器进行操作
            % 在每一个处理器内部，要对其余的处理器进行遍历
            % 但如果没有直接相连的节点，则刚度矩阵分量为0，所以只需要遍历直接相连的处理器和自身
            % 先对所有处理器进行赋初值
            Slave_Processor(M).K_amI_alJ=zeros(Num_Displacement_Model,Num_Displacement_Model,Num_Slave_Processor);
            for m=1:1:Num_Displacement_Model
                
                for l=1:1:Num_Displacement_Model
                    % 遍历自身
                    for i=1:1:Slave_Processor(M).Num_Node_Local*2
                        for j=1:1:Slave_Processor(M).Num_Node_Local*2
                            Slave_Processor(M).K_amI_alJ(m,l,M)=Slave_Processor(M).K_amI_alJ(m,l,M)...
                                +Slave_Processor(M).Displacement_Model{m}(i)*Slave_Processor(M).Global_Stiffness_Matrix_nxn(i,j)*Slave_Processor(M).Displacement_Model{l}(j);
                        end
                    end
                    g=1;
                    % 遍历邻居节点North
                    if Slave_Processor(M).Neighbour_North~=-1
                        N=Slave_Processor(M).Neighbour_North;
                        for q=1:1:2*Slave_Processor(M).Num_X_Node_Local-1
                            local_id=Slave_Processor(M).Neighbour_Link_List(g,1);
                            neigh_id=Slave_Processor(M).Neighbour_Link_List(g,3);
                            %                             if(M==5)
                            %                                         neigh_id
                            %                             end
                            
                            K0=Slave_Processor(M).Neighbour_Link_Stiffness{g};
                            for i1=1:1:2
                                for i2=1:1:2
                                    Slave_Processor(M).K_amI_alJ(m,l,N)=Slave_Processor(M).K_amI_alJ(m,l,N)...
                                        +Slave_Processor(M).Displacement_Model{m}(2*local_id-2+i1)*K0(i1,i2)*Slave_Processor(N).Displacement_Model{l}(2*neigh_id-2+i2);
                                    %                                     if(M==5)
                                    %                                         disp([num2str(Slave_Processor(M).Displacement_Model{m}(2*local_id-2+i1)) '----------' num2str(K0(i1,i2)) '----------' num2str(Slave_Processor(N).Displacement_Model{l}(2*neigh_id-2+i2))])
                                    %                                     end
                                end
                            end
                            g=g+1;
                        end
                    end
                    % 遍历邻居节点NorthEast
                    if Slave_Processor(M).Neighbour_NorthEast~=-1
                        N=Slave_Processor(M).Neighbour_NorthEast;
                        for q=1
                            local_id=Slave_Processor(M).Neighbour_Link_List(g,1);
                            neigh_id=Slave_Processor(M).Neighbour_Link_List(g,3);
                            K0=Slave_Processor(M).Neighbour_Link_Stiffness{g};
                            for i1=1:1:2
                                for i2=1:1:2
                                    Slave_Processor(M).K_amI_alJ(m,l,N)=Slave_Processor(M).K_amI_alJ(m,l,N)...
                                        +Slave_Processor(M).Displacement_Model{m}(2*local_id-2+i1)*K0(i1,i2)*Slave_Processor(N).Displacement_Model{l}(2*neigh_id-2+i2);
                                end
                            end
                            g=g+1;
                        end
                    end
                    %                     % 遍历邻居节点East
                    if Slave_Processor(M).Neighbour_East~=-1
                        N=Slave_Processor(M).Neighbour_East;
                        for q=1:1:2*Slave_Processor(M).Num_Y_Node_Local-1
                            local_id=Slave_Processor(M).Neighbour_Link_List(g,1);
                            neigh_id=Slave_Processor(M).Neighbour_Link_List(g,3);
                            K0=Slave_Processor(M).Neighbour_Link_Stiffness{g};
                            for i1=1:1:2
                                for i2=1:1:2
                                    Slave_Processor(M).K_amI_alJ(m,l,N)=Slave_Processor(M).K_amI_alJ(m,l,N)...
                                        +Slave_Processor(M).Displacement_Model{m}(2*local_id-2+i1)*K0(i1,i2)*Slave_Processor(N).Displacement_Model{l}(2*neigh_id-2+i2);
                                end
                            end
                            g=g+1;
                        end
                    end
                    % 遍历邻居节点SouthEast
                    
                    % 遍历邻居节点South
                    if Slave_Processor(M).Neighbour_South~=-1
                        N=Slave_Processor(M).Neighbour_South;
                        for q=1:1:2*Slave_Processor(M).Num_X_Node_Local-1
                            local_id=Slave_Processor(M).Neighbour_Link_List(g,1);
                            neigh_id=Slave_Processor(M).Neighbour_Link_List(g,3);
                            K0=Slave_Processor(M).Neighbour_Link_Stiffness{g};
                            for i1=1:1:2
                                for i2=1:1:2
                                    Slave_Processor(M).K_amI_alJ(m,l,N)=Slave_Processor(M).K_amI_alJ(m,l,N)...
                                        +Slave_Processor(M).Displacement_Model{m}(2*local_id-2+i1)*K0(i1,i2)*Slave_Processor(N).Displacement_Model{l}(2*neigh_id-2+i2);
                                end
                            end
                            g=g+1;
                        end
                    end
                    % 遍历邻居节点SouthWest
                    if Slave_Processor(M).Neighbour_SouthWest~=-1
                        N=Slave_Processor(M).Neighbour_SouthWest;
                        for q=1
                            local_id=Slave_Processor(M).Neighbour_Link_List(g,1);
                            neigh_id=Slave_Processor(M).Neighbour_Link_List(g,3);
                            K0=Slave_Processor(M).Neighbour_Link_Stiffness{g};
                            for i1=1:1:2
                                for i2=1:1:2
                                    Slave_Processor(M).K_amI_alJ(m,l,N)=Slave_Processor(M).K_amI_alJ(m,l,N)...
                                        +Slave_Processor(M).Displacement_Model{m}(2*local_id-2+i1)*K0(i1,i2)*Slave_Processor(N).Displacement_Model{l}(2*neigh_id-2+i2);
                                end
                            end
                            g=g+1;
                        end
                    end
                    % 遍历邻居节点West
                    if Slave_Processor(M).Neighbour_West~=-1
                        N=Slave_Processor(M).Neighbour_West;
                        for q=1:1:2*Slave_Processor(M).Num_Y_Node_Local-1
                            local_id=Slave_Processor(M).Neighbour_Link_List(g,1);
                            neigh_id=Slave_Processor(M).Neighbour_Link_List(g,3);
                            K0=Slave_Processor(M).Neighbour_Link_Stiffness{g};
                            for i1=1:1:2
                                for i2=1:1:2
                                    Slave_Processor(M).K_amI_alJ(m,l,N)=Slave_Processor(M).K_amI_alJ(m,l,N)...
                                        +Slave_Processor(M).Displacement_Model{m}(2*local_id-2+i1)*K0(i1,i2)*Slave_Processor(N).Displacement_Model{l}(2*neigh_id-2+i2);
                                end
                            end
                            g=g+1;
                        end
                    end
                    %遍历邻居节点NorthWest
                end
            end
            M=M+1;
        end
    end
    %=========================================
    %   从处理器基于计算误差，得到载荷向量
    %=========================================
    M=1;
    for I=1:1:Num_Slave_Processor_Row
        for J=1:1:Num_Slave_Processor_Col   %对每一个从处理器进行遍历
            % 从处理器下为凝聚自由度的载荷向量分配内存空间
            Slave_Processor(M).P_amI=zeros(Num_Displacement_Model,1);
            for m=1:1:Num_Displacement_Model %对每一个从处理器下的位移增量模式进行遍历
                for i=1:1:Slave_Processor(M).Num_Node_Local % 对每一个下属的节点的每一个自由度进行累加
                    for i1=1:2
                        dof_id=(i-1)*2+i1;
                        p0=Slave_Processor(M).Res(dof_id)*Slave_Processor(M).Displacement_Model{m}(dof_id);
%                         if M==2 && m==1
%                             p0
%                         end
                        Slave_Processor(M).P_amI(m)=Slave_Processor(M).P_amI(m)+p0;
                    end
                end
            end
            M=M+1;
        end
    end
    
    %=======================================
    %   主处理器获取从处理器的凝聚方程
    %=======================================
    %主处理器为凝聚方程建立保存空间
    Master_Processor.K=zeros(Num_Slave_Processor*Num_Displacement_Model);
    Master_Processor.P=zeros(Num_Slave_Processor*Num_Displacement_Model,1);
    M=1;
    ROW=1;
    COL=1;
    for I=1:1:Num_Slave_Processor_Row
        for J=1:1:Num_Slave_Processor_Col
            for m=1:1:Num_Displacement_Model
                % 获取凝聚方程的系数矩阵
                COL=1;
                for N=1:1:Num_Slave_Processor
                    for l=1:1:Num_Displacement_Model
                        Master_Processor.K(ROW,COL)=Slave_Processor(M).K_amI_alJ(m,l,N);
                        COL=COL+1;
                    end
                end
                % 获取凝聚方程的载荷向量
                Master_Processor.P(ROW)=Slave_Processor(M).P_amI(m);
                ROW=ROW+1;
            end
            M=M+1;
        end
    end
    %=======================================
    %   主处理器求解凝聚方程
    %=======================================
    Master_Processor.Displacement_Model_Coefficient=Master_Processor.K\Master_Processor.P;
    %=======================================
    %   从处理器获取位移增量系数，本地更新位移
    %=======================================
    
    M=1;
    for I=1:1:Num_Slave_Processor_Row
        for J=1:1:Num_Slave_Processor_Col
            % 获取位移增量系数
            Slave_Processor(M).Displacement_Model_Coefficient=zeros(Num_Displacement_Model,1);
            for m=1:1:Num_Displacement_Model
                Slave_Processor(M).Displacement_Model_Coefficient(m)=Master_Processor.Displacement_Model_Coefficient((M-1)*Num_Displacement_Model+m);
            end
            % 本地更新位移增量
            for m=1:1:Num_Displacement_Model
                Slave_Processor(M).U_n_old=Slave_Processor(M).U_n_old+Slave_Processor(M).Displacement_Model_Coefficient(m)*Slave_Processor(M).Displacement_Model{m};
            end
            M=M+1;
        end
    end
    
    %========================================
    %   邻居传递更新新的位移
    %========================================
    M=1;
    for I=1:1:Num_Slave_Processor_Row
        for J=1:1:Num_Slave_Processor_Col
            % 更新邻居信息 North
            if Slave_Processor(M).Neighbour_North~=-1
                N=Slave_Processor(M).Neighbour_North;
                g=1;
                for j=1:1:Num_Node_Extend_Y
                    for i=1:1:Slave_Processor(M).Num_X_Node_Local %遍历邻居消息传递的每一个点
                        neigh_id_in_local=Slave_Processor(M).Neighbour_North_Index_Calculation_List(g,1);
                        neigh_id_in_neigh=Slave_Processor(M).Neighbour_North_Index_Calculation_List(g,2);
                        if neigh_id_in_local>Slave_Processor(M).Num_Node_Cal_Inner
                            %若属于边界节点，则将信息更新到U_m_old中,自由度有两个,记得保存两个自由度
                            neigh_id_in_local=neigh_id_in_local-Slave_Processor(M).Num_Node_Cal_Inner;
                            for i1=1:2
                                Slave_Processor(M).U_m_old((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).U_n_old((neigh_id_in_neigh-1)*2+i1);
                            end
                        else
                            %若属于内部节点，则将信息更新到U_n_old中
                            for i1=1:2
                                Slave_Processor(M).U_n_old((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).U_n_old((neigh_id_in_neigh-1)*2+i1);
                            end
                        end
                        g=g+1;
                    end
                end
            end
            % 更新邻居信息 NorthEast
            if Slave_Processor(M).Neighbour_NorthEast~=-1
                N=Slave_Processor(M).Neighbour_NorthEast;
                g=1;
                for j=1:1:Num_Node_Extend_Y
                    for i=1:1:Num_Node_Extend_X %遍历邻居消息传递的每一个点
                        neigh_id_in_local=Slave_Processor(M).Neighbour_NorthEast_Index_Calculation_List(g,1);
                        neigh_id_in_neigh=Slave_Processor(M).Neighbour_NorthEast_Index_Calculation_List(g,2);
                        if neigh_id_in_local>Slave_Processor(M).Num_Node_Cal_Inner
                            %若属于边界节点，则将信息更新到U_m_old中,自由度有两个,记得保存两个自由度
                            neigh_id_in_local=neigh_id_in_local-Slave_Processor(M).Num_Node_Cal_Inner;
                            for i1=1:2
                                Slave_Processor(M).U_m_old((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).U_n_old((neigh_id_in_neigh-1)*2+i1);
                            end
                        else
                            %若属于内部节点，则将信息更新到U_n_old中
                            for i1=1:2
                                Slave_Processor(M).U_n_old((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).U_n_old((neigh_id_in_neigh-1)*2+i1);
                            end
                        end
                        g=g+1;
                    end
                end
            end
            % 更新邻居信息 East
            if Slave_Processor(M).Neighbour_East~=-1
                N=Slave_Processor(M).Neighbour_East;
                g=1;
                for j=1:1:Slave_Processor(M).Num_Y_Node_Local
                    for i=1:1:Num_Node_Extend_X %遍历邻居消息传递的每一个点
                        neigh_id_in_local=Slave_Processor(M).Neighbour_East_Index_Calculation_List(g,1);
                        neigh_id_in_neigh=Slave_Processor(M).Neighbour_East_Index_Calculation_List(g,2);
                        if neigh_id_in_local>Slave_Processor(M).Num_Node_Cal_Inner
                            %若属于边界节点，则将信息更新到U_m_old中,自由度有两个,记得保存两个自由度
                            neigh_id_in_local=neigh_id_in_local-Slave_Processor(M).Num_Node_Cal_Inner;
                            for i1=1:2
                                Slave_Processor(M).U_m_old((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).U_n_old((neigh_id_in_neigh-1)*2+i1);
                            end
                        else
                            %若属于内部节点，则将信息更新到U_n_old中
                            for i1=1:2
                                Slave_Processor(M).U_n_old((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).U_n_old((neigh_id_in_neigh-1)*2+i1);
                            end
                        end
                        g=g+1;
                    end
                end
            end
            % 更新邻居信息 SouthEast
            if Slave_Processor(M).Neighbour_SouthEast~=-1
                N=Slave_Processor(M).Neighbour_SouthEast;
                g=1;
                for j=1:1:Num_Node_Extend_Y
                    for i=1:1:Num_Node_Extend_X %遍历邻居消息传递的每一个点
                        neigh_id_in_local=Slave_Processor(M).Neighbour_SouthEast_Index_Calculation_List(g,1);
                        neigh_id_in_neigh=Slave_Processor(M).Neighbour_SouthEast_Index_Calculation_List(g,2);
                        if neigh_id_in_local>Slave_Processor(M).Num_Node_Cal_Inner
                            %若属于边界节点，则将信息更新到U_m_old中,自由度有两个,记得保存两个自由度
                            neigh_id_in_local=neigh_id_in_local-Slave_Processor(M).Num_Node_Cal_Inner;
                            for i1=1:2
                                Slave_Processor(M).U_m_old((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).U_n_old((neigh_id_in_neigh-1)*2+i1);
                            end
                        else
                            %若属于内部节点，则将信息更新到U_n_old中
                            for i1=1:2
                                Slave_Processor(M).U_n_old((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).U_n_old((neigh_id_in_neigh-1)*2+i1);
                            end
                        end
                        g=g+1;
                    end
                end
            end
            % 更新邻居信息 South
            if Slave_Processor(M).Neighbour_South~=-1
                N=Slave_Processor(M).Neighbour_South;
                g=1;
                for j=1:1:Num_Node_Extend_Y
                    for i=1:1:Slave_Processor(M).Num_X_Node_Local %遍历邻居消息传递的每一个点
                        neigh_id_in_local=Slave_Processor(M).Neighbour_South_Index_Calculation_List(g,1);
                        neigh_id_in_neigh=Slave_Processor(M).Neighbour_South_Index_Calculation_List(g,2);
                        if neigh_id_in_local>Slave_Processor(M).Num_Node_Cal_Inner
                            %若属于边界节点，则将信息更新到U_m_old中,自由度有两个,记得保存两个自由度
                            neigh_id_in_local=neigh_id_in_local-Slave_Processor(M).Num_Node_Cal_Inner;
                            for i1=1:2
                                Slave_Processor(M).U_m_old((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).U_n_old((neigh_id_in_neigh-1)*2+i1);
                            end
                        else
                            %若属于内部节点，则将信息更新到U_n_old中
                            for i1=1:2
                                Slave_Processor(M).U_n_old((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).U_n_old((neigh_id_in_neigh-1)*2+i1);
                            end
                        end
                        g=g+1;
                    end
                end
            end
            % 更新邻居信息 SouthWest
            if Slave_Processor(M).Neighbour_SouthWest~=-1
                N=Slave_Processor(M).Neighbour_SouthWest;
                g=1;
                for j=1:1:Num_Node_Extend_Y
                    for i=1:1:Num_Node_Extend_X %遍历邻居消息传递的每一个点
                        neigh_id_in_local=Slave_Processor(M).Neighbour_SouthWest_Index_Calculation_List(g,1);
                        neigh_id_in_neigh=Slave_Processor(M).Neighbour_SouthWest_Index_Calculation_List(g,2);
                        if neigh_id_in_local>Slave_Processor(M).Num_Node_Cal_Inner
                            %若属于边界节点，则将信息更新到U_m_old中,自由度有两个,记得保存两个自由度
                            neigh_id_in_local=neigh_id_in_local-Slave_Processor(M).Num_Node_Cal_Inner;
                            for i1=1:2
                                Slave_Processor(M).U_m_old((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).U_n_old((neigh_id_in_neigh-1)*2+i1);
                            end
                        else
                            %若属于内部节点，则将信息更新到U_n_old中
                            for i1=1:2
                                Slave_Processor(M).U_n_old((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).U_n_old((neigh_id_in_neigh-1)*2+i1);
                            end
                        end
                        g=g+1;
                    end
                end
            end
            % 更新邻居信息 West
            if Slave_Processor(M).Neighbour_West~=-1
                N=Slave_Processor(M).Neighbour_West;
                g=1;
                for j=1:1:Slave_Processor(M).Num_Y_Node_Local
                    for i=1:1:Num_Node_Extend_X %遍历邻居消息传递的每一个点
                        neigh_id_in_local=Slave_Processor(M).Neighbour_West_Index_Calculation_List(g,1);
                        neigh_id_in_neigh=Slave_Processor(M).Neighbour_West_Index_Calculation_List(g,2);
                        if neigh_id_in_local>Slave_Processor(M).Num_Node_Cal_Inner
                            %若属于边界节点，则将信息更新到U_m_old中,自由度有两个,记得保存两个自由度
                            neigh_id_in_local=neigh_id_in_local-Slave_Processor(M).Num_Node_Cal_Inner;
                            for i1=1:2
                                Slave_Processor(M).U_m_old((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).U_n_old((neigh_id_in_neigh-1)*2+i1);
                            end
                        else
                            %若属于内部节点，则将信息更新到U_n_old中
                            for i1=1:2
                                Slave_Processor(M).U_n_old((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).U_n_old((neigh_id_in_neigh-1)*2+i1);
                            end
                        end
                        g=g+1;
                    end
                end
            end
            % 更新邻居信息 NorthWest
            if Slave_Processor(M).Neighbour_NorthWest~=-1
                N=Slave_Processor(M).Neighbour_NorthWest;
                g=1;
                for j=1:1:Num_Node_Extend_Y
                    for i=1:1:Num_Node_Extend_X %遍历邻居消息传递的每一个点
                        neigh_id_in_local=Slave_Processor(M).Neighbour_NorthWest_Index_Calculation_List(g,1);
                        neigh_id_in_neigh=Slave_Processor(M).Neighbour_NorthWest_Index_Calculation_List(g,2);
                        if neigh_id_in_local>Slave_Processor(M).Num_Node_Cal_Inner
                            %若属于边界节点，则将信息更新到U_m_old中,自由度有两个,记得保存两个自由度
                            neigh_id_in_local=neigh_id_in_local-Slave_Processor(M).Num_Node_Cal_Inner;
                            for i1=1:2
                                Slave_Processor(M).U_m_old((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).U_n_old((neigh_id_in_neigh-1)*2+i1);
                            end
                        else
                            %若属于内部节点，则将信息更新到U_n_old中
                            for i1=1:2
                                Slave_Processor(M).U_n_old((neigh_id_in_local-1)*2+i1)=Slave_Processor(N).U_n_old((neigh_id_in_neigh-1)*2+i1);
                            end
                        end
                        g=g+1;
                    end
                end
            end
            M=M+1;
        end
    end
end
figure
set(gcf,'position',[50,50,1000,618]);
plot(1:1:Max_Iter,Master_Processor.Res,'sk--');
set(gca,'fontsize',24);
set(gca,'yscale','log');
%注意！载荷信息邻居节点也需要传递给本地节点