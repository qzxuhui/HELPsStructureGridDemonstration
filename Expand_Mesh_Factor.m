function [dX,dY,Num_X_Node,Num_Y_Node,Num_Node,Num_X_Element,Num_Y_Element,Num_Element]= Expand_Mesh_Factor(Mesh_Factor)
dX=Mesh_Factor(1);
dY=Mesh_Factor(2);
Num_X_Node=Mesh_Factor(3);
Num_Y_Node=Mesh_Factor(4);
Num_Node=Mesh_Factor(5);
Num_X_Element=Mesh_Factor(6);
Num_Y_Element=Mesh_Factor(7);
Num_Element=Mesh_Factor(8);
end

