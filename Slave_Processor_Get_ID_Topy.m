function Slave_Processor = Slave_Processor_Get_ID_Topy(Slave_Processor,Num_Slave_Processor_Row,Num_Slave_Processor_Col)
m=1;
for i=1:1:Num_Slave_Processor_Row
    for j=1:1:Num_Slave_Processor_Col
        Slave_Processor(m).Processor_ID=m;
        Slave_Processor(m).Topy_Row=i;
        Slave_Processor(m).Topy_Col=j;
        m=m+1;
    end
end

end

