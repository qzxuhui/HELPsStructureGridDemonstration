function Slave_Processor = Slave_Processor_Get_Neighbour_ID(Slave_Processor,Num_Slave_Processor_Row,Num_Slave_Processor_Col)
m=1;
for i=1:1:Num_Slave_Processor_Row
    for j=1:1:Num_Slave_Processor_Col
        Slave_Processor(m).Neighbour_North=Slave_Processor(m).Processor_ID-Num_Slave_Processor_Col;
        Slave_Processor(m).Neighbour_NorthEast=Slave_Processor(m).Processor_ID-Num_Slave_Processor_Col+1;
        Slave_Processor(m).Neighbour_East=Slave_Processor(m).Processor_ID+1;
        Slave_Processor(m).Neighbour_SouthEast=Slave_Processor(m).Processor_ID+Num_Slave_Processor_Col+1;
        Slave_Processor(m).Neighbour_South=Slave_Processor(m).Processor_ID+Num_Slave_Processor_Col;
        Slave_Processor(m).Neighbour_SouthWest=Slave_Processor(m).Processor_ID+Num_Slave_Processor_Col-1;
        Slave_Processor(m).Neighbour_West=Slave_Processor(m).Processor_ID-1;
        Slave_Processor(m).Neighbour_NorthWest=Slave_Processor(m).Processor_ID-Num_Slave_Processor_Col-1;
        if Slave_Processor(m).Topy_Col==1
            Slave_Processor(m).Neighbour_SouthWest=-1;
            Slave_Processor(m).Neighbour_West=-1;
            Slave_Processor(m).Neighbour_NorthWest=-1;
        end
        if Slave_Processor(m).Topy_Col==Num_Slave_Processor_Col
            Slave_Processor(m).Neighbour_SouthEast=-1;
            Slave_Processor(m).Neighbour_East=-1;
            Slave_Processor(m).Neighbour_NorthEast=-1;
        end
        if Slave_Processor(m).Topy_Row==1
            Slave_Processor(m).Neighbour_North=-1;
            Slave_Processor(m).Neighbour_NorthEast=-1;
            Slave_Processor(m).Neighbour_NorthWest=-1;
        end
        if Slave_Processor(m).Topy_Row==Num_Slave_Processor_Row
            Slave_Processor(m).Neighbour_South=-1;
            Slave_Processor(m).Neighbour_SouthEast=-1;
            Slave_Processor(m).Neighbour_SouthWest=-1;
        end
        m=m+1;
    end
end
end

