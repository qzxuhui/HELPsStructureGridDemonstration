function Slave_Processor = Slave_Processor_Get_Processor_Type(Slave_Processor,Num_Slave_Processor_Row,Num_Slave_Processor_Col)
m=0;
for I=1:1:Num_Slave_Processor_Row
    for J=1:1:Num_Slave_Processor_Col
        m=m+1;
        Slave_Processor(m).Processor_Type=5;
        if Slave_Processor(m).Topy_Col==1 && Slave_Processor(m).Topy_Row==1
            Slave_Processor(m).Processor_Type=1;
            continue;
        end
        if Slave_Processor(m).Topy_Col==Num_Slave_Processor_Col && Slave_Processor(m).Topy_Row==1
            Slave_Processor(m).Processor_Type=3;
            continue;
        end
        if Slave_Processor(m).Topy_Col==1 && Slave_Processor(m).Topy_Row==Num_Slave_Processor_Row
            Slave_Processor(m).Processor_Type=7;
            continue;
        end
        if Slave_Processor(m).Topy_Col==Num_Slave_Processor_Col && Slave_Processor(m).Topy_Row==Num_Slave_Processor_Row
            Slave_Processor(m).Processor_Type=9;
            continue;
        end
        if  Slave_Processor(m).Topy_Col==1
            Slave_Processor(m).Processor_Type=4;
            continue;
        end
        if  Slave_Processor(m).Topy_Row==1
            Slave_Processor(m).Processor_Type=2;
            continue;
        end
        if  Slave_Processor(m).Topy_Col==Num_Slave_Processor_Col
            Slave_Processor(m).Processor_Type=6;
            continue;
        end
        if  Slave_Processor(m).Topy_Row==Num_Slave_Processor_Row
            Slave_Processor(m).Processor_Type=8;
            continue;
        end        
    end
end
end

