function [xData_th1, yData_th2] = get_Data_from(data, L1, L2, flag)
    if string(flag) == "thetas"
        th1Data = data.get('th1').Values.Data;
        th2Data = data.get('th2').Values.Data;
    else %return references
        th1Data = data.get('ref_th1').Values.Data;
        th2Data = data.get('ref_th2').Values.Data; 
    end

    xData_th1 = (L1*cos(deg2rad(th1Data)) + L2*cos(deg2rad(th2Data)))*1000;
    yData_th2= (L1*sin(deg2rad(th1Data)) + L2*sin(deg2rad(th2Data)))*1000;
end