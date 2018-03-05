function change2next = switch2nextTarget(SFCD,c,d,gs,aM,jM)
%whether vehicle is already in, whether the vehicle will stop in
isVehicle_in = 0;
isStop_in = 0;

%project the current global states along the line c-d
[xs, ys, ~, ~] = ThreeDTransform(c', d', gs);

%calculate whether the vehcile is within the c-d corridor
dist = norm(c-d);
if(xs.p>-SFCD && xs.p<dist+SFCD && ys.p>-SFCD && ys.p<SFCD)
    isVehicle_in = 1;
end

%calculate whether the vehcile's full stop position
if(isVehicle_in)
    Params = ParaGen(0, xs, aM, jM);
    stp_x = FindEndPoint(Params, xs);
    
    Params = ParaGen(0, ys, aM, jM);
    stp_y = FindEndPoint(Params, ys);
    
    %check whether the full stop position is with in the c-d corridor
    if(stp_x>-SFCD && stp_x<dist+SFCD && stp_y>-SFCD && stp_y<SFCD)
        isStop_in = 1;
    end
end
change2next = isVehicle_in && isStop_in;