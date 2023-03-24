function TM = shortestTime_synced(wp1,wp2,vM,aM)
%vM/aM = [horizon vertical];
[G2L,L2G,yaw,pitch] = global2local(wp1',wp2');
[vMc,aMc,jMc]=tiltConstraintsApprx(vM',aM',aM',pitch);

state.p = 0;
state.v = 0;
state.a = 0;

Params = ParaGen(norm(wp2-wp1), state, vMc(1), aMc(1));
TM = Params.T3;
end