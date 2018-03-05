%Generate the path reference using the given parameters
function [Ostate] = RefGen(Params, t)

if t<Params.T1
    Ostate.p = P3rdOrder(t, Params.p0, Params.v0, Params.a0, Params.j1);
    Ostate.v = P2ndOrder(t, Params.v0, Params.a0, Params.j1);
    Ostate.a = V2ndOrder(t, Params.a0, Params.j1);
elseif t>=Params.T1 && t<Params.T2
    Ostate.a = V2ndOrder(t - Params.T1, Params.a1, 0);
    Ostate.v = P2ndOrder(t - Params.T1, Params.v1, Params.a1, 0);
    Ostate.p = P3rdOrder(t - Params.T1, Params.p1, Params.v1, Params.a1, 0);
elseif t>=Params.T2 && t<Params.T3
    Ostate.a = V2ndOrder(t - Params.T2, Params.a2, Params.j3);
    Ostate.v = P2ndOrder(t - Params.T2, Params.v2, Params.a2, Params.j3);
    Ostate.p = P3rdOrder(t - Params.T2, Params.p2, Params.v2, Params.a2, Params.j3);
else
    Ostate.p = Params.p3 + Params.v3 * (t - Params.T3);
    Ostate.v = Params.v3;
    Ostate.a = 0;
end
