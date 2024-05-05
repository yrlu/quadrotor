function Params = ParaGen(vf, state, aM, jM)
%input:
%vf: velocity setpoint
%state: initial state
%aM,jM: max acc and jerk
%
%output:
%Params [T1,T2,T3,j1,j3,a(0~3),v(0~3),p(0~3)]:
%For reconstructing the trajectory

% Calculate the velocity when acc driectly comes to zero
%-------------------------------------------------------
v_stop = state.a^2 / 2 / jM * sign(state.a) + state.v;

% Deteremin the cruise direction and cruise acc
%-------------------------------------------------------
d = sign(vf - v_stop);
cruse_acc = d * aM;

% Calculate the phase 1 params
%-------------------------------------------------------
t1 = abs(cruse_acc - state.a) / jM;
Params.j1 = jM * sign(cruse_acc-state.a);
v1 = state.v + state.a * t1 + 0.5 * Params.j1 * t1^2;

% Calculate the pahse 3 params
%-------------------------------------------------------
t3 = abs(-cruse_acc) / jM;
Params.j3 = jM * sign(-cruse_acc);
v3bar = cruse_acc * t3 + 0.5 * Params.j3 * t3^2;

% Calculate the phase 2 params
%-------------------------------------------------------
v2bar = vf - v1 - v3bar;

if d == 0
    t2 = 0;
else
    t2 = v2bar / cruse_acc;
end

% There is a cruise phase iff t2 >= 0
%------------------------------------------------------
if t2 >= 0
    Params.T1 = t1;
    Params.T2 = t1+t2;
    Params.T3 = t1+t2+t3;
else
    % Here a_norm stands for the maximum reachable acc if the cruse acc now
    % cannot be reached
    %----------------------------------------------------------------------
    a_norm = d * sqrt(d * jM * (vf - state.v) + 0.5 * state.a^2);
    t1 = abs(a_norm - state.a) / jM;
    t2 = 0;
    t3 = abs(0 - a_norm)/jM;
    Params.T1 = t1;
    Params.T2 = t1+t2;
    Params.T3 = t1+t2+t3;
end

%Complete the params
%---------------------------------------------------------------------------
Params.a0 = state.a;
Params.v0 = state.v;
Params.p0 = state.p;

Params.a1 = V2ndOrder(Params.T1, Params.a0, Params.j1);
Params.v1 = P2ndOrder(Params.T1, Params.v0, Params.a0, Params.j1);
Params.p1 = P3rdOrder(Params.T1, Params.p0, Params.v0, Params.a0, Params.j1);

Params.a2 = V2ndOrder(Params.T2 - Params.T1, Params.a1, 0);
Params.v2 = P2ndOrder(Params.T2 - Params.T1, Params.v1, Params.a1, 0);
Params.p2 = P3rdOrder(Params.T2 - Params.T1, Params.p1, Params.v1, Params.a1, 0);

Params.a3 = V2ndOrder(Params.T3 - Params.T2, Params.a2, Params.j3);
Params.v3 = P2ndOrder(Params.T3 - Params.T2, Params.v2, Params.a2, Params.j3);
Params.p3 = P3rdOrder(Params.T3 - Params.T2, Params.p2, Params.v2, Params.a2, Params.j3);

end