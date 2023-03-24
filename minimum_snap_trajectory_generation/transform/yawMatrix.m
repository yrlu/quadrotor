function A = yawMatrix(yaw)
A = [cos(yaw), sin(yaw),  0, 0;
    -sin(yaw), cos(yaw),  0, 0;
    0,        0,  1, 0;
    0,        0,  0, 1;
    ];
end