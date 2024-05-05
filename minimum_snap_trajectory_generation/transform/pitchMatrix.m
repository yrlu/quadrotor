function A = pitchMatrix(pitch)
A = [cos(pitch), 0, sin(pitch), 0;
              0, 1,          0, 0;
    -sin(pitch), 0, cos(pitch), 0;
              0, 0,          0, 1;
    ];
end