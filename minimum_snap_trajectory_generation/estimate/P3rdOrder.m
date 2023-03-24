function [x] = P3rdOrder(t,x0,v0,a0,j)

x = x0+v0*t+0.5*a0*t^2+1/6*j*t^3;