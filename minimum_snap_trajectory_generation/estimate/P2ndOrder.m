function [x] = P2ndOrder(t,x0,v0,a)

x = x0+v0*t+0.5*a*t^2;