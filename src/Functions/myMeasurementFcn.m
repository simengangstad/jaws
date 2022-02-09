function y=myMeasurementFcn(x)
C_omega=[zeros(3,3) eye(3,3)];
H=[C_omega eye(3,3) zeros(3,6)];
y=H*x;
end