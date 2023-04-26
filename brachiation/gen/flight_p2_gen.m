function p2 = flight_p2_gen(in1)
%FLIGHT_P2_GEN
%    P2 = FLIGHT_P2_GEN(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    26-Apr-2023 00:21:22

th1 = in1(3,:);
th2 = in1(4,:);
x = in1(1,:);
y = in1(2,:);
t2 = th1+th2;
p2 = [x+sin(t2)+sin(th1);y-cos(t2)-cos(th1)];
end
