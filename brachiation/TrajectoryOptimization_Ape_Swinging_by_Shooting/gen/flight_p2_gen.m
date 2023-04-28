function p2 = flight_p2_gen(in1)
%FLIGHT_P2_GEN
%    P2 = FLIGHT_P2_GEN(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    27-Apr-2023 14:47:49

th1 = in1(3,:);
th2 = in1(4,:);
x = in1(1,:);
y = in1(2,:);
p2 = [x+sin(th1)+sin(th2);y-cos(th1)-cos(th2)];
end
