function p1 = flight_p1_gen(in1)
%FLIGHT_P1_GEN
%    P1 = FLIGHT_P1_GEN(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    27-Apr-2023 14:47:49

th1 = in1(3,:);
x = in1(1,:);
y = in1(2,:);
p1 = [x+sin(th1);y-cos(th1)];
end
