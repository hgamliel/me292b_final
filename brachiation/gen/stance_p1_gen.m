function p1 = stance_p1_gen(in1)
%STANCE_P1_GEN
%    P1 = STANCE_P1_GEN(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    29-Apr-2023 23:28:24

th1 = in1(1,:);
p1 = [sin(th1);-cos(th1)];
end
