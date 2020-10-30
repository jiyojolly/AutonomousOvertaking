clc; clear all;
close all;
c = [1;1];
G = [1;1;1];
F = [1 1;1 1];
% zono = zonotope(c,G);
hold off;
% plot(zono)
% zono = zonotope(c,F);
% plot(zono)
S2 = zonotope([1;1],...
               [1 0;...
                0 1]);
% R0 = zonotope([[0; 0; 0; 22; 0 ; 0; -2.1854; 0],...
%                       0.05*diag([1, 1, 1, 1, 1, 1, 1, 1])]);            
U = zonotope(interval([0.9; -0.25; -0.1; 0.25; -0.75], ...
                        [1.1; 0.25; 0.1; 0.75; -0.25]));
plot(U)