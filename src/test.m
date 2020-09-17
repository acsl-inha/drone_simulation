clear;
clc;

dt = 0.01;            
tf = 7;             
g = 9.8;              
global counter;
t_fail = 3;
t_detect = 0.2;     
counter = 0;
t = 0:dt:tf-0.01;
m = 23.56;           
n = length(t);
state = zeros(12,1);
global flag 
flag = true;
tic;
for k = 1:(n-1)
   [thrust_, phi_cmd_, the_cmd_, psi_cmd_] = compute_guidance(state,t(k),k);
end
toc;