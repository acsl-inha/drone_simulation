dt = 0.01;            
tf = 7;             
g = 9.8;              
t_fail = 3;         
t_detect = 0.2;     
t = 1:dt:tf;
Deg2Rad = pi/180;
Rad2Deg = 1/Deg2Rad;
m = 23.56;           
n = length(t);
state = zeros(12,1);
tic;
for k = 1:(n-1)
    compute_guidance(state,t(k),k);
end
toc;