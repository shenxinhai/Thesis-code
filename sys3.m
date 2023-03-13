clc;clear;
%% 滑模：全局
% 神经网络：RBF(最小参数学习法)
% 事件触发：已加入
% 模型：二阶异构多智能体
% FUNCTIONAL PARAMETER
% dymamic parameter
m0=1;
l=0.25;
l1=0.3;
l2=0.35;
l3=0.4;
g=9.8;
c0=2;
gama=0.7;
miu=0.8;
et=30;
k=2*miu/gama;
cc=[-2 -1 0 1 2;
    -2 -1 0 1 2];
bb=10;
gx=3/(4*m0*l*l);
gx1=3/(4*m0*l1*l1);
gx2=3/(4*m0*l2*l2);
gx3=3/(4*m0*l3*l3);

% MASs parameter
B=[1 0 0 0;0 1 0 0;0 0 0 0;0 0 0 0];%控制矩阵
N=4;%智能体个数
A=[0 0 1 0;0 0 0 0;0 1 0 0;0 0 1 0];
b1=1;
b2=1;
b3=0;
b4=0;
L=[1 0 -1 0;0 0 0 0;0 -1 1 0;0 0 -1 1];
BL=B+L;%laplace matrix
BL_n=norm(BL,2);
c=50;
c1=20;
c2=20;
c3=20;
n=0;
n1=0;
n2=0;
n3=0;

%% executive parameter
time=10;
delta_time=0.001;
loop=time/delta_time;

%% initialize
%event-triggered instant
t_tk1=zeros(1,1001);
t_tk1(1)=1;
t_tk2=zeros(1,1001);
t_tk2(1)=1;
t_tk3=zeros(1,1001);
t_tk3(1)=1;
t_tk4=zeros(1,1001);
t_tk4(1)=1;

%event-triggered interval
delta1=zeros(2,1001);
delta2=zeros(2,1001);
delta3=zeros(2,1001);
delta4=zeros(2,1001);

%leader
x0=zeros(1,1001);
x0(1)=0;
v0=zeros(1,1001);
v0(1)=0;

%i=1
x1=zeros(1,loop+1);
v1=zeros(1,loop+1);
x1(1)=0.5;
v1(1)=0.5;


x1_event=zeros(1,loop+1);
v1_event=zeros(1,loop+1);
x1_event(1)=0;
v1_event(1)=0;

%i=2
x2=zeros(1,loop+1);
v2=zeros(1,loop+1);
x2(1)=1;
v2(1)=1;

x2_event=zeros(1,loop+1);
v2_event=zeros(1,loop+1);
x2_event(1)=0;
v2_event(1)=0;

%i=3
x3=zeros(1,loop+1);
v3=zeros(1,loop+1);
x3(1)=1.5;
v3(1)=1.5;

x3_event=zeros(1,loop+1);
v3_event=zeros(1,loop+1);
x3_event(1)=0;
v3_event(1)=0;

%i=4
x4=zeros(1,loop+1);
v4=zeros(1,loop+1);
x4(1)=2;
v4(1)=2;

x4_event=zeros(1,loop+1);
v4_event=zeros(1,loop+1);
x4_event(1)=0;
v4_event(1)=0;

%Sliding-mode controllor
u0=zeros(1,loop+1);
u0(1)=0;
u1=zeros(1,loop+1);
u1(1)=0;
u2=zeros(1,loop+1);
u2(1)=0;
u3=zeros(1,loop+1);
u3(1)=0;
u4=zeros(1,loop+1);
u4(1)=0;

%Sliding-mode controllor
u1tk=zeros(1,loop+1);
u1tk(1)=0;
u2tk=zeros(1,loop+1);
u2tk(1)=0;
u3tk=zeros(1,loop+1);
u3tk(1)=0;
u4tk=zeros(1,loop+1);
u4tk(1)=0;

%known function
f1=zeros(1,loop+1);
f2=zeros(1,loop+1);
f3=zeros(1,loop+1);
f4=zeros(1,loop+1);


%RBF parameter
fin1=zeros(1,loop+1);
fin2=zeros(1,loop+1);
fin3=zeros(1,loop+1);
fin4=zeros(1,loop+1);

%% 执行器
for T=1:1:loop
% es=exp(-0.009*T); %event_triggered threshold
es=exp(-0.0012*T);
% es=0.05;
%% es=alpha;
t=T*delta_time;

%% leader
u0(T)=cos(0.1*t);
f0(T)=-3*c0/(4*m0*l*l)*x0(T)-3*g/(4*l);
x0(T+1)=x0(T)+v0(T)*delta_time;
v0(T+1)=v0(T)+(f0(T)+gx*u0(T))*delta_time;
% u0(T)=cos(0.1*t);
% f0(T)=-3*c0/(4*m0*l*l)*x0(T)-3*g/(4*l);
% x0(T+1)=0;
% v0(T+1)=0;


%% error
ex1(T)=A(1,2)*(x1(T)-x2(T))+A(1,3)*(x1(T)-x3(T))+...
    A(1,4)*(x1(T)-x4(T))+b1*(x1(T)-x0(T));%跟踪误差
ev1(T)=A(1,2)*(v1(T)-v2(T))+A(1,3)*(v1(T)-v3(T))+...
    A(1,4)*(v1(T)-v4(T))+b1*(v1(T)-v0(T));
ex2(T)=A(2,1)*(x2(T)-x1(T))+A(2,3)*(x2(T)-x3(T))+...
    A(2,4)*(x2(T)-x4(T))+b2*(x2(T)-x0(T));%跟踪误差
ev2(T)=A(2,1)*(v2(T)-v1(T))+A(2,3)*(v2(T)-v3(T))+...
    A(2,4)*(v2(T)-v4(T))+b2*(v2(T)-v0(T));
ex3(T)=A(3,1)*(x3(T)-x1(T))+A(3,2)*(x3(T)-x2(T))+...
    A(3,4)*(x3(T)-x4(T))+b3*(x3(T)-x0(T));%跟踪误差
ev3(T)=A(3,1)*(v3(T)-v1(T))+A(3,2)*(v3(T)-v2(T))+...
    A(3,4)*(v3(T)-v4(T))+b3*(v3(T)-v0(T));
ex4(T)=A(4,1)*(x4(T)-x1(T))+A(4,2)*(x4(T)-x2(T))+...
    A(4,3)*(x4(T)-x3(T))+b4*(x4(T)-x0(T));%跟踪误差
ev4(T)=A(4,1)*(v4(T)-v1(T))+A(4,2)*(v4(T)-v2(T))+...
    A(4,3)*(v4(T)-v3(T))+b4*(v4(T)-v0(T));

e1=[ex1;ev1];
e2=[ex2;ev2];
e3=[ex3;ev3];
e4=[ex4;ev4];

%% 滑模面
s10=c*ex1(1)+ev1(1);
s20=c1*ex2(1)+ev2(1);
s30=c2*ex3(1)+ev3(1);
s40=c3*ex4(1)+ev4(1);

q1=s10*exp(-2*T*delta_time);
q2=s20*exp(-2*T*delta_time);
q3=s30*exp(-2*T*delta_time);
q4=s40*exp(-1.5*T*delta_time);

% s1=c*ex1(T)+ev1(T)-q1;
% s2=c1*ex2(T)+ev2(T)-q2;
% s3=c2*ex3(T)+ev3(T)-q3;
% s4=c3*ex4(T)+ev4(T)-q4;
s1=c*ex1(t_tk1(T))+ev1(t_tk1(T))-q1;
s2=c1*ex2(t_tk2(T))+ev2(t_tk2(T))-q2;
s3=c2*ex3(t_tk3(T))+ev3(t_tk3(T))-q3;
s4=c3*ex4(t_tk4(T))+ev4(t_tk4(T))-q4;
%% dynamic fuction (known)
f1(T)=-3*c0/(4*m0*l*l)*x1(T)-3*g/(4*l);
f2(T)=-3*c0/(4*m0*l1*l1)*x2(T)-3*g/(4*l1);
f3(T)=-3*c0/(4*m0*l2*l2)*x3(T)-3*g/(4*l2);
f4(T)=-3*c0/(4*m0*l3*l3)*x4(T)-3*g/(4*l3);

%% disturbance
d1=3*(v1(T)*sin(x1(T))+0.1*cos(t))/(4*m0*l*l)+5;
d2=3*(v2(T)*sin(x2(T))+0.1*cos(t))/(4*m0*l1*l1)+5;
d3=3*(v3(T)*sin(x3(T))+0.1*cos(t))/(4*m0*l2*l2)+5;
d4=3*(v4(T)*sin(x4(T))+0.1*cos(t))/(4*m0*l3*l3)+5;

%% RBF input
xi1=[x1(T);v1(T)];
xi2=[x2(T);v2(T)];
xi3=[x3(T);v3(T)];
xi4=[x4(T);v4(T)];

%% RBF function
h1=zeros(5,1);
h2=zeros(5,1);
h3=zeros(5,1);
h4=zeros(5,1);

%% output of Gauss function
for j=1:1:5
    h1(j)=exp(-norm(xi1-cc(:,j))^2/(2*bb*bb));
end

for j=1:1:5
    h2(j)=exp(-norm(xi2-cc(:,j))^2/(2*bb*bb));
end

for j=1:1:5
    h3(j)=exp(-norm(xi3-cc(:,j))^2/(2*bb*bb));
end

for j=1:1:5
    h4(j)=exp(-norm(xi4-cc(:,j))^2/(2*bb*bb));
end

%% fin=norm(W)^2
fin1(T+1)=fin1(T)+(gama*BL_n/2*s1^2*(h1'*h1)-k*gama*fin1(T))*delta_time;
fin2(T+1)=fin2(T)+(gama*BL_n/2*s2^2*(h2'*h2)-k*gama*fin2(T))*delta_time;
fin3(T+1)=fin3(T)+(gama*BL_n/2*s3^2*(h3'*h3)-k*gama*fin3(T))*delta_time;
fin4(T+1)=fin4(T)+(gama*BL_n/2*s4^2*(h4'*h4)-k*gama*fin4(T))*delta_time;


%% 控制器

u1(T+1)=1/(2)*(A(1,2)*u2(T)+A(1,3)*u3(T)+A(1,4)*u4(T)+...
    1/gx*(-0.5*s1*fin1(T+1)*(h1'*h1)+v0(T+1)-c*ev1(T)-f1(T)-miu*s1-et*sign(s1)+q1));
u2(T+1)=1/(1)*(A(2,1)*u1(T)+A(2,3)*u3(T)+A(2,4)*u4(T)+...
    1/gx1*(-0.5*s2*fin2(T+1)*(h2'*h2)+v0(T+1)-c1*ev2(T)-f2(T)-miu*s2-et*sign(s2)+q2));
u3(T+1)=1/(1)*(A(3,1)*u1(T)+A(3,2)*u2(T)+A(3,4)*u4(T)+...
    1/gx2*(-0.5*s3*fin3(T+1)*(h3'*h3)+v0(T+1)-c2*ev3(T)-f3(T)-miu*s3-et*sign(s3)+q3));
u4(T+1)=1/(1)*(A(4,1)*u1(T)+A(4,2)*u2(T)+A(4,3)*u3(T)+...
    1/gx3*(-0.5*s4*fin4(T+1)*(h4'*h4)+v0(T+1)-c3*ev4(T)-f4(T)-miu*s4-et*sign(s4)+q4));

%% event-triggered rules update
delta1(1,T)=x1(t_tk1(T))-x1(T);
delta1(2,T)=v1(t_tk1(T))-v1(T);
delta2(1,T)=x2(t_tk2(T))-x2(T);
delta2(2,T)=v2(t_tk2(T))-v2(T);
delta3(1,T)=x3(t_tk3(T))-x3(T);
delta3(2,T)=v3(t_tk3(T))-v3(T);
delta4(1,T)=x4(t_tk4(T))-x4(T);
delta4(2,T)=v4(t_tk4(T))-v4(T);

ed1=norm(delta1(:,T),2);
ed2=norm(delta2(:,T),2);
ed3=norm(delta3(:,T),2);
ed4=norm(delta4(:,T),2);

% ed1=abs(ev1(T));
% ed2=abs(ev2(T));
% ed3=abs(ev3(T));
% ed4=abs(ev4(T));
%% 判断是否更新
if ed1<=es
    t_tk1(T+1)=t_tk1(T);
    u1(T+1)=u1(T);
else
    t_tk1(T+1)=T;
    u1(T+1)=u1(T+1);
    n=n+1;
end

if ed2<=es
    t_tk2(T+1)=t_tk2(T);
    u2(T+1)=u2(T);
else
    t_tk2(T+1)=T; 
    u2(T+1)=u2(T+1);
    n1=n1+1;
end

if ed3<=es
    t_tk3(T+1)=t_tk3(T);
    
else
    t_tk3(T+1)=T;
    u3(T+1)=u3(T+1);
    n2=n2+1;
end

if ed4<=es
    t_tk4(T+1)=t_tk4(T);
    u4(T+1)=u4(T);
else
    t_tk4(T+1)=T;
    u4(T+1)=u4(T+1);
    n3=n3+1;
end

%% state update
x1(T+1)=x1(T)+v1(T)*delta_time;
v1(T+1)=v1(T)+(f1(T)+gx*u1(T+1)+d1)*delta_time;

x2(T+1)=x2(T)+v2(T)*delta_time;
v2(T+1)=v2(T)+(f2(T)+gx1*u2(T+1)+d2)*delta_time;

x3(T+1)=x3(T)+v3(T)*delta_time;
v3(T+1)=v3(T)+(f3(T)+gx2*u3(T+1)+d3)*delta_time;

x4(T+1)=x4(T)+v4(T)*delta_time;
v4(T+1)=v4(T)+(f4(T)+gx3*u4(T+1)+d4)*delta_time;

end
%%
T = 1:loop;
t=T*0.001;
figure(1);
%% 状态
plot(t,x0(1,T),'r',t,x1(1,T),'g',t,x2(1,T),'b',t,x3(1,T),'c',t,x4(1,T),'m');
xlabel('\it t/s','FontName','Times New Roman','FontSize',12);
ylabel('Agent horizontal angle position/\itrad','FontName','Times New Roman','FontSize',12);
legend('leader','Agent1','Agent2','Agent3','Agent4','FontName','Times New Roman','FontSize',12);
%% 控制器
figure(2);
plot(t,u1(1,T),'r',t,u2(1,T),'b',t,u3(1,T),'c',t,u4(1,T),'m');
xlabel('\it t/s','FontName','Times New Roman','FontSize',12);
ylabel('Agent output of controllor/\itN·m','FontName','Times New Roman','FontSize',12);
legend('Agent1','Agent2','Agent3','Agent4','FontName','Times New Roman','FontSize',12);

%% 速度
figure(3);
plot(t,v0(1,T),'r',t,v1(1,T),'g',t,v2(1,T),'b',t,v3(1,T),'c',t,v4(1,T),'m');
xlabel('\it t/s','FontName','Times New Roman','FontSize',12);
ylabel('Agents horizontal angle velocity/\itrad/s','FontName','Times New Roman','FontSize',12);
legend('leader','Agent1','Agent2','Agent3','Agent4','FontName','Times New Roman','FontSize',12);

% %% 速度
% figure(4);
% plot(T,s1(1,:),'r',T,s2(1,:),'g',T,s3(1,:),'b',T,s4(1,:),'c');
% xlabel('t/s','FontName','Times New Roman','FontSize',12);
% ylabel('Agents horizontal angle','FontName','Times New Roman','FontSize',12);
% legend('Agent1','Agent2','Agent3','Agent4','FontName','Times New Roman','FontSize',12);
