% %船舶操纵运动模型
% M*dV+C(v)*V+D(v)*V+g(Eta)=Tau+W
% V=[u,v,r]';
% Eta=[x,y,phi]';
% Tau=[tau,0,delta]';
% W=[w1,w2,w3]';
% g(Eta)=0;
% M=[m11 0 0;0 m22 0;0 0 m33];
% C(v)=[0 0 -m22*v;0 0 m11*u;m22*v -m11*u 0];
% D(v)=[d11 0 0;0 d22 0;0 0 d33];
% m11=m-Xdu;
% m22=m-Ydv;
% m33=Iz-Ndr;
% d11=-Xu-Xu1u1*abs(u); 
% d22=-Yv-Yv1v1*abs(v); 
% d33=-Nr-Nr1r1*abs(r);

% %状态变量
% x	%纵荡方向位置	m
% y     %横荡方向位置	m
% phi	%艏摇角度	rad
% u	%纵荡方向线速度	m/s
% v	%横荡方向线速度	m/s
% r	%艏摇角速度	rad/s

% %输入变量
% tau	%纵荡方向驱动力	N
% delta	%艏摇方向驱动扭矩	Nm

% %扰动变量 
% w1	%纵荡方向扰动	N
% w2	%横荡方向扰动	N
% w3	%艏摇方向扰动	Nm

%%风浪流干扰公式
%tau_wind=1/2*rou_air*Vr^2*[(C_surge*A_surge;C_sway*A_sway;C_yaw*A_yaw)]
%A_yaw=L_BP*A_sway %风载荷计算
%tau_current=1/2*rou_water*Vr^2*[(C_surge*A_surge;C_sway*A_sway;C_yaw*A_yaw)]
%A_yaw=L_BP^2*D %流载荷计算
%tau_wave=∑_(i=1:n) ∑_(j=1:n)Re{QTF_wave_drift(Bi,Bj,Ti,Tj)*ai*aj*exp[i*(wi-wj)*t-(phii-phij)]}

% %船舶运动数学模型参数
% m=1;   %船舶质量 	Kg
% Iz=1;  %艏摇转动惯量	Kgm^2
% xg=1;  %船舶重心距船中距离
% taou=1;%艏向驱动力
% taov=0;%横向驱动力
% taor=1;%艏摇方向驱动力
% 
% d11=1;%艏向干扰力
% d22=1;%横向干扰力
% d33=1;%艏摇方向干扰力
% Xu_d=1;%艏向附加质量，0.05~0.1*m，%纵荡方向附加质量	Kg
% Yv_d=1;%横向附加质量，0.9~1.2*m， %横荡方向附加质量	Kg
% Nr_d=1;%艏摇附加转动惯量，1~2*Iz， %艏摇方向附加质量	Kgm^2
% Yr_d=1;%绕Z轴的力对横向附加力矩，
% 
% Xu=1;%艏向匀速运动，艏向水作用力，%纵荡方向线性阻尼系数	Kg/s
% Yv=1;%横向匀速运动，横向水作用力，%横荡方向线性阻尼系数	Kg/s
% Nv=1;%横向匀速运动，绕Z轴水作用力矩
% Yr=1;%绕Z轴匀角速度运动，横向水作用力
% Nr=1;%绕Z轴匀角速度运动，绕Z轴水作用力矩，%艏摇方向线性阻尼系数	Kgm^2/s
	
% Xu1u1; %纵荡方向二次阻尼系数	Kg/m
% Yv1v1; %横荡方向二次阻尼系数	Kg/m
% Nr1r1; %艏摇方向二次阻尼系数	Kgm^2/s
% Xuu=1; %二阶修正系数
% Yvv=1; %船体左右对称时值为0
% Yvr=1; %船体左右对称时值为0
% Nvv=1; %船体左右对称时值为0
% Nvr=1; %船体左右对称时值为0
% Nrr=1; %船体左右对称时值为0

% yita=[x,y,fi]';%运动学变量
% niu=[u,v,r]';%动力学变量
% tao=[taou taov taor]';%驱动力
% d=[d1 d2 d3]';%环境干扰力 
% 
% J_yita=[cos(fi) -sin(fi) 0;sin(fi) -cos(fi) 0;0 0 1];
% M=[m-Xu_d 0 0;0 m-Yv_d m*xg-Yr_d;0 m*xg-Yr_d Iz-Nr_d];
% D=-[Xu 0 0;0 Yv Yr;0 Nv Nr];
% C_niu=[0 0 -m*(xg*r+v)+Yv_d*v+Yr_d*r;0 0 m*u-Xu_d*u;m*(xg*r+v)-Yv_d*v-Yr_d*r -m*u+Xu_d*u 0];
% Dn_niu=[Xuu*abs(u) 0 0;0 Yvv*abs(v)+Yrv*abs(r) Yvr*abs(v);0 Nvv*abs(v)+Nrv*abs(r) Nvr*abs(v)+Nrr*abs(r)];
% 
% yita_d=J_yita*niu;
% niu_d=J_yita*niu;
% M*niu_d=C_niu*niu-(D+Dn_niu)*niu+tao+d;

%%简化福森船舶运动模型
% dZ=A*Z+B*U;
% dX=C*Z;
% A=[-d11/m11 m22*r/m11 m22*v/m11;-m11*r/m22 -d22/m22 -m11*u/m22;(m22-m11)*v/m33 (m22-m11)*u/m33 -d33/m33];
% B=[1/m11 0 0;0 0 0;0 0 1/m33];
% C=[cos(phi) -sin(phi) 0;sin(phi) cos(phi) 0;0 0 1];
% Z=[u v r]; 
% U=[tau 0 delta]; 

% %一阶离散化模型
% Z(k+1)=Ak*Z(k)+Bk*U(k);
% W=Z*T;E=B*T;
% W(k+1)=Ak*W(k)+Ek*U(k);
% Ak=[1-d11*T/m11 m22*T*r(k)/m11 m22*T*v(k)/m11;-m11*T*r(k)/m22 1-d22*T/m22 -m11*T*u(k)/m22;
%     (m22-m11)*T*v(k)/m33 (m22-m11)*T*u(k)/m33 1-d33*T/m33];
% Bk=[T/m11 0 0;0 0 0;0 0 T/m33];
% Ek=[T*T/m11 0 0;0 0 0;0 0 T*T/m33];
% X(k)=C*W(k);
% C=[cos(phi) -sin(phi) 0;sin(phi) cos(phi) 0;0 0 1];

%预测模型
% ksi(k+1)=[w(k+1);u(k)];
% ksi(k+1)=AjT*ksi(k)+EjT*deltaU(k);
% yita(k)=C*ksi(k);
% AjT=[AkT EkT;0mn Im];
% EjT=[EkT Im];
% deltaU(k)=U(k)-U(k-1);
% yita(k)=[C*W(k+1);C*U(k)];
% Y(k)=psi*ksi(k)+thet*deltaU(k);
% Y(k)=[yita(k+1);yita(k+2);'...';yita(k+c);'...';yita(k+p)];
% psi=[C*AjT1;C*AjT2;'...';C*AjTc;'...';C*AjTp]];
% deltaU(k)=[deltau(k);deltau(k+1);'...';deltau(k+c)];
% thet=[C*EjT 0 0 0;C*AjT*EjT C*EjT 0 0;'...' '...' '...' '...';C*AjTc-1*EjT C*AjTc-2*EjT '...' C*EjT;
%     C*AjTc*EjT C*AjTc-1*EjT '...' C*AjT*EjT;C*AjTp-1*EjT C*AjTp-2*EjT '...' C*AjTp-c-1*EjT];

% %优化目标函数
% J(k)=miu1*(∑_(i=1:p)(yita(k+i)-yitar(k+i))^2)+miu2*(∑_(i=1:c)(deltaU(k+i))^2)+miu3*epsilon^2;
% %miu1,miu2,miu3是三部分的权重系数，epsilon^2是松弛因子
% c<p;u(k)<umax;u(k)>umin;
% deltau(k)<deltaumax;deltau(k)>deltaumin;
