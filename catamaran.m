%根据位置与艏向绘制双体船平面3DOF图
function []=catamaran(rou,psi)
    % x=3000; y=5000; psi=pi/4; %rad
    x=rou*cos(psi);
    y=rou*sin(psi);
    l=1700;%船长，垂线间长
    w=1060;%船宽，左右舷间吃水线宽
    d=775;%双体船片间距
    %A1=[x-3/10*l,y+w/2];
    ra=sqrt((-0.3*l)^2+(0.5*w)^2);
    aa=atan((5*w)/(-3*l));
    A1=[x-ra*cos(aa+psi),y-ra*sin(aa+psi)];
    %B1=[x+1/2*l,y+w/2];
    rb=sqrt((0.5*l)^2+(0.5*w)^2);
    ab=atan(w/l);
    B1=[x+rb*cos(ab+psi),y+rb*sin(ab+psi)];
    %C1=[x+7/10*l,y+d/2];
    rc=sqrt((0.7*l)^2+(0.5*d)^2);
    ac=atan((5*d)/(7*l));
    C1=[x+rc*cos(ac+psi),y+rc*sin(ac+psi)];
    %D1=[x+1/2*l,y+d-w/2];
    rd=sqrt((0.5*l)^2+(d-0.5*w)^2);
    ad=atan((d-0.5*w)/(0.5*l));
    D1=[x+rd*cos(ad+psi),y+rd*sin(ad+psi)];
    % A2=[x-3/10*l,y-w/2];
    A2=[x-ra*cos(-aa+psi),y-ra*sin(-aa+psi)];
    % B2=[x+5/10*l,y-w/2];
    B2=[x+rb*cos(-ab+psi),y+rb*sin(-ab+psi)];
    % C2=[x+7/10*l,y-d/2];
    C2=[x+rc*cos(-ac+psi),y+rc*sin(-ac+psi)];
    % D2=[x+1/2*l,y-d+w/2];
    D2=[x+rd*cos(-ad+psi),y+rd*sin(-ad+psi)];
    E=[A1',B1',C1',D1',D2',C2',B2',A2',A1'];
    plot(E(1,:),E(2,:),'k','LineWidth',1)%绘制船舶平面图
    hold on
    plot(x,y,'ko','MarkerFaceColor','k','MarkerSize',4)%绘制船舶重心
    hold on
    plot([x,x+l*cos(psi)],[y,y+l*sin(psi)],'r','LineWidth',2)%箭头杆
    hold on
    rf=sqrt((0.9*l)^2+(0.1*l)^2);
    af=atan(1/9);
    plot([x+rf*cos(-af+psi),x+l*cos(psi),x+rf*cos(af+psi)],[y+rf*sin(-af+psi),y+l*sin(psi),y+rf*sin(af+psi)],'r','LineWidth',2)%箭头尖
    hold off
end

