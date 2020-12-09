%clc;clear;
%% ------------------------Initialization---------------------
SampleT = 0.02;
TotalT = 200/SampleT;

%parameters
T_v = 4; T_phi = 0.5;
UAVvmax = 50; UAVvmin = 25;
r = 1; r2 = 1; r3 = 0.01; r4 = 0.3; r5 = 0.01;
gamma1 = 0.2; gamma2 = 0.5; 
gamma3 = 0.5; gamma4 = 0.3; 
% gamma3 = 1; gamma4 = 1;
% gamma5 = 0.05; gamma6 = 0.01; 
gamma5 = 1; gamma6 = 10;
gamma7 = 0.01; gamma8 = 0.01; gamma9 = 0.5;
DDlambda = - 0.2;

%target states
TargetTrajectoryP=zeros(TotalT,2); %x,y;v,psi
xi1 = zeros(TotalT,1); xi2 = 5*ones(TotalT,1); xi3=zeros(TotalT,1); xi4=zeros(TotalT,1);

%UAV initial states
UAV1P = zeros(TotalT,2); UAV1P(1,1) = 0; UAV1P(1,2)= 250;
UAV1x1 = zeros(TotalT,1); UAV1x2 = zeros(TotalT,1); UAV1x3 = zeros(TotalT,1); UAV1x4 = zeros(TotalT,1);
UAV1x5 = zeros(TotalT,1); UAV1x6 = zeros(TotalT,1); UAV1x7 = zeros(TotalT,1);
UAV1u1 = zeros(TotalT,1); UAV1u2 = zeros(TotalT,1);
UAV1x1(1) = sqrt((UAV1P(1,1)-TargetTrajectoryP(1,2))^2 + (UAV1P(1,2)-TargetTrajectoryP(1,2))^2);
UAV1x2(1) = atan2(UAV1P(1,2)-TargetTrajectoryP(1,2), UAV1P(1,1)-TargetTrajectoryP(1,1));
UAV1x4(1) = -0.18;
UAV1x5(1) = 0;
UAV1x6(1) = 45; UAV1x7(1) = -0.6174;
UAValphas = zeros(TotalT,7);

%UAV desired states
UAV1x1d = 200; 
UAV1x2d = zeros(TotalT,1);
UAV1x2d(1) = UAV1x2(1);

%% -----------------------main----------------------------
for iter=2:TotalT
    %update target states
    TargetTrajectoryP(iter,1) = TargetTrajectoryP(iter-1,1) + xi2(iter)*cos(xi1(iter))*SampleT;
    TargetTrajectoryP(iter,2) = TargetTrajectoryP(iter-1,2) + xi2(iter)*sin(xi1(iter))*SampleT;
    
    %update functions
    %f
    UAV1f1 = UAV1x1(iter-1) + UAV1x3(iter-1)*SampleT;
    UAV1f2 = UAV1x2(iter-1) + UAV1x4(iter-1)*SampleT;
%     UAV1f3 = UAV1x3(iter-1) - UAV1x4(iter-1)*UAV1x6(iter-1)*sin(UAV1x2(iter-1)-UAV1x5(iter-1))*SampleT;
%     UAV1f4 = UAV1x4(iter-1) - (UAV1x2(iter-1)*UAV1x3(iter-1) ...
%         + UAV1x4(iter-1)*UAV1x6(iter-1)*cos(UAV1x2(iter-1)-UAV1x5(iter-1)))/UAV1x1(iter-1)*SampleT;
    UAV1f3 = UAV1x3(iter-1) - UAV1x6(iter-1)*cos(UAV1x2(iter-1)-UAV1x5(iter-1))/T_v*SampleT ...
        + UAV1x4(iter-1)*UAV1x6(iter-1)*sin(UAV1x2(iter-1)-UAV1x5(iter-1))*SampleT;
    UAV1f4 = UAV1x4(iter-1) + (UAV1x6(iter-1)*(-UAV1x3(iter-1)-UAV1x1(iter-1)/T_v)*sin(UAV1x2(iter-1)-UAV1x5(iter-1)) ...
        - UAV1x1(iter-1)*UAV1x4(iter-1)*UAV1x6(iter-1)*cos(UAV1x2(iter-1)-UAV1x5(iter-1)))/(UAV1x1(iter-1))^2*SampleT;
    UAV1f5 = UAV1x5(iter-1);
    UAV1f6 = (1-SampleT/T_v)*UAV1x6(iter-1);
    UAV1f7 = (1-SampleT/T_phi)*UAV1x7(iter-1);
%     UAV1f3bar = - UAV1x4(iter-1)*UAV1x6(iter-1)*sin(UAV1x2(iter-1)-UAV1x5(iter-1));
%     UAV1f4bar = - (UAV1x2(iter-1)*UAV1x3(iter-1)...
%         + UAV1x4(iter-1)*UAV1x6(iter-1)*cos(UAV1x2(iter-1)-UAV1x5(iter-1)))/UAV1x1(iter-1);
    UAV1f3bar =  - UAV1x6(iter-1)*cos(UAV1x2(iter-1)-UAV1x5(iter-1))/T_v ...
        + UAV1x4(iter-1)*UAV1x6(iter-1)*sin(UAV1x2(iter-1)-UAV1x5(iter-1));
    UAV1f4bar =  (UAV1x6(iter-1)*(-UAV1x3(iter-1)-UAV1x1(iter-1)/T_v)*sin(UAV1x2(iter-1)-UAV1x5(iter-1)) ...
        - UAV1x1(iter-1)*UAV1x4(iter-1)*UAV1x6(iter-1)*cos(UAV1x2(iter-1)-UAV1x5(iter-1)))/(UAV1x1(iter-1))^2;
    UAV1f6bar = -UAV1x6(iter-1)/T_v;
    UAV1f7bar = -UAV1x7(iter-1)/T_phi;
    %g
%     UAV1g1 = UAV1x6(iter-1)*sin(UAV1x2(iter-1)-UAV1x5(iter-1))*SampleT;
%     UAV1g2 = cos(UAV1x2(iter-1)-UAV1x5(iter-1))*SampleT;
%     UAV1g3 = UAV1x6(iter-1)/UAV1x1(iter-1)*cos(UAV1x2(iter-1)-UAV1x5(iter-1))*SampleT;
%     UAV1g4 = -sin(UAV1x2(iter-1)-UAV1x5(iter-1))/UAV1x1(iter-1)*SampleT;
    UAV1g1 = cos(UAV1x2(iter-1)-UAV1x5(iter-1))/T_v*SampleT;
    UAV1g2 = -sin(UAV1x2(iter-1)-UAV1x5(iter-1))*9.8*SampleT;
    UAV1g3 = sin(UAV1x2(iter-1)-UAV1x5(iter-1))/UAV1x1(iter-1)/T_v*SampleT;
    UAV1g4 = 9.8*cos(UAV1x2(iter-1)-UAV1x5(iter-1))/UAV1x1(iter-1)*SampleT;
    UAV1g5 = 9.8/UAV1x1(iter-1)*SampleT;
    UAV1g6 = SampleT/T_v;
    UAV1g7 = SampleT/T_phi;
    UAV1s = tan(UAV1x7(iter-1));
%     UAV1g1bar = UAV1x6(iter-1)*sin(UAV1x2(iter-1)-UAV1x5(iter-1));
%     UAV1g2bar = cos(UAV1x2(iter-1)-UAV1x5(iter-1));
%     UAV1g3bar = UAV1x6(iter-1)/UAV1x1(iter-1)*cos(UAV1x2(iter-1)-UAV1x5(iter-1));
%     UAV1g4bar = -sin(UAV1x2(iter-1)-UAV1x5(iter-1))/UAV1x1(iter-1);
    UAV1g1bar = cos(UAV1x2(iter-1)-UAV1x5(iter-1))/T_v;
    UAV1g2bar = -sin(UAV1x2(iter-1)-UAV1x5(iter-1))*9.8;
    UAV1g3bar = sin(UAV1x2(iter-1)-UAV1x5(iter-1))/UAV1x1(iter-1)/T_v;
    UAV1g4bar = 9.8*cos(UAV1x2(iter-1)-UAV1x5(iter-1))/UAV1x1(iter-1);
    UAV1g5bar = 9.8/UAV1x1(iter-1);
    UAV1g6bar = 1/T_v;
    UAV1g7bar = 1/T_phi;
    UAV1g8 = 1/(cos(UAV1x7(iter-1)))^2;
    %h
    UAV1h1 = -xi4(iter)*cos(xi1(iter)-UAV1x2(iter-1)) ...
        - xi2(iter)*(xi3(iter)-UAV1x4(iter-1))*sin(UAV1x2(iter-1)-xi1(iter));
    UAV1h2 = (UAV1x1(iter-1)*xi4(iter) - UAV1x3(iter-1)*xi2(iter))*sin(UAV1x2(iter-1)-xi1(iter))/(UAV1x1(iter-1))^2 ...
       - xi2(iter)*(xi3(iter)-UAV1x4(iter-1))*cos(UAV1x2(iter-1)-xi1(iter))/UAV1x1(iter-1);
    
    %update control law
    %step1
    UAV1x2d(iter) = mod(UAV1x2d(iter-1) + DDlambda*SampleT,2*pi);
    if (UAV1x2d(iter)>pi) 
        UAV1x2d(iter) = UAV1x2d(iter) - 2*pi;
    end
    Delta2 = mod(UAV1x2d(iter)-UAV1x2(iter-1),2*pi);
    if (Delta2>pi) 
        Delta2 = Delta2 - 2*pi;
    end
    if(Delta2>1)
        UAV1x2d(iter) = UAV1x2(iter-1) + 1;
    end
    if(Delta2<-1)
        UAV1x2d(iter) = UAV1x2(iter-1) - 1;
    end
    UAV1x2d(iter) = mod(UAV1x2d(iter),2*pi);
    if (UAV1x2d(iter)>pi) 
        UAV1x2d(iter) = UAV1x2d(iter) - 2*pi;
    end
    UAV1z1 = UAV1x1d - UAV1x1(iter-1);
    UAV1z2 = mod(UAV1x2d(iter) - UAV1x2(iter-1),2*pi);
    UAV1z2(UAV1z2>pi) = UAV1z2 - 2*pi;
    UAV1alpha1 = gamma1*UAV1z1;%d d
    UAV1alpha2 = DDlambda + gamma2*UAV1z2;% d lambda
    if(iter==2)
        UAV1alpha1_last = UAV1alpha1; UAV1alpha2_last = UAV1alpha2;
    end
    UAV1dalpha1 = (UAV1alpha1 - UAV1alpha1_last)/SampleT;
    UAV1dalpha2 = (UAV1alpha2 - UAV1alpha2_last)/SampleT;
    %step2
    if(iter==2)
        UAV1alpha3_last = 45; UAV1alpha4_last = 0;
    end
    UAV1z3 = UAV1alpha1 - UAV1x3(iter-1);
    UAV1z4 = UAV1alpha2 - UAV1x4(iter-1);
    UAV1G = [UAV1g1bar, UAV1g2bar; UAV1g3bar, UAV1g4bar];
%     UAV1G = [UAV1g1, UAV1g2; UAV1g3, UAV1g4];
%     if(abs(det(UAV1G))<0.000001)
%         UAV1G=[];
%     end
    UAV1Ginv = inv(UAV1G);
%     step2temp1 = gamma3*UAV1z3 + UAV1alpha1 - UAV1alpha1_last - UAV1f3 - UAV1h1 + UAV1x3(iter-1);
%     step2temp2 = gamma4*UAV1z4 + UAV1alpha2 - UAV1alpha2_last - UAV1f4 - UAV1h2 + UAV1x4(iter-1);
    step2temp1 = UAV1dalpha1 - UAV1f3bar - UAV1h1;
    step2temp2 = UAV1dalpha2 - UAV1f4bar - UAV1h2;
    UAV1alpha3 = UAV1Ginv(1,1)*step2temp1 + UAV1Ginv(1,2)*step2temp2 - gamma3*(UAV1z3*UAV1g1+UAV1z4*UAV1g3);
    UAV1alpha4 = UAV1Ginv(2,1)*step2temp1 + UAV1Ginv(2,2)*step2temp2 - gamma4*(UAV1z3*UAV1g2+UAV1z4*UAV1g4);
%     step2temp1 = UAV1z1 + UAV1dalpha1 - UAV1f3bar - UAV1h1;
%     step2temp2 = r*UAV1z2 + r*UAV1dalpha2 - UAV1f4bar - UAV1h2;
%     UAV1alpha3 = UAV1Ginv(1,1)*step2temp1 + UAV1Ginv(1,2)*step2temp2 + gamma3*(UAV1z3*UAV1g1bar + UAV1z4*UAV1g3bar);
    UAV1alpha3(UAV1alpha3>49.9)=49.9; UAV1alpha3(UAV1alpha3<25.1)=25.1; 
%     UAV1alpha4 = UAV1Ginv(2,1)*step2temp1 - UAV1Ginv(2,2)*step2temp2 + gamma4*(UAV1z3*UAV1g2bar + UAV1z4*UAV1g4bar);
    UAV1alpha4(UAV1alpha4>0.8)=0.8; UAV1alpha4(UAV1alpha4<-0.8)=-0.8;
    %step3
    if(iter==2)
        UAV1alpha5_last = 0; UAV1alpha6_last = 45;
    end
    UAV1z5 = UAV1f5 + UAV1g5*UAV1alpha4 - UAV1x5(iter-1);
    UAV1z6 = UAV1f6 + UAV1g6*UAV1alpha3 - UAV1x6(iter-1);
    a = 0.5*(UAVvmax-UAVvmin); b = 0.5*(UAVvmax+UAVvmin);
%     step3temp1 = UAV1z6*UAV1g6bar - (r2*a^2*(UAV1x6(iter-1)-b)*UAV1g6bar)/(a^2 - (UAV1x6(iter-1)-b)^2)^2;
%     step3temp2 = (UAV1z6*UAV1g6bar*(UAV1alpha3-UAV1alpha3_last)*(a^2 - (UAV1x6(iter-1)-b)^2)^2 ...
%         + r2*a^2*(UAV1x6(iter-1)-b)*UAV1f6bar)/...
%         (UAV1z6*UAV1g6bar*(a^2 - (UAV1x6(iter-1)-b)^2)^2 -  r2*a^2*(UAV1x6(iter-1)-b)*UAV1g6bar);
    UAV1alpha5 = UAV1alpha5_last + UAV1alpha4 - UAV1alpha4_last + gamma5*UAV1z5/UAV1g5;
    UAV1alpha5(UAV1alpha5>1)=1; UAV1alpha5(UAV1alpha5<-1)=-1;
    UAV1alpha6 = UAV1alpha6_last + UAV1alpha3 - UAV1alpha3_last + gamma6*UAV1z6/UAV1g6;
%     UAV1alpha6 = gamma6*step3temp1 + step3temp2;
    UAV1alpha6(UAV1alpha6>50)=50; UAV1alpha6(UAV1alpha6<25)=25;
    %step4
    if(iter==2)
        UAV1alpha7_last = 0;
    end
    UAV1z7 = UAV1alpha4 - UAV1s;
%     UAV1z7 = UAV1alpha5 - UAV1s;
    a7 = pi/3;
%     step4temp1 = UAV1z7*UAV1g8*UAV1g7bar - r3*a7^2*UAV1x7(iter-1)*UAV1g7bar/(a7^2-(UAV1x7(iter-1))^2)^2;
%     step4temp2 = (UAV1z7*(UAV1alpha5 - UAV1alpha5_last - UAV1g8*UAV1f7bar)*(a7^2-(UAV1x7(iter-1))^2)^2 ...
%         + r3*a7^2*UAV1x7(iter-1)*UAV1f7bar)...
%         /(UAV1z7*UAV1g8*UAV1g7bar*(a7^2-(UAV1x7(iter-1))^2)^2 - r3*a7^2*UAV1x7(iter-1)*UAV1f7bar);
%     UAV1alpha7 = gamma7*step4temp1 + step4temp2;
    UAV1alpha7 = (gamma7*UAV1z7 + UAV1alpha4 - UAV1alpha4_last - UAV1g8*UAV1f7bar)/(UAV1g8*UAV1g7bar);
    UAV1alpha7(UAV1alpha7>1.7)=1.7; UAV1alpha7(UAV1alpha7<-1.7)=-1.7;
    %step5
     if(iter==2)
        UAV1u1(iter-1) = b; UAV1u2(iter-1) = -0.22;
     end
%     UAV1z8 = UAV1alpha6 - UAV1u1(iter-1);
%     UAV1z9 = UAV1alpha7 - UAV1u2(iter-1);
%     step5temp1 = (UAV1z8*(a^2-(UAV1u1(iter-1)-b)^2)^2 - r4*a^2*(UAV1u1(iter-1)-b))/(a^2-(UAV1u1(iter-1)-b)^2)^2;
%     step5temp2 = UAV1z8*(UAV1alpha6 - UAV1alpha6_last)*(a^2-(UAV1u1(iter-1)-b)^2)^2 ...
%         /(UAV1z8*(a^2-(UAV1u1(iter-1)-b)^2)^2 - r4*a^2*(UAV1u1(iter-1)-b));
%     step5temp3 = (UAV1z9*(a7^2-UAV1u2(iter-1)^2)^2 - r5*a7^2*UAV1u2(iter-1))/(a7^2-UAV1u2(iter-1)^2)^2;
%     step5temp4 = UAV1z9*(UAV1alpha7-UAV1alpha7_last)*(a7^2-UAV1u2(iter-1)^2)^2 ...
%         /(UAV1z9*(a7^2-UAV1u2(iter-1)^2)^2 - r5*a7^2*UAV1u2(iter-1));
%     step5temp5 = UAV1u1(iter-1) + gamma8*step5temp1 + step5temp2;
%     step5temp5(step5temp5>49.9)=49.9; step5temp5(step5temp5<25.1)=25.1;
%     step5temp6= mod(UAV1u2(iter-1) + gamma9*step5temp3 + step5temp4,2*pi);
%     step5temp6(step5temp6>pi) = step5temp6-2*pi;
%     step5temp6(step5temp6>pi/3-0.01)=pi/3-0.01; step5temp6(step5temp6<-pi/3+0.01)=-pi/3+0.01;
%     UAV1u1(iter) = step5temp5; UAV1u2(iter) = step5temp6;

    UAV1u1(iter) = UAV1alpha3;
    temp = 0.5*SampleT*(UAV1alpha4 - UAV1alpha4_last)/tan(UAV1u2(iter-1)) + gamma9*UAV1z7;
    temp(temp>1) = 1; temp(temp<-1)=-1;
    UAV1u2(iter) = UAV1u2(iter-1) + asin(temp);
    
    %update alphas
    UAValphas(iter,1) = UAV1alpha1; UAValphas(iter,2) = UAV1alpha2;
    UAValphas(iter,3) = UAV1alpha3; UAValphas(iter,4) = UAV1alpha4;
    UAValphas(iter,5) = UAV1alpha5; UAValphas(iter,6) = UAV1alpha6;
    UAValphas(iter,7) = UAV1alpha7;
    UAV1alpha1_last = UAV1alpha1; UAV1alpha2_last = UAV1alpha2;
    UAV1alpha3_last = UAV1alpha3; UAV1alpha4_last = UAV1alpha4;
    UAV1alpha5_last = UAV1alpha5; UAV1alpha6_last = UAV1alpha6;
    UAV1alpha7_last = UAV1alpha7;

    %update UAV states
    UAV1x6(iter) = UAV1x6(iter-1) + (UAV1u1(iter)-UAV1x6(iter-1))/T_v*SampleT;
    UAV1x7(iter) = UAV1x7(iter-1) + (UAV1u2(iter)-UAV1x7(iter-1))/T_phi*SampleT;
    UAV1x5(iter) = UAV1x5(iter-1) + 9.8*tan(UAV1x7(iter-1))/UAV1x6(iter-1)*SampleT;
    UAV1P(iter,1) = UAV1P(iter-1,1) + UAV1x6(iter)*cos(UAV1x5(iter))*SampleT;
    UAV1P(iter,2) = UAV1P(iter-1,2) + UAV1x6(iter)*sin(UAV1x5(iter))*SampleT;
    UAV1x1(iter) = sqrt((UAV1P(iter,1)-TargetTrajectoryP(iter,1))^2 ...
        + (UAV1P(iter,2)-TargetTrajectoryP(iter,2))^2);
    UAV1x2(iter) = atan2(UAV1P(iter,2)-TargetTrajectoryP(iter,2), UAV1P(iter,1)-TargetTrajectoryP(iter,1));
    UAV1x3(iter) = (UAV1x1(iter) - UAV1x1(iter-1))/SampleT;
%     UAV1f3 + UAV1g1*UAV1u1(iter) + UAV1g2*tan(UAV1x7(iter)) + UAV1h1*SampleT;
    UAV1x4(iter) = (UAV1x2(iter) - UAV1x2(iter-1))/SampleT;
%     UAV1f4 + UAV1g3*UAV1u1(iter) + UAV1g4*tan(UAV1x7(iter)) + UAV1h2*SampleT;
%     UAV1x3(iter) = UAV1f3 + UAV1g1*UAV1u1(iter) + UAV1g2*tan(UAV1x7(iter)) + UAV1h1*SampleT;
%     UAV1x4(iter) = UAV1f4 + UAV1g3*UAV1u1(iter) + UAV1g4*tan(UAV1x7(iter)) + UAV1h2*SampleT;
%     UAV1x1(iter) = UAV1f1;
%     UAV1x2(iter) = UAV1f2;
    
%     plot([UAV1P(iter-1,2),UAV1P(iter,2)],[UAV1P(iter-1,1),UAV1P(iter,1)],'r'); hold on;
%     plot([TargetTrajectoryP(iter-1,2),TargetTrajectoryP(iter,2)],[TargetTrajectoryP(iter-1,1),TargetTrajectoryP(iter,1)],'r');
%     pause(0.01)
   
end

%% ------------------------plot figures------------------------

[ang,rad] = getAngle([UAV1P(10,2),UAV1P(10,1)],[UAV1P(9,2),UAV1P(9,1)]);                
psi = rad -(6.28/4);

figure(2);
plot(TargetTrajectoryP(:,2),TargetTrajectoryP(:,1),'k'); hold on;
plot(UAV1P(:,2),UAV1P(:,1),'r');
%waypoint = 
amount=size(UAV1P(:,1));
 for i=1:100
     
    xn = 0;     % UAV1P(2351,2);
    yn = 253; % UAV1P(2351,1);

    xn1 = 191; %UAV1P(618,2);
    yn1 = 500; %UAV1P(3032,1);

    delx = xn1-xn;
    dely = yn1-yn;

    ghe(1,1) = atan2d(dely,delx);
    ghe(1,1) = round(ghe(1,1));
 end
disp(ghe);


axis equal;
title('trajectory');

t = SampleT:SampleT:200;
% 
% figure(3);
% plot(t,UAV1x1);
% title('distance');
% 
% figure(4);
% plot(t,UAV1x2,'k'); hold on;
% plot(t,UAV1x2d,'r');
% title('lambda - desired lambda');
% 
% figure(5);
% plot(t,UAV1x3);
% title('D-distance');
% 
% figure(6);
% plot(t,UAV1x4);
% title('D-lambda');
% 
% figure(7);
% plot(t,UAV1x5); 
% title('psi');
% 
% figure(8);
% plot(t,UAV1x6, 'k'); hold on;
% plot(t,UAV1u1, 'r');
% title('v and desired-v');
% 
% figure(9);
% plot(t,UAV1x7,'k');hold on;
% plot(t,UAV1u2,'r');
% title('phi');
% 
% figure(10);
% plot(t, UAValphas(:,3),'r'); hold on;
% plot(t, UAValphas(:,6),'k');
% title('alpha1: v')
% 
% figure(11);
% plot(t, UAValphas(:,4),'r'); hold on;
% plot(t, UAValphas(:,5),'k');
% plot(t, UAValphas(:,7),'b');
% title('alpha1: psi')
