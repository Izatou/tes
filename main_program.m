%Main
clc
clear
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%path parameters
%size [y,x,z] and resolution
sizeE=[150 150 30];
d_grid=1;
%Starting point
x0=5;
y0=5;
z0=10;
%Arrival point
xend=125;
yend=140;
zend=10;


%% Defining environment variables
start = [x0,y0];       % start position
goal = [xend, yend];   % goal position
%n = 2;                 % no. of obstacles


%Number of points with low elevation around start and end point area 
n_low=3;
%Theta * cost weights
kg=1;
kh=1.25;
ke=sqrt((xend-x0)^2+(yend-y0)^2+(zend-z0)^2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Map definition
%Average flight altitude
h=max(z0,zend);
%Points coordinates in [y,x,z] format
P0=[y0 x0 z0];
Pend=[yend xend zend];
%Generate map

keys = {'a',... 
        'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u',... 
        'v', 'w', 'x', 'y', 'z','aa', 'ab', 'ac', 'ad', 'ae', 'af', 'ag', 'ah', 'ai', 'aj', 'ak', 'al', 'am','an', 'ao', ...
        'ap', 'aq','ar','as','at', 'au', 'av', 'aw', 'ax','ay','az','ba','bb','bc','bd','be','bf','bg','bh', 'bi',...
         'bj','bk','bl','bm','bn','bo','bp','bp','bq','br','bs','bt','bu','bv','bw','bx','by','bz','ca','cb',...
         'cc', 'cd', 'ce', 'cf', 'cg', 'ch', 'ci', 'cj', 'ck', 'cl', 'cm', 'cn', 'co','cp','cq','cr','cs','ct','cu','cv',...
         'cw','cx','cy','cz','da','dd','dd','dd','de','df','dg','dh', 'di','dj','dk','dl','dm','dn','do','dp'...
         'dq','dr','ds','dt','du','dv','dw','dx','dy','dz','ea','eb','ec','ed','ee','ef','eg','eh', 'ei','ej',...
         'ek'};
values = {start,...
                 [20,10],[20,30], [40,30], [40,10],...
                 [50,5], [50,25], [65,25], [65,5],....
                 [70,10],[70,30], [90,30], [90,10],...
                 [10,40], [10,60], [30,60], [30,40],...
                 [40,40], [40,60], [75,60], [75,45],... %5obstacle
                 [80,35], [80,75], [95,75], [95,35],...
                 [10,70], [10,90], [15,90], [15,70],...
                 [20,70], [20,90], [40,90], [40,70],...
                 [50,65], [50,90], [70,90], [70,65],...
                 [75,80], [75,95], [90,95], [90,80],... %10obstacle
                 [100,10], [100,30], [115,30], [115,10],...
                 [120,20], [120,40], [135,40], [135,20],...
                 [105,45], [105,60], [125,60], [125,45],...
                 [135,40], [135,50], [145,50], [145,40],...
                 [110,70], [110,80], [130,80], [130,70],... %15obstacle
                 [100,90], [100,110], [110,110], [110,90],...
                 [10,100], [10,120], [20,120], [20,100],...
                 [20,125], [20,140], [40,140], [40,125],...
                 [55,120], [55,130], [80,130], [80,120],...
                 [110,120], [110,140], [120,140], [120,110],... %20obstacle
                 [120,5], [120,15], [130,15], [130,50],...
                 [30,95], [30,100], [60,110], [60,95],...
                 [25,110], [25,120], [40,120], [40,110],...
                 [45,105], [45,115], [60,115], [60,105],...
                 [120,90], [120,100], [130,100], [130,90],... %25obstacle
                 [65,100], [65,110], [80,110], [80,100],...
                 [50,135], [50,145], [60,145], [60,135],...
                 [100,115], [100,130], [105,130], [105,115],...
                 [125,130], [125,145], [140,145], [140,130],...
                 [140,5], [140,30], [145,30], [145,5],... %30obstacle
                 [20,10], [20,10], [20,10], [20,10],...
                 [20,10], [20,10], [20,10], [20,30],...
                 [20,30], [20,30], [20,30], [20,30],...
                 [20,30], [20,30], [40,30], [40,30],...
                 [40,30], [40,30], [40,30], [40,30],...
                 goal};
key = {'a','b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't'};        
nodes={          [20,10], [20,10], [20,10], [20,10],...
                 [20,10], [20,10], [20,10], [20,30],...
                 [20,30], [20,30], [20,30], [20,30],...
                 [20,30], [20,30], [40,30], [40,30],...
                 [40,30], [40,30], [40,30], [40,30],...
                };  
titik = [20,10,5];
%values = {};
%values = [values start];
[E,E_safe,E3d,E3d_safe]= safezone(sizeE,d_grid,h,P0,Pend,n_low,values);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%[xs,ys,zs ] = E3d_safe;
%disp((E3d_safe));
%mx = 2;
%for i=1:sizeE(1)
%    for j=1:sizeE(2)
%        if E3d_safe(j,i,1)== 1
%           %if  values(i,j) == [i,j] 
%               values = [values [i,j]];
%               mx = mx + 1;
%           %end
%        end      
%    end
%end
%keys = {};
%for i=1:mx
%    keys = [keys,strcat('n',int2str(i))];    
%end
%values = [values,goal];
disp(size(keys));
disp(size(values));
disp(size(nodes));
Map = containers.Map(keys, values);
len = Map.values;
%node
%a= [20 20 20];
%b = [10 10 10];
c =[7 8 9];

%% Building all the obstacle edges
length = size(keys);    % this contains the number of nodes
edges = [];
for i = 2:(length(2)-2)   
    temp = [values{1,i}(1), values{1,i+1}(1), values{1,i}(2), values{1,i+1}(2)];
    edges = vertcat(edges, temp);                
end
% Removing edges which are not obstacle edges
sizeEdges = size(edges);
i = 4;
while i < (sizeEdges(1)-1)
    [edges,ps] = removerows(edges,'ind',i);
    sizeEdges = size(edges);
    i = i + 3;
end
% Adding 1 edge pair in each obstacle which were not added in the earlier
% for loop
i = 2;
while i < length(2)-2    
    temp = [values{1,i}(1), values{1,i+3}(1), values{1,i}(2), values{1,i+3}(2)];
    edges = vertcat(edges, temp);
    i = i + 4;    
end
%% Calculating valid edges -> adding them to the graph
ledgeSize = size(edges);
noEdges = ledgeSize(1);
G = graph();

for i = 1:length(2)    
    for j = (i + 1):length(2)        
        % find equation of the edge to be checked
        p1 = values{1,i};
        q1 = values{1,j};
        m1 = (q1(2)-p1(2))/(q1(1)-p1(1));
        c1 = p1(2) - m1*(p1(1));
        %%%%%%%%%%%%
        flag = 1;    % flag to check if the edge has any intersection with any other edge;
                     % '1' means no intersection
        % need to compare with the edges   
        for k = 1:noEdges            
            ed = edges(k,:);            
            m2 = (ed(4) - ed(3))/(ed(2) - ed(1));
            if(ed(2)==ed(1))
                m2 = 1e+10;
            end
            c2 = ed(3) - m2*ed(1);             
            if m1==m2 %% ignoring 
                t = 1;
            else                
                %%%%%%
                temp1 = ed(3) - m1*ed(1) - c1;
                temp2 = ed(4) - m1*ed(2) - c1;                
                temp3 = p1(2) - m2*p1(1) - c2;
                temp4 = q1(2) - m2*q1(1) - c2;
                
                if (sign(temp1) ~= sign(temp2)) &&  sign(temp1)~=0 && sign(temp2)~=0 && (sign(temp3) ~= sign(temp4)) &&  sign(temp3)~=0 && sign(temp4)~=0
                    flag = 0;
                    break
                end
                %%%%%%
            end
        end
        if flag==1
            G = addedge(G,keys{i}, keys{j});
        end                
    end    
end


%% Removing the diagonals of the obstacle from the visible edges
length = size(key);    % this contains the number of nodes
i = 2;
while i < (length(2)-2)
    G = rmedge(G,key{i}, key{i+2});
    G = rmedge(G,key{i+1}, key{i+3});
    i = i + 4;
end
visEd = G.Edges;  % visible edges
sizeEd = size(G.Edges);

%Path generation
%Store gains in vector
K=[kg kh ke];
%Measure path computation  
 
tic                                          
%Generate path 
[path,n_points]=theta_star_3D(K,E3d_safe,x0,y0,z0,xend,yend,zend,sizeE);
T_path=toc;
%Path waypoints partial distance
%Initialize
path_distance=zeros(n_points,1);
for i=2:n_points 
	path_distance(i)=path_distance(i-1)+sqrt((path(i,2)-path(i-1,2))^2+(path(i,1)-path(i-1,1))^2+(path(i,3)-path(i-1,3))^2);      
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Plot
%Map grid
x_grid=1:d_grid:sizeE(2);
y_grid=1:d_grid:sizeE(1);
%Path on safe map
%figure(1)
%surf(x_grid(2:end-1),y_grid(2:end-1),E_safe(2:end-1,2:end-1))
%hold on
%plot3(x0,y0,z0,'gs')
%plot3(xend,yend,zend,'rd')
%plot3(path(:,2),path(:,1),path(:,3),'yx')
%plot3(path(:,2),path(:,1),path(:,3),'w')
%axis tight
%axis equal
%view(45*7,20);
%colorbar

%Path on standard map
figure(2)
disp('Size Ed:'); disp(sizeEd(1));
disp('=======================');
disp('Size E:'); disp(size(E));
disp('=======================');
disp('Distance:'); 
disp(path_distance);
disp('=======================');
disp('computation time:'); toc;
disp('=======================');


for i=1:sizeEd(1)
   x = visEd(i,1);
   xx = x{1,1};
   p1 = Map(xx{1,1});
   p2 = Map(xx{1,2});
   
   xpoints = [p1(1,1), p2(1,1)];
   ypoints = [p1(1,2), p2(1,2)];
   %disp('Point :');
   %disp(p1);
   %disp(xpoints);
   %disp('Point Y:');
   %disp(ypoints);
   %disp('----------------');
   zpoints = [0 0];
   for g=1:sizeE(1) %y
        for t=1:sizeE(2) %x
            if t ==   p1(1,1) && g == p1(1,2)
                if E(g,t) == 0 || E(t,g) == 0
                   if t == x0 && g == y0
                      zpoints(1) = z0; 
                   elseif t == xend && g == yend
                      zpoints(1) = zend;     
                   end             
                else
                   zpoints(1) = E(g,t);
                end
            end
        end
   end
   for g=1:sizeE(1) %y
       for t=1:sizeE(2) %x
           if t ==   p2(1,1) && g == p2(1,2)
              if E(g,t) == 0 || E(t,g) == 0
                 if t == x0 && g == y0
                    zpoints(2) = z0; 
                 elseif t == xend && g == yend
                    zpoints(2) = zend;     
                 end             
              else
                 zpoints(2) = E(g,t);
              end
          end
      end
   end
   hold on
%      plot3(xpoints, ypoints, zpoints,'linewidth',0.1,'Color',[0.5 0.5 1]);
   %plot3(xpoints, ypoints, zpoints,'-y');
end


surf(x_grid(2:end-1),y_grid(2:end-1),E(2:end-1,2:end-1))
shading interp
%colormap winter
hold on
plot3(x0,y0,z0,'gs')
plot3(xend,yend,zend,'rd')
plot3(path(:,2),path(:,1),path(:,3),'wx')
% plot3(path(:,2),path(:,1),path(:,3),'linewidth',2,'Color',[1 0 0])
% plot3(a,b,c,'linewidth',4,'Color',[1 0 0]);
axis tight
axis equal
view(45*7,20);
colorbar


%------------------------------------------
%UAV motion
%------------------------------------------
[a , ~] = size(path(:,1));
%figure(4), clf
hold on
         title('UAV')
         xlabel('East')
%         ylabel('North')
%         zlabel('-Down')
%         view(45*7,90)  % set the view angle for figure
%         axis([-S,S,-S,S,-.1,  S]);
%         grid on

curvature = 0.1;

% [a b] = size(X);
% 
% hold on
% plot3(X,Y,Z,'r')
% hold off

% initialize the UAV controllers 
thrust_c_params = [1.5; 2; 2.5];
kp = 2.5; ki = 2.75; kd = 1.075;
phy_c_params = [kp; ki;  kd];
theta_c_params = [kp; ki; kd];
psy_c_params = [kp; ki; kd];

%fixedwings_ctrl = construct_controllers(thrust_c_params, phy_c_params, theta_c_params, psy_c_params);
fixedwings_ctrl = controller(2, 3, 1);

%param
para.g=9.8;
para.m=1.2;
para.Iy=0.5;
para.Ix=0.5;
para.Iz=0.5;
para.b=10^-3;
para.l=0.75;
para.d=10^-4;
para.Jr=0.1;
para.k1=0.2;
para.k2=0.2;
para.k3=0.2;
para.k4=0.5;
para.k5=0.1;
para.k6=0.1;
para.omegaMax=330;

xl=[0;0;0];
vl=[0;0;0];
n = 1;
s=zeros(12,n);

%s(1:3,:)=unifrnd(-0,0,[3,n]);
%disp(s(1:3,:));

s(7,:) = 0;
s(8,:) = 0;
s(9,:) = 0;
dt=0.001;

fx = zeros(1,a);
fy = zeros(1,a);
fz = zeros(1,a);
for ij =1:a
    fx(ij) = path(ij,2); %x
    fy(ij) = path(ij,1); %y
    fz(ij) = path(ij,3); %z
end    

[X,Y,Z] = pathsmoothbezier(fx,fy,fz,curvature);
hold on

% plot3(path(:,2),path(:,1),path(:,3),'wx')
% plot3(path(:,2),path(:,1),path(:,3),'linewidth',2,'Color',[1 0 0])

plot3(X,Y,Z,'linewidth',5,'Color',[1 1 0]);
theta_node= [fx;fy;fz];

% disp(theta_node);
% plot3(X,Y,Z,'linewidth',2,'Color',[1 0 1]);

hold off

%------------------------------------------
%UAV motion
%------------------------------------------
[a, b] = size(X(1,:));

n_max = 100;
%figure(4), clf
hold on
phi      = 0;           % roll angle         
theta    = 0;           % pitch angle     
%psi      = 3.14;       % yaw angle  
x1 = X(1,1);    %path(1,1);
y1 = Y(1,1);    %path(1,2);
x2 = X(1,2);    %path(2,1);
y2 = Y(1,2);    %path(2,2);
psi = (atan2d(x1*y2-y1*x2,x1*x2+y1*y2))*(3.14/180); 

pn       = X(1,1);  %path(1,2);         % y position     
pe       = Y(1,1);  %path(1,1);         % x position
pd       = -Z(1,1);  %-path(1,3);       % z position

[Vertices,Faces,facecolors] = defineAircraftBody;                
aircraft_handle = drawBody(Vertices,Faces,facecolors,...
                                   pn,pe,pd,phi,theta,psi,...
                                   [], 'normal');
                               hold off

%[b a] = (size(X));
%DSFSDFS



%iki opo
 for i=2:b
%     xf = path(i-1,2);
%     yf = path(i-1,1);
%     zf = path(i-1,3);
%     xe = path(i,2);
%     ye = path(i,1);
%     ze = path(i,3);

    xf = X(1,i-1);
    yf = Y(1,i-1);
    zf = Z(1,i-1);
    xe = X(1,i);
    ye = Y(1,i);
    ze = Z(1,i);
    

    Pmatrix = [(linspace(xf,xe,n_max));
        (linspace(yf,ye,n_max));
        (linspace(-zf,-ze,n_max))];  
    
    
    for c = 2:n_max
        Z1 = Pmatrix(3,c-1);
        Z2 = Pmatrix(3,c);
        X1 = Pmatrix(1,c-1);
        Y1 = Pmatrix(2,c-1);
        X2 = Pmatrix(1,c);
        Y2 = Pmatrix(2,c);
       
        
        s(1:3,:)=[X1;Y1;Z1];
        s(4:6,:)=[X2;Y2;Z2];
        %s(7,:)=unifrnd(-0.0*pi,0.0*pi,[1,n]);
        %s(8,:)=unifrnd(-0.0*pi,0.0*pi,[1,n]);
        %s(9,:)=unifrnd(-0.0*pi,0.0*pi,[1,n]);
        
        x=s(1);y=s(2);z=s(3);
        
       
      
        omega=uav_controller(s,xl,vl,0,para,1,10);
        s=uav_kinematics(s,omega,para,dt);
        
        phi=s(7);theta=s(8);
        %psi=s(9);%-(6.28/4);
        vphi=s(10);vtheta=s(11);vpsi=s(12);
        
        vx=s(4);vy=s(5);vz=s(6);
        x1 = X1;y1 = Y1;
        x2 = X2;y2 = Y2;
        [ang,rad] = getAngle([vx,vy],[x,y]);                
        psi = rad -(6.28/4);
        s(9) = psi;
    
        
        Pmatrix(1,c) = vx;
        Pmatrix(2,c) = vy;    
        Pmatrix(3,c) = vz;
        
    pn       =  s(2);       % y position     
    pe       =  s(1);       % Pmatrix(1,c-1);       % x position
    pd       =  s(3);       % Pmatrix(3,c-1);       % z position
    %disp(vpsi);
    
    
    drawBody(Vertices,Faces,facecolors,...
                     pn,pe,pd,-phi,theta,psi,...
                     aircraft_handle);
    pause(0.001);
    %hold off
    end
    
end




%Path height profile
%figure(3)
% disp(path_distance,path(:,3))
%grid on
%xlabel('Path distance')
%ylabel('Path height')
