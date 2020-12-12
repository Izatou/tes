%3D map generator
function [high,E,E_safe,E3d,E3d_safe]=safezone(sizeE,d_grid,h,P0,Pend,n_low,values)


%size
y_size=sizeE(1);
x_size=sizeE(2);
z_size=sizeE(3);

%Vertical vector
z_grid=1:d_grid:z_size;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Random grid generation, discrete height in big blocks

%random points 0-1 altitude
mean_E=0;
sigma=1;
k_sigma= 2.5; %obstacles density
E=random('Normal',mean_E,sigma,y_size,x_size);
sigma_obstacle=k_sigma*sigma;
E=double(E>sigma_obstacle);
%random altitude ->blocks around points 
%Minimum obstacle altitude
hh_min=2;
%Initialize temporary matrix for evaluation

[a b] = size(values);

high = { 10, 15, 10, 15, 20,... 
         15, 12, 15, 10, 15,...
         10, 5, 15, 20, 10,...
         15,30,5,10,15,5,...
         10,15,20,15,10,...
         20,5,10,5,9,...
         8,7,6,5,4,...
         3,2,7,2,9,...
         8,7,6,5,4,...
         3,2,9,8,7,...
         6 };


for i=1:x_size
    for j=1:y_size
        for t=2:b-1
            l = values{t}(1); k = values{t}(2);
            if l == i && k == j
               %E(k,l)=hh;   
            else
               %E(j,i)= hh_min; 
                
            end
        end
        %
    end
end

val = {};
for i=2:b-1
    %hh=round(random('Normal',0.75*h,0.5*h));
    %l = values{i}(2); k = values{i}(1);
    val = [val values{i}];
    %E(l,k)=hh;      
end
[a b] = size(val);
b1 = b - 1;
%point = 1;
EE=E;
for i=1:(b/4)    
    point = i * 4;
    %disp(i);
    xq = val{point-3};
    yq = val{point-2};
    xv = val{point-1};
    yv = val{point};
    hh=round(random('Normal',0.75*h,0.5*h));
    if hh<hh_min
       hh=hh_min;
    elseif hh>z_size
       hh=z_size;
    end
    hh = high{i}; 
    for k = xq(1): yv(1)
        for m = xq(2) : yq(2)
            E(m,k)=hh;    
            EE(m,k) = 1; 
        end
    end
    %disp(sprintf('%d,%d',xq(1),xq(2)));
    %disp(sprintf('%d,%d',yq(1),yq(2)));
    %disp(sprintf('%d,%d',xv(1),xv(2)));
    %disp(sprintf('%d,%d',yv(1),yv(2)));
    %disp('\n');
end


for i=1:x_size
    for j=1:y_size
        %Block boundaries with random dimension (max -2:+2)
        k=i-1-round(random('beta',0.5,0.5)):1:i+1+round(random('beta',0.5,0.5));
        l=j-1-round(random('beta',0.5,0.5)):1:j+1+round(random('beta',0.5,0.5));        
        
        %If block boundaries within the grid and if the node point value is high,
        if min(k)>0 && min(l)>0 && max(k)<=x_size && max(l)<=y_size && EE(j,i)==1            
            
            %Assign random value to block
            hh=round(random('Normal',0.75*h,0.5*h));
            
            %Give a minimum value to the altitude and limit maximum altitude
            if hh<hh_min
                hh=hh_min;
            elseif hh>z_size
                hh=z_size;
            end
            
            %E(l,k)=hh;           
            
        end
    end
end


%Assign low elevation to and around start and end points
E(P0(1)-n_low:P0(1)+n_low,P0(2)-n_low:P0(2)+n_low)=0;
E(Pend(1)-n_low:Pend(1)+n_low,Pend(2)-n_low:Pend(2)+n_low)=0;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Create 3D

%Initialize
E3d=zeros(y_size,x_size,z_size);

%Create 3D matrix occupancy index (0=free, 1=obstacle)
for i=1:z_size  
	E3d(:,:,i)=E>=z_grid(i);       
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Create safe zone near high elevation elements
%Initialize
E_safe=E;

for i=1:x_size
    for j=1:y_size
        
        %Check->neighbour nodes
        k=i-1:i+1;
        l=j-1:j+1;
        
        %Limit neighbours within the grid
        if min(k)<1
            k=i:i+1;
        elseif max(k)>x_size
            k=i-1:i;
        end
        if min(l)<1
            l=j:j+1;
        elseif max(l)>y_size
            l=j-1:j;
        end
        
        %Evaluation matrix
        E_eval=E(l,k);    
            
        %free point -> there is obstacle
        if E(j,i)==0 && max(E_eval(:))>20
            %Assign the maximum value of the neighbour nodes
            E_safe(j,i)=max(E_eval(:));
        end
        
        %If point is elevated add one safe step in altitude
        if E_safe(j,i)>0 && E_safe(j,i)<z_size-1
            E_safe(j,i)=E_safe(j,i)+1;            
        end        
        
    end
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Create 3D safe  matrix occupancy index (0=free, 0.5=safe, 1=obstacle)

%Initialize
E3d_safe=E3d;

for i=1:x_size
    for j=1:y_size
        for k=1:z_size
            %Check neighbour nodes
            l=i-1:i+1;
            m=j-1:j+1;
            n=k-1:k+1;
            
            %Limit neighbours within the grid
            if min(l)<1
                l=i:i+1;
            elseif max(l)>x_size
                l=i-1:i;
            end
            if min(m)<1
                m=j:j+1;
            elseif max(m)>y_size
                m=j-1:j;
            end
            if min(n)<1
                n=k:k+1;
            elseif max(n)>z_size
                n=k-1:k;
            end            
            
            %Evaluation matrix
            E_eval=E3d(m,l,n);            
            
            %If we are in a free point and nearby there is obstacle
            if E3d(j,i,k)==0 && max(E_eval(:))==1
                %Assign safe value of the neighbour nodes
               E3d_safe(j,i,k)=0.5;
            end

        end
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Create safe zone near borders

E([1 end],:)=z_size;
E(:,[1 end])=z_size;

E_safe([1 end],:)=z_size;
E_safe(:,[1 end])=z_size;

E3d([1 end],:,:)=1;
E3d(:,[1 end],:)=1;
E3d(:,:,[1 end])=1;

E3d_safe([1 end],:,:)=1;
E3d_safe(:,[1 end],:)=1;
E3d_safe(:,:,[1 end])=1;

