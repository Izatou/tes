 function [Map,Pend,P0,h,keys,values,key,nodes,len] = obstacles(xend,yend,zend,y0,x0,z0,start,goal)
% 
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
%titik = [20,10,5];
%values = {};
%values = [values start];


   
Map = containers.Map(keys, values);
len = Map.values;
 end
