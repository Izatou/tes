Skip to content
Search or jump to…

Pull requests
Issues
Marketplace
Explore
 
@Izatou 
Izatou
/
tes
1
00
Code
Issues
Pull requests
Actions
Projects
Wiki
Security
Insights
Settings
tes
/
mainProg.m
 

Tabs

8

No wrap
1
%Main
2
clc
3
clear
4
​
5
​
6
%path parameters
7
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
8
​
9
%size [y,x,z] and resolution
10
sizeE=[150 150 30];
11
d_grid=1;
12
​
13
%Starting point
14
x0=5;
15
y0=5;
16
z0=10;
17
​
18
%Arrival point
19
xend=125;
20
yend=140;
21
zend=10;
22
​
23
​
24
%% Defining environment variables
25
start = [x0,y0];                                % start position
26
goal = [xend, yend];                            % goal position
27
%n = 2;                                         % no. of obstacles
28
​
29
​
30
%Number of points with low elevation around start and end point area 
31
n_low=3;
32
​
33
%Theta * cost weights
34
kg=1;
35
kh=1.25;
36
ke=sqrt((xend-x0)^2+(yend-y0)^2+(zend-z0)^2);
37
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
38
​
39
%Map definition
40
%Average flight altitude
41
h=max(z0,zend);
42
​
43
%Points coordinates in [y,x,z] format
44
P0=[y0 x0 z0];
45
Pend=[yend xend zend];
@Izatou
Commit changes
Commit summary
Update mainProg.m
Optional extended description
Add an optional extended description…

farisce.009@gmail.com
Choose which email address to associate with this commit

 Commit directly to the master branch.
 Create a new branch for this commit and start a pull request. Learn more about pull requests.
 
© 2020 GitHub, Inc.
Terms
Privacy
Security
Status
Help
Contact GitHub
Pricing
API
Training
Blog
About
