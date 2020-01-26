% Reading the object.key file
clear all
close all
clear workspace

a=dlmread('object.key');
k=a(1,1);
t=a(1,2);
a(1,:)=[];
rotation_matrix=a(:,1:3);
position=zeros(k,3);
l=1;

for i=1:3:3*k
   position(l,:)=transpose(a(i:i+2,4));
   l=l+1;
end

%step size
u=(k-1)/(t-1);



H = [-.5,1.5,-1.5,.5; 1,-2.5,2,-.5; -.5,0,.5,0; 0,1,0,0];

f = 0.0;                        
o=1;
%performing catmull-rom for positional interpolation
for i=1:k-1
    
    if i == 1
        p0 = position(i,:);
        p1 = position(i,:);
        p2 = position(i+1,:);
        p3 = position(i+2,:);
    
    elseif (i+1)==k
        p0  = position(i-1,:);
        p1 = position(i,:);
        p2 = p1 + (p1 - p0);
        p3 = p2 + (p2 - p1);
    
    elseif (i+2) ==k
        p0  = position(i-1,:);
        p1 = position(i,:);
        p2 = position(i+1,:);
        p3 = p2 + (p2 - p1);
    
    else 
        p0 = position(i-1,:);
        p1 = position(i,:);
        p2 = position(i+1,:);
        p3 = position(i+2,:);
     
    end 
    
    P=[p0; p1; p2; p3];
    
    while f < 1.0
       U=[f^3,f^2,f,1];
       pos(o,:) = U*H*P;
       o=o+1;
       f=f+u ;
    end
  f=f-1.0000001;
end
 
%doing rotational interpolation
q=[];
%converting rotation matrix to quaternions
for i = 1:3:3*k
    q = [q; R_to_Q( rotation_matrix(i:i+2,:) )];
end

%slerp
interpolation = Q_interpolation(q,k,t);

%converting quaternions back to rotation matrix
rot=[];
for i = 1:t
    rot=[rot; Q_to_R(interpolation(i,:))];
end

out=[];

o=1

for i = 1:t
       for j = 1:3
           glass_traj(o,:)=[rot((3*(i-1))+j,:),pos(i,j)];
           o=o+1;     
       end
end

dlmwrite('object.traj',t);
dlmwrite('object.traj',glass_traj,'-append','delimiter',' '); 



