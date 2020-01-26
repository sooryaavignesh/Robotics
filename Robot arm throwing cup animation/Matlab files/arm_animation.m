clear all
close all
a=dlmread('robot.key')
k=a(1,1)
t=a(1,2)
a(1,:)=[]
%step size
u=(k-1)/(t-1)

%separating the Position and Velocity values into 2 matrices
j=1;
for i=1:2:(2*k)
    pos(j,:)=a(i,:);
    vel(j,:)=a(i+1,:);
    j=j+1;
end

%positional interpolation
joint_angles=Hermite(pos,vel,k,u)
 
dlmwrite('robot.ang',t);
dlmwrite('robot.ang',joint_angles,'-append','delimiter',' ');


    