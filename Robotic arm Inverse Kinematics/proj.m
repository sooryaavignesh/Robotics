clc 
clear all
close all
load arm
load trajectory

%Obtaining values from the given set of files and storing it into variables.
a=arm;
b=trajectory;
n=a(1,1);
m=b(1,1);
lambda=a(1,2);
l=a(2:n+1,1);
theta=a(2:n+1,2);



I=eye(2,2);
angles=zeros(2,n);
for i=1:m  
     
     fprintf('Evaluating for point:%i\n', i);
     j=0;
     loop=0;
     dev=0.00001;
     while j==0
         
        loop=loop+1;
        xd=b(i+1,:);                      %obtaining the X desired point from the trajectory file.
        
        %finding jacobian and forward kinematics                      
        [ja,xa]=jacobian(n,l,theta);    
        
        delx= (xd-xa).';
        
        %Perfoming Damped least squares to get deltheta   
        jt=ja.'; 
        in=inv((ja*jt)+lambda^2*I);
        deltheta=jt*in*delx;
        
        g1=norm(deltheta);
      
        theta=theta+deltheta*0.094;        %adding deltheta to existing theta matrix       
                                       
                                           
        g2=norm(delx);
        
        %loop exit conditions   
        if i==1
            if g2 < dev                    % checks if the norm of Delx matrix is less than the deviation.
               j=1;
            end
        else 
            if g1 < dev                    % checks if the norm of Deltheta matrix is less than the deviation.
               j=1;
            end
        end
       
        if loop>20                         %Conditional statement that increments the deviation variable after 20 iterations inside the loop. 
          loop=0;
          dev=dev*1.01;
        end     
     end
     angles(i,:)=theta;                    %appending the theta value to the angles matrix.                                           
    
    
end


disp('Successfull arm movement');

dlmwrite('angles',angles,'delimiter',' ');










            