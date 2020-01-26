function [ja,xa] = jacobian(n, l, theta)

thetasum=zeros(1,n);
for i=1:n
    for j=1:i
        thetasum(i)=thetasum(i)+theta(j);    
        thetasumcos(i)=cos(thetasum(i));
        thetasumsin(i)=sin(thetasum(i));   
    end
end


xa=actualposition(thetasumsin, thetasumcos, l);     %forward kinematics


ja=zeros(2,n);
for j=1:n
        ja(1,j)=(-1)*thetasumsin(j:n)*l(j:n);
        ja(2,j)=thetasumcos(j:n)*l(j:n);
end
end

