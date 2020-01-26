function JA=Hermite(pos,vel,k,u)

h=[1 0 0 0;0 0 1 0;-3 3 -2 -1;2 -2 1 1];

JA=[];
l=1;
j=0.0;
o=1;
for i=1:k-1
    
    Pi=pos(i,:);
    Pii=pos(i+1,:);
    Di=vel(i,:);
    Dii=vel(i+1,:);
    comp=[Pi;Pii;Di;Dii];
    
    while j<1.0
        p=[1 j j^2 j^3];
        JA(o,:)=(p*h*comp);
        disp(o);
        o=o+1;
        j=j+u;
        
    end
    j=j-1.0000001;
    
end



end

