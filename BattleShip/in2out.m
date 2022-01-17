function in = in2out (Data,u,w)
sum = 0;
for i = 1:length(Data)
    if i == length(Data)
        vecto1 = -[u,w] + Data(i,:);
        vecto2 = -[u,w] + Data(1,:);
        
    else
        vecto1 = -[u,w] + Data(i,:);
        vecto2 = -[u,w] + Data(i+1,:);
    end
    cos_phi = vecto1*vecto2'/norm(vecto1)/norm(vecto2);
    if abs(cos_phi) > 1
        cos_phi = cos_phi/abs(cos_phi);
    end
    sum = sum + acos(cos_phi);
end
fprintf('sum = %d\n',sum)
if sum == 2*pi
    in = 1;
else
    in = 0;
end
end