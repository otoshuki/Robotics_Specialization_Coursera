distr = zeros(160);
mu = [80,80];
sigma = [1,0;0,1];
for i=1:160
    for j=1:160
        prob = gaussian([i,j],mu, sigma);
        distr(i,j) = prob;
    end
end
X = [1:1:160];
Y = [1:1:160];
figure, 
hold on;
plot(mu(1),mu(2),'r*');
contour(X,Y,distr,16);
title('Try');
