distr = zeros(160);
mu = [50,80];
sigma = [1,0.2;0.4,0.5];
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
plot(mu(2),mu(1),'r*');
contour(X,Y,distr,16);
title('Try');
