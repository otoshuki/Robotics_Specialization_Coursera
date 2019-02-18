data = load('sample_data');
Samples = data.Samples;
D = 2;
[mu, sigma] = MLE(Samples(:,1:D));
mu
sigma

% save('params', 'mu','sigma');
h_points = linspace(0, 1, 120);
s_points = linspace(0, 1, 160);
distr = zeros(120,160);

for i=1:120
    for j=1:160
        prob = gaussian([s_points(j),h_points(i)],mu,sigma);
        distr(i,j) = prob;
    end
end
[X,Y] = meshgrid(s_points,h_points);
figure, 
hold on;
scatter(Samples(:,1),Samples(:,2),'.');
plot(mu(1),mu(2),'r*');
contour(X,Y,distr,16);
title('Pixel Color Distribubtion');
xlabel('Hue');
ylabel('Saturation');
% zlabel('Value');
imagepath = './train';
I = imread(sprintf('%s/%03d.png',imagepath,1));
HSV = rgb2hsv(I);
H = HSV(:,:,1);
S = HSV(:,:,2);
V = HSV(:,:,3);
segI = zeros(size(HSV,1),size(HSV,2));
for i=1:size(HSV,1)
    for j=1:size(HSV,2)
        input = [V(i,j),S(i,j)];
        prob = gaussian(input, mu,sigma);
        if prob > 0.5
            segI(i,j) = 1;
        end
    end
end
segI;
