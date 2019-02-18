function [gaussian] = gaussian(x, mu, sigma)
% Computes Gaussian
x = double(x);
D = size(x,2);
constant = nthroot(2*pi, D/2)*sqrt(det(sigma));
gaussian = exp(-(x-mu)*pinv(sigma)*(x-mu)'/2)/constant;
end

