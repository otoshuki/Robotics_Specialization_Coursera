function [mu,sigma] = MLE(samples)
% Computes the Gaussian model parameters mu and sigma
% INPUT 
% samples       : N dimensions - sample data
% OUTPUT
% mu, sigma     : Model parameters
samples = double(samples);
[L,D] = size(samples);

% Params
mu = mean(samples);
size(mu);
sigma = zeros(D);
for i = 1:L
    temp = (samples(i,:) - mu);
    sigma = sigma + temp'*temp;
end
sigma = sigma/L;
size(sigma);
%Plot
end
