function [ x, y, z ] = Geoditic2ECEF( lat0, lon0, h0 )
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
% a = 6378137;
% b = 6356752.314245;
a = 6378137 ; b = 6356752;
f = (a - b) / a;

e_sq = f*(2-f);

lambda = lat0 / 180.0 * pi;
phi = lon0 / 180.0 * pi;

sin_lambda = sin(lambda);
cos_lambda = cos(lambda);
sin_phi = sin(phi);
cos_phi = cos(phi);

N = a / sqrt(1 - e_sq * sin_lambda * sin_lambda);

x = (h0 + N) * cos_lambda * cos_phi;
y = (h0 + N) * cos_lambda * sin_phi;
z = (h0 + (1 - e_sq) * N) * sin_lambda;

end