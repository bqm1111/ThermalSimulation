function [ lat, lon, h ] = ECEF2Geoditic( x, y, z )
% a = 6378137;
% b = 6356752.314245;
a = 6378137 ; b = 6356752;
f = (a - b) / a;

e_sq = f*(2-f);

eps = e_sq / (1.0 - e_sq);
p = sqrt(x * x + y * y);
q = atan2((z * a), (p * b));
sin_q = sin(q);
cos_q = cos(q);
sin_q_3 = sin_q * sin_q * sin_q;
cos_q_3 = cos_q * cos_q * cos_q;
phi = atan2((z + eps * b * sin_q_3),(p - e_sq * a * cos_q_3));
lambda = atan2(y, x);
v = a / sqrt(1.0 - e_sq * sin(phi) * sin(phi));
h = (p/cos(phi)) - v;
lat = phi / pi * 180.0;
lon = lambda / pi * 180.0;

end