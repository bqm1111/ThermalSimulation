function [u,w]  = imageModel(lat,lon,h,lat0,lon0,h0,roll,pitch,yaw,azimuth,elevator,fov)

[ x0, y0, z0 ] = Geoditic2ECEF( lat0, lon0, h0 );
[ x, y, z ] = Geoditic2ECEF( lat, lon, h );

distance = norm([x-x0,y-y0,z-z0]);
NED = [x-x0,y-y0,z-z0]'/distance;

Ldoni = [-sin(azimuth) sin(elevator)*cos(azimuth) cos(elevator)*cos(azimuth);
          cos(azimuth) sin(elevator)*sin(azimuth) cos(elevator)*sin(azimuth);
            0          cos(elevator)              -sin(elevator)];
Ri2b = [cos(pitch)*cos(yaw) sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw) cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw);
        cos(pitch)*sin(yaw) sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw) cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw);
       -sin(pitch)          sin(roll)*cos(pitch)                             cos(roll)*cos(pitch)];

lambda = lat / 180.0 * pi;
phi = lon / 180.0 * pi;

sin_lambda = sin(lambda);
cos_lambda = cos(lambda);
sin_phi = sin(phi);
cos_phi = cos(phi);

M = [-cos_phi * sin_lambda -sin_lambda * sin_phi cos_lambda;
     -sin_phi              cos_phi               0;
     -cos_lambda * cos_phi -cos_lambda * sin_phi -sin_lambda];

Ldonic = Ldoni' * Ri2b' * M * NED;
if Ldonic(3,1)< 0
    Ldonic = -Ldonic;
end
f = 320/tan(deg2rad(fov/2));
u = Ldonic(1,1)/Ldonic(3,1)*f;
w = Ldonic(2,1)/Ldonic(3,1)*f;
end