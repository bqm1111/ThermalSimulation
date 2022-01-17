function [Data] = convert2image (Data,missile,target,seeker)
lat = missile.lat;
lon = missile.lon;
h = missile.h;
roll = missile.roll;
pitch = missile.pitch;
yaw = missile.yaw;

data = Data.vertices;

lat0 = target.lat;
lon0 = target.lon;
h0 = target.h;
roll0 = target.roll;
pitch0 = target.pitch;
yaw0 = target.yaw;

azimuth = seeker.az;
elevator = seeker.el;
fov = seeker.fov;

[ x, y, z ] = Geoditic2ECEF( lat0, lon0, h0 );

Ri2b0 =[cos(pitch0)*cos(yaw0) sin(roll0)*sin(pitch0)*cos(yaw0)-cos(roll0)*sin(yaw0) cos(roll0)*sin(pitch0)*cos(yaw0)+sin(roll0)*sin(yaw0);
        cos(pitch0)*sin(yaw0) sin(roll0)*sin(pitch0)*sin(yaw0)+cos(roll0)*cos(yaw0) cos(roll0)*sin(pitch0)*sin(yaw0)-sin(roll0)*cos(yaw0);
       -sin(pitch0)           sin(roll0)*cos(pitch0)                             cos(roll0)*cos(pitch0)];

lambda = lat0 / 180.0 * pi;
phi = lon0 / 180.0 * pi;
sin_lambda = sin(lambda);
cos_lambda = cos(lambda);
sin_phi = sin(phi);
cos_phi = cos(phi);

M = [-cos_phi * sin_lambda -sin_lambda * sin_phi cos_lambda;
     -sin_phi              cos_phi               0;
     -cos_lambda * cos_phi -cos_lambda * sin_phi -sin_lambda];

for i = 1:length(data)
    NED = M'*Ri2b0 *[data(i,1);data(i,3);-data(i,2)];
    
    [ lat01, lon01, h01 ] = ECEF2Geoditic( x+NED(1,1), y+NED(2,1), z+NED(3,1) );
    Data.GPS(i,:) = [lat01,lon01,h01];
    [u,w]  = imageModel(lat,lon,h,lat01,lon01,h01,roll,pitch,yaw,azimuth,elevator,fov);
    Data.imagePosition(i,:) = [u,w];
end
end