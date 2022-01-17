
lat = missile.lat;
lon = missile.lon;
h = missile.h;
roll = missile.roll;
pitch = missile.pitch;
yaw = missile.yaw;

lat0 = target.lat;
lon0 = target.lon;
h0 = target.h;
roll0 = target.roll;
pitch0 = target.pitch;
yaw0 = target.yaw;

azimuth = seeker.az;
elevator = seeker.el;
fov = seeker.fov;
f = 320/tan(deg2rad(fov/2));

[u0,w0]  = imageModel(lat,lon,h,lat0,lon0,h0,roll,pitch,yaw,azimuth,elevator,fov);
[ x0, y0, z0 ] = Geoditic2ECEF( lat0, lon0, h0 );
[ x, y, z ] = Geoditic2ECEF( lat, lon, h );
distance0 = norm([x-x0,y-y0,z-z0]);

delta_az = asin(u0/norm([u0 f]));
delta_el = asin(w0/norm([u0 w0 f]));

L = (max(Data.vertices(:,1))-min(Data.vertices(:,1)))/2;
if round(L) < L
    L = round(L) + 1;
else
    L = round(L);
end

h = (max(Data.vertices(:,2))-min(Data.vertices(:,2)))/2;
if round(h) < h
    h = round(h) + 1;
else
    h = round(h);
end

delta_u0 = atan(L/distance0)*f;
delta_w0 = atan(h/distance0)*f;
ImageMatrix = zeros(480,640);
% tic;
for j = 1:480
    for i = 1:640
        tic;
        sum_radiance = [];
        if (abs(delta_az) < deg2rad(fov/2)+atan(L/distance0)) && (abs(delta_el) < deg2rad(3/4*fov/2)+atan(h/distance0))
            if (u0-delta_u0)< i-320 && i-320 < (u0+delta_u0) && (w0-delta_w0) < j-240 && j-240 <(w0+delta_w0)
                parfor n = -4:1:5
                    for m = -4:1:5
                        [distance,beta] = surfaceCross (Data,missile,seeker,i+(m-0.5)/10-320,j+(n-0.5)/10-240);
                        sum_radiance = [sum_radiance ,cos(beta)^5/100];
                    end
                end
            end
        end
%         fprintf('-------------------------------\n')
%         fprintf('u = %d\n',j);
%         fprintf('w = %d\n',i)
        ImageMatrix(j,i) = sum(sum_radiance);
        toc;
    end
end
% toc;
figure(2);
imshow(ImageMatrix);

