function [distance,beta] = distanceToface (Data,faces,missile,seeker,u,w)

lat = missile.lat;
lon = missile.lon;
h = missile.h;
roll = missile.roll;
pitch = missile.pitch;
yaw = missile.yaw;

azimuth = seeker.az;
elevator = seeker.el;
fov = seeker.fov;
f = 320/tan(deg2rad(fov/2));

Ldonic = [u;w;f]/norm([u;w;f]);
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
NED = M'*Ri2b*Ldoni*Ldonic;

[ x, y, z ] = Geoditic2ECEF( lat, lon, h );

lat1 = Data.GPS(faces(1,1),1);
lon1 = Data.GPS(faces(1,1),2);
h1 = Data.GPS(faces(1,1),3);
[ x1, y1, z1 ] = Geoditic2ECEF( lat1, lon1, h1 );

lat2 = Data.GPS(faces(1,2),1);
lon2 = Data.GPS(faces(1,2),2);
h2 = Data.GPS(faces(1,2),3);

lat3 = Data.GPS(faces(1,3),1);
lon3 = Data.GPS(faces(1,3),2);
h3 = Data.GPS(faces(1,3),3);

vector1  = vectorCalculator (lat1,lon1,h1,lat2,lon2,h2);
vector2  = vectorCalculator (lat2,lon2,h2,lat3,lon3,h3);
n = cross(vector1,vector2);
d = - [x1,y1,z1]*n;
if (d+[x,y,z]*n) == 0
    [distance,beta] = averageDistance (Data,faces,missile,NED);
else
    distance = norm(NED)/abs(NED'*n)*abs((d+[x,y,z]*n));
    beta = acos(abs(NED'*n)/norm(NED)/norm(n));
end
end

function [distance,beta] = averageDistance (Data,faces,missile,NED)
lat = missile.lat;
lon = missile.lon;
h = missile.h;
[ x, y, z ] = Geoditic2ECEF( lat, lon, h );
count = 0;
range = zeros(0,0);
for i = 2:length(faces')
    if isnan(faces(1,i))
        vector1 = Data.imagePosition(faces(1,i-1),:)-[u,w];
        vector2 = Data.imagePosition(faces(1,1),:)-[u,w];
        if norm(vector1)==0 || norm(vector2)==0
            if norm(vector1)==0
                count = count + 1; 
                [ x1, y1, z1 ] = Geoditic2ECEF( Data.GPS(faces(1,i-1),1),Data.GPS(faces(1,i-1),2),Data.GPS(faces(1,i-1),3) );
                range(count) = norm([x1-x,y1-y,z1-z]);
            end
            if norm(vector2)==0
                count = count + 1; 
                [ x1, y1, z1 ] = Geoditic2ECEF( Data.GPS(faces(1,1),1),Data.GPS(faces(1,1),2),Data.GPS(faces(1,1),3) );
                range(count) = norm([x1-x,y1-y,z1-z]);
            end
        else
            if (vector1*vector2'/norm(vector1)/norm(vector2)) < 0
                [ x1, y1, z1 ] = Geoditic2ECEF( Data.GPS(faces(1,i-1),1),Data.GPS(faces(1,i-1),2),Data.GPS(faces(1,i-1),3) );
                [ x2, y2, z2 ] = Geoditic2ECEF( Data.GPS(faces(1,1),1),Data.GPS(faces(1,1),2),Data.GPS(faces(1,1),3) );
                NED1 = [x1-x2;y1-y2;z1-z2]/norm([x1-x2;y1-y2;z1-z2]);
                A = [NED,-NED1];
                B = [x1;y1;z1]-[x;y;z];
                t = pinv(A'*A)*A'*B;
                count = count + 1; 
                range(count) = norm(NED*t(1,1));
            end
        end
        break;
    else
        vector1 = Data.imagePosition(faces(1,i-1),:)-[u,w];
        vector2 = Data.imagePosition(faces(1,i),:)-[u,w];
        if norm(vector1)==0 || norm(vector2)==0
            if norm(vector1)==0
                count = count + 1;
                
                [ x1, y1, z1 ] = Geoditic2ECEF( Data.GPS(faces(1,i-1),1),Data.GPS(faces(1,i-1),2),Data.GPS(faces(1,i-1),3) );
                range(count) = norm([x1-x,y1-y,z1-z]);
            end
            if norm(vector2)==0
                count = count + 1;
                
                [ x1, y1, z1 ] = Geoditic2ECEF( Data.GPS(faces(1,i),1),Data.GPS(faces(1,i),2),Data.GPS(faces(1,i),3) );
                range(count) = norm([x1-x,y1-y,z1-z]);
            end
        else
            if (vector1*vector2'/norm(vector1)/norm(vector2)) < 0
                [ x1, y1, z1 ] = Geoditic2ECEF( Data.GPS(faces(1,i-1),1),Data.GPS(faces(1,i-1),2),Data.GPS(faces(1,i-1),3) );
                [ x2, y2, z2 ] = Geoditic2ECEF( Data.GPS(faces(1,i),1),Data.GPS(faces(1,i),2),Data.GPS(faces(1,i),3) );
                NED1 = [x1-x2;y1-y2;z1-z2]/norm([x1-x2;y1-y2;z1-z2]);
                A = [NED,-NED1];
                B = [x1;y1;z1]-[x;y;z];
                t = pinv(A'*A)*A'*B;
                count = count + 1; 
                range(count) = norm(NED*t(1,1));
            end
        end
    end
end
maximum = max(range);
[minimum,idx] = min(range);
distance = minimum/2;
range(idx) = maximum;
minimum = min(range);
distance = distance + minimum/2;
beta = pi/2;
end

function NED = vectorCalculator (lat,lon,h,lat1,lon1,h1)

[ x, y, z ] = Geoditic2ECEF( lat, lon, h );
[ x1, y1, z1 ] = Geoditic2ECEF( lat1, lon1, h1 );

NED = [x1-x;y1-y;z1-z]/norm([x1-x;y1-y;z1-z]);

end