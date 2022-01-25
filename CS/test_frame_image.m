clear all;clc;
load('ship_data_3.mat')
ocean = 4.5468;
    path  = 40.0641;
    sky   = 1.0555;
    solar = 27.7758;
    object = 8.5245;
    
    lat = missile.lat;
    lon = missile.lon;
    h = missile.h;
    roll = missile.roll;
    pitch = missile.pitch;
    yaw = missile.yaw;
    
    M_missile = Recef2inertia (lat,lon);
    Ri2b_missile = Rinertia2body (roll,pitch,yaw);
    
    lat0 = target.lat;
    lon0 = target.lon;
    h0 = target.h;
    roll0 = target.roll;
    pitch0 = target.pitch;
    yaw0 = target.yaw;
    
    M_target = Recef2inertia (lat0,lon0);
    Ri2b_target = Rinertia2body (roll0,pitch0,yaw0);
    
    
    azimuth = seeker.az;
    elevator = seeker.el;
    fov = deg2rad(seeker.fov);
%     f = 320/tan(fov/2);
    gain = 1e6*100/(rad2deg(fov))^2;
    
    Rb2g = Rbody2gimbal (azimuth,elevator);
    
    
    [u0,w0]  = imageModel(lat,lon,h,lat0,lon0,h0,M_missile,Ri2b_missile,Rb2g,fov);
    [ x0, y0, z0 ] = Geoditic2ECEF( lat0, lon0, h0 );
    [ x, y, z ] = Geoditic2ECEF( lat, lon, h );
    distance0 = norm([x-x0,y-y0,z-z0]);
    
    L = max(abs(max(Data.vertices(:,1))),abs(min(Data.vertices(:,1))));
    if round(L) < L
        L = round(L) + 1;
    else
        L = round(L);
    end
    
    height = max(abs(max(Data.vertices(:,2))),abs(min(Data.vertices(:,2))));
    if round(height) < height
        height = round(height) + 1;
    else
        height = round(height);
    end
    
    check.angle.azimuth = asin(u0/norm([u0 320/tan(fov/2)]));
    check.angle.elevator = asin(w0/norm([u0 w0 320/tan(fov/2)]));
    check.range.azimuth = fov/2+atan(L/distance0);
    check.range.elevator = 3/4*fov/2+atan(L/distance0);
    
%     check.pixels.u = atan(L/distance0)*320/tan(deg2rad(fov/2));
%     check.pixels.w = atan(height/distance0)*320/tan(deg2rad(fov/2));
    
    if abs(check.angle.azimuth) < check.range.azimuth && abs(check.angle.elevator) < check.range.elevator
        [Data] = convert_to_image (Data,lat,lon,h,lat0,lon0,h0,M_missile,M_target,Ri2b_missile,Rb2g,Ri2b_target,fov);
        check.pixels.u_left = floor(min(Data.imagePosition(:,1)))-1;
        check.pixels.u_right = floor(max(Data.imagePosition(:,1)))+1;
        check.pixels.w_left = floor(min(Data.imagePosition(:,2)))-1;
        check.pixels.w_right = floor(max(Data.imagePosition(:,2)))+1;
    end
    ImageMatrix = zeros(480,640);
%     fprintf('run of percent = %3.2f %\n',k*100/457720);
    for j = 1:480
        tic;
        for i = 1:640
            if abs(check.angle.azimuth) < check.range.azimuth && abs(check.angle.elevator) < check.range.elevator
                if (check.pixels.u_left)< i-320 && i-320 < (check.pixels.u_right) && (check.pixels.w_left) < j-240 && j-240 <(check.pixels.w_right)
                    radiance = radiance_object_region (Data,solar,sky,ocean,path,object,lat,lon,h,M_missile,Ri2b_missile,Rb2g,fov,check,i-320,j-240);
                else
                    radiance = pixel_radiance(solar,ocean,sky,path,lat,lon,h,M_missile,Ri2b_missile,Rb2g,0.9,i-320,j-240,fov);
                end
            else
                radiance = pixel_radiance(solar,ocean,sky,path,lat,lon,h,M_missile,Ri2b_missile,Rb2g,0.9,i-320,j-240,fov);
            end
            ImageMatrix(j,i) = radiance*gain;
        end
        toc;
    end

    imshow(0.5 * ImageMatrix);

%%
function radiance = radiance_object_region (Data,solar,sky,ocean,path,object,lat,lon,h,M,Ri2b,Rb2g,fov,check,i,j)
sum_radiance = [];
if (check.pixels.u_right-check.pixels.u_left)/2 < 50
    parfor idx = 0:15
        n = floor(idx/4) - 1;
        m = mod(idx,4) - 1;
        ifov_radiance = surface_cross (Data,solar,sky,ocean,path,object,lat,lon,h,M,Ri2b,Rb2g,fov,i+(m-0.5)/4,j+(n-0.5)/4);
        sum_radiance = [sum_radiance,ifov_radiance];
    end
elseif (check.pixels.u_right-check.pixels.u_left)/2 < 70
    parfor idx = 0:7
        n = floor(idx/4);
        m = mod(idx,4) - 1;
        ifov_radiance = surface_cross (Data,solar,sky,ocean,path,object,lat,lon,h,M,Ri2b,Rb2g,fov,i+(m-0.5)/4,j+(n-0.5)/2);
        sum_radiance = [sum_radiance,ifov_radiance];
    end
elseif (check.pixels.u_right-check.pixels.u_left)/2 < 100
    parfor idx = 0:3
        n = floor(idx/2);
        m = mod(idx,2);
        ifov_radiance = surface_cross (Data,solar,sky,ocean,path,object,lat,lon,h,M,Ri2b,Rb2g,fov,i+(m-0.5)/2,j+(n-0.5)/2);
        sum_radiance = [sum_radiance,ifov_radiance];
    end
elseif (check.pixels.u_right-check.pixels.u_left)/2 < 142
    parfor idx = 0:1
        m = mod(idx,2);
        ifov_radiance = surface_cross (Data,solar,sky,ocean,path,object,lat,lon,h,M,Ri2b,Rb2g,fov,i+(m-0.5)/2,j);
        sum_radiance = [sum_radiance,ifov_radiance];
    end
else
    parfor idx = 0
    ifov_radiance = surface_cross (Data,solar,sky,ocean,path,object,lat,lon,h,M,Ri2b,Rb2g,fov,i,j);
    sum_radiance = [sum_radiance,ifov_radiance];
    end
end
radiance = sum(sum_radiance)/length(sum_radiance);

end
%%
function out = createImageblur(imageMatrix,Ri2b,Rb2g,Ri2b0,Rb2g0,fov)
imageMatrix = imgaussfilt(imageMatrix,1);

f = 640/2/tan(fov/2);

NED = Ri2b * Rb2g * [0;0;1];
NED2 = Ri2b * Rb2g * [0;240;f]/sqrt(f^2+240^2);

NED = Rb2g0'*Ri2b0' * NED;
NED2 = Rb2g0'*Ri2b0' * NED2;


u = NED(1,1)*f/NED(3,1);
w = NED(2,1)*f/NED(3,1);

u2 = NED2(1,1)*f/NED2(3,1);
w2 = NED2(2,1)*f/NED2(3,1);

roll = acos((w2-w)/sqrt((w2-w)^2+(u2-u)^2));
l = min(400,sqrt(u^2+w^2));
phi = atan2(-w,u);
if phi >= pi
    phi = phi-2*pi;
end
h = fspecial('motion',l,rad2deg(phi));
out = imfilter(imageMatrix,h,'replicate','same');
end
%%
function [Data] = convert_to_image (Data,lat,lon,h,lat0,lon0,h0,M,M0,Ri2b,Rb2c,Ri2b0,fov)

[ x, y, z ] = Geoditic2ECEF( lat0, lon0, h0 );

for i = 1:length(Data.vertices)
    NED = M0'*Ri2b0 *[Data.vertices(i,1);Data.vertices(i,3);-Data.vertices(i,2)];
    
    [ lat01, lon01, h01 ] = ECEF2Geoditic( x+NED(1,1), y+NED(2,1), z+NED(3,1) );
    Data.GPS(i,:) = [lat01,lon01,h01];
    [u,w]  = imageModel(lat,lon,h,lat01,lon01,h01,M,Ri2b,Rb2c,fov);
    Data.imagePosition(i,:) = [u,w];
end
for j = 1:length(Data.faces)
    lat1 = Data.GPS(Data.faces(j,1),1);
    lon1 = Data.GPS(Data.faces(j,1),2);
    h1   = Data.GPS(Data.faces(j,1),3);
    [ x1, y1, z1 ] = Geoditic2ECEF( lat1, lon1, h1 );
    
    lat2 = Data.GPS(Data.faces(j,2),1);
    lon2 = Data.GPS(Data.faces(j,2),2);
    h2   = Data.GPS(Data.faces(j,2),3);
    
    lat3 = Data.GPS(Data.faces(j,3),1);
    lon3 = Data.GPS(Data.faces(j,3),2);
    h3   = Data.GPS(Data.faces(j,3),3);
    
    vector1  = vector_calculator (lat1,lon1,h1,lat2,lon2,h2);
    vector2  = vector_calculator (lat2,lon2,h2,lat3,lon3,h3);
    n = cross(vector1,vector2);
    Data.faces_perpendicular(j,:) = n';
    d = - [x1,y1,z1]*n ;
    Data.faces_gain_freedom(j,:) = d;  
end
end
%%
function [rdiance] = surface_cross (Data,solar,sky,ocean,path,object,lat,lon,h,M,Ri2b,Rb2g,fov,u,w)
ifov = fov/640;
count_faces = 0;
faces_data = zeros(0,0);
faces_idx = zeros(0,0);
distance0 = zeros(0,0);
beta0 = zeros(0,0);
for i = 1:length(Data.faces)
    vector = zeros(3,2);
    for j = 1:3
        vector(j,:) = Data.imagePosition(Data.faces(i,j),:);
    end
    if in_or_out (vector,u,w)
        count_faces = count_faces + 1;
        faces_data(count_faces,1:3) = Data.faces(i,1:3);
        faces_idx(count_faces) = i;
    end
end

if count_faces > 0
    for i = 1:count_faces
        [dis,bt] = distance_to_face (Data,faces_idx(i),lat,lon,h,M,Ri2b,Rb2g,fov,u,w);
        distance0(i) = dis;
        beta0(i) = bt;
    end
    [~,idx] = min(distance0);
    beta = beta0(idx);
    distance = distance0(idx);
    
    transmiss = Transmittance(distance);
    e = 0.1 *(1-(1- cos(beta))^5);
    solar_ifov = solar/pi * (1 - e) * transmiss * (ifov)^2;
    ocean_ifov = ocean/pi * 0.9 *(1 - e) * transmiss * (ifov)^2;
    sky_ifov = sky/pi * (1 - e) * transmiss * (ifov)^2;
    object_ifov = object/pi * e * transmiss * (ifov)^2;
    %     path_ifov = path/pi*(deg2rad(ifov/10))^2*(1-exp(-0.25*distance/1000));
    path_ifov = path/pi*(ifov)^2;
    
    
    sum_radiance = object_ifov + solar_ifov + ocean_ifov + sky_ifov + path_ifov;
else
    [distance,beta] = distance_to_seasurface (lat,lon,h,M,Ri2b,Rb2g,fov,u,w);
    if distance > -1
        transmiss = Transmittance(distance);
        e = 0.9 *(1-(1- cos(beta))^5);
        solar_ifov = solar/pi * (1 - e) * transmiss * (ifov)^2;
        ocean_ifov = ocean/pi * e  * transmiss * (ifov)^2;
        sky_ifov = sky/pi * (1 - e ) * transmiss * (ifov)^2;
        path_ifov = path/pi*(ifov)^2*(1-exp(-0.25*distance/1000));
        
        sum_radiance =  solar_ifov + ocean_ifov + sky_ifov + path_ifov;
    else
        sum_radiance =  sky/pi*(ifov)^2 + path/pi*(ifov)^2;
    end
end
rdiance = sum(sum(sum_radiance));
end

function in = in_or_out (Data,u,w)
sum = 0;
for i = 1:length(Data)
    if i == length(Data)
        vecto1 = -[u,w] + Data(i,:);
        vecto2 = -[u,w] + Data(1,:);
        
    else
        vecto1 = -[u,w] + Data(i,:);
        vecto2 = -[u,w] + Data(i+1,:);
    end
    cos_phi = vecto1*vecto2'/norm(vecto1)/norm(vecto2);
    if abs(cos_phi) > 1
        cos_phi = cos_phi/abs(cos_phi);
    end
    sum = sum + acos(cos_phi);
end
if sum >= 2*pi-0.001
    in = 1;
else
    in = 0;
end
end

function [distance,beta] = distance_to_face (Data,faces,lat,lon,h,M,Ri2b,Rb2g,fov,u,w)

if min(size(faces)) == 0
    distance = -1;
    beta = inf;
    fprintf('Error in run');
    return;
end
f = 320/tan(fov/2);
Ldonic = [u;w;f]/norm([u;w;f]);

NED = M'*Ri2b*Rb2g*Ldonic;

[ x, y, z ] = Geoditic2ECEF( lat, lon, h );

n = Data.faces_perpendicular(faces,:)';
d = Data.faces_gain_freedom(faces,:);
if (d+[x,y,z]*n) == 0
    [distance,beta] = distance_in_face (Data,faces,lat,lon,h,NED,u,w);
else
    distance = norm(NED)/abs(NED'*n)*abs((d+[x,y,z]*n));
    beta = acos(abs(NED'*n)/norm(NED)/norm(n));
end
end

function [distance,beta] = distance_in_face (Data,faces,lat,lon,h,NED,u,w)
[ x, y, z ] = Geoditic2ECEF( lat, lon, h );
count = 0;
range = zeros(0,0);
for i = 2:4
    if i==4
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

function NED = vector_calculator (lat,lon,h,lat1,lon1,h1)

[ x, y, z ] = Geoditic2ECEF( lat, lon, h );
[ x1,y1,z1] = Geoditic2ECEF( lat1,lon1,h1 );

NED = [x1-x;y1-y;z1-z]/norm([x1-x;y1-y;z1-z]);

end
%%
function [radiance_ifov] = pixel_radiance(solar,ocean,sky,path,lat,lon,h,M,Ri2b,Rb2g,e,i_pixel,j_pixel,fov)
ifov = fov/640;
[distance,beta] = distance_to_seasurface (lat,lon,h,M,Ri2b,Rb2g,fov,i_pixel,j_pixel);

[distance1,~] = distance_to_seasurface (lat,lon,h,M,Ri2b,Rb2g,fov,i_pixel+0.5,j_pixel+0.5);
[distance2,~] = distance_to_seasurface (lat,lon,h,M,Ri2b,Rb2g,fov,i_pixel+0.5,j_pixel-0.5);
[distance3,~] = distance_to_seasurface (lat,lon,h,M,Ri2b,Rb2g,fov,i_pixel-0.5,j_pixel+0.5);
[distance4,~] = distance_to_seasurface (lat,lon,h,M,Ri2b,Rb2g,fov,i_pixel-0.5,j_pixel-0.5);
if (distance1 > -1 && distance2 > -1&&distance3 > -1 && distance4 >-1)
    transmiss = Transmittance(distance);
    solar_ifov = solar/pi * (1 - e *(1-(1- cos(beta))^5)) * transmiss * (ifov)^2;
    ocean_ifov = ocean/pi * e *(1-(1- cos(beta))^5) * transmiss * (ifov)^2;
    sky_ifov = sky/pi * (1 - e *(1-(1- cos(beta))^5)) * transmiss * (ifov)^2;
    radiance_ifov = solar_ifov + ocean_ifov + sky_ifov + path/pi*(ifov)^2*(1-exp(-0.25*distance/1000));
elseif (distance1 == -1 && distance2 == -1 && distance3 == -1 && distance4 == -1)
    solar_ifov = 0;
    ocean_ifov = 0;
    sky_ifov = sky/pi*(ifov)^2;
    radiance_ifov = solar_ifov + ocean_ifov + sky_ifov + path/pi*(ifov)^2;
else
    radiance_ifov = horizoncal_radiance (solar,sky,ocean,path,ifov,lat,lon,h,M,Ri2b,Rb2g,fov,i_pixel,j_pixel);
end

end

function [radiance] = horizoncal_radiance (solar,sky,ocean,path,ifov,lat,lon,h,M,Ri2b,Rb2g,fov,i_pixel,j_pixel)
radiance_ifov = 0;
for i = -0.475:0.05:0.475
    for j = -0.475:0.05:0.475
        u = i_pixel + i;
        w = j_pixel + j;
        [distance,beta] = distance_to_seasurface (lat,lon,h,M,Ri2b,Rb2g,fov,u,w);
        if distance > -1
            transmiss = Transmittance(distance);
            e = 0.9 *(1-(1- cos(beta))^5);
            solar_ifov = solar/pi * (1 - e) * transmiss * (ifov/20)^2;
            ocean_ifov = ocean/pi * e  * transmiss * (ifov/20)^2;
            sky_ifov = sky/pi * (1 - e ) * transmiss * (ifov/20)^2;
            path_ifov = path/pi*(ifov/20)^2*(1-exp(-0.25*distance/1000));
            radiance_ifov = radiance_ifov + solar_ifov + ocean_ifov + sky_ifov + path_ifov;
        else
            radiance_ifov = radiance_ifov + sky/pi*(ifov/20)^2 + path/pi*(ifov/20)^2;
        end
    end
end
radiance = radiance_ifov;
end

function [d,beta] = distance_to_seasurface (lat,lon,h,M,Ri2b,Rb2g,fov,i_pixel,j_pixel)

f = 640/2/tan(fov/2);
u = i_pixel;
w = j_pixel;

Ldonic = [u w f]'/norm([u w f]);

NED = M' * Ri2b * Rb2g * Ldonic;

[ xA, yA, zA ] = Geoditic2ECEF( lat, lon, h );
NED1 = [xA yA zA]'/norm([xA,yA,zA]);
% fprintf('\n%f',NED'*NED1)
L = norm([xA, yA, zA])*abs(NED'*NED1);
fprintf('L = %f\n',L);
[~,~,h0] = ECEF2Geoditic( xA+L*NED(1,1), yA+L*NED(2,1), zA+L*NED(3,1) );

if h0 > 0
    d = -1;
    beta = -1;
else
    [d,beta] = calculator ( xA,yA,zA,NED,L,h,h0 );
end
end

function [d,beta] = calculator ( xA,yA,zA,NED,L,h,h0 )
loop = 0;
while (abs(h0)>0.05 && loop < 2000)
    L = L * h/(h-h0);
    [~,~,h0] = ECEF2Geoditic( xA+L*NED(1,1), yA+L*NED(2,1), zA+L*NED(3,1) );
    loop = loop + 1;
end
d = L;
% fprintf('\n%f',h0)
NED1 = [xA+L*NED(1,1), yA+L*NED(2,1), zA+L*NED(3,1)]'/norm([xA+L*NED(1,1), yA+L*NED(2,1), zA+L*NED(3,1)]);
beta = acos(abs(NED'*NED1));
end

%%
function [u,w]  = imageModel(lat,lon,h,lat0,lon0,h0,M,Ri2b,Rb2g,fov)

[ x0, y0, z0 ] = Geoditic2ECEF( lat0, lon0, h0 );
[ x, y, z ] = Geoditic2ECEF( lat, lon, h );

distance = norm([x-x0,y-y0,z-z0]);
NED = [x-x0,y-y0,z-z0]'/distance;

Ldonic = Rb2g' * Ri2b' * M * NED;
if Ldonic(3,1)< 0
    Ldonic = -Ldonic;
end
f = 320/tan(fov/2);
u = Ldonic(1,1)/Ldonic(3,1)*f;
w = Ldonic(2,1)/Ldonic(3,1)*f;
end

function M = Recef2inertia (lat,lon)
lambda = lat / 180.0 * pi;
phi = lon / 180.0 * pi;

sin_lambda = sin(lambda);
cos_lambda = cos(lambda);
sin_phi = sin(phi);
cos_phi = cos(phi);

M = [-cos_phi * sin_lambda -sin_lambda * sin_phi cos_lambda;
    -sin_phi              cos_phi               0;
    -cos_lambda * cos_phi -cos_lambda * sin_phi -sin_lambda];
end

function Ri2b = Rinertia2body (roll,pitch,yaw)

Ri2b = [cos(pitch)*cos(yaw) sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw) cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw);
    cos(pitch)*sin(yaw) sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw) cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw);
    -sin(pitch)          sin(roll)*cos(pitch)                             cos(roll)*cos(pitch)];
end

function Rb2g = Rbody2gimbal (azimuth,elevator)
Rb2g = [-sin(azimuth) sin(elevator)*cos(azimuth) cos(elevator)*cos(azimuth);
    cos(azimuth) sin(elevator)*sin(azimuth) cos(elevator)*sin(azimuth);
    0          cos(elevator)              -sin(elevator)];
end

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

function gain = Transmittance (distance)

gain = exp(-0.26*distance/1000);

end

