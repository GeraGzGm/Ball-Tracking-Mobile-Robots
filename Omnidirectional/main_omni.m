function main()
clear all;close all;clc;
   L = 0.1981;
   l = 0.1990;
   format short g 
   
    c_d = [300 270]';
    a_d = 11000;
    
    e = [0 0 0]';
    e_sum = [0 0 0]';
    e_old = [0 0 0]';

    kp = [0.0001 0.015 0.015]';
    ki = [0.0 0.0 0.0]';
    kd = [0.0 0.0 0.0]';

    VREP = remApi('remoteApi');
    VREP.simxFinish(-1);
    ClientID = VREP.simxStart('127.0.0.1',19999,true,true,5000,5);

    if ClientID == -1
        disp('Failed connecting to remote API server');
        return
    end

    
    [~,camera] = VREP.simxGetObjectHandle(ClientID,'kinect_rgb',VREP.simx_opmode_oneshot_wait);
    [~,Omni] = VREP.simxGetObjectHandle(ClientID,'OmniPlatform',VREP.simx_opmode_oneshot_wait);
    [~,motor_v1] = VREP.simxGetObjectHandle(ClientID,'OmniWheel_regularRotation#1',VREP.simx_opmode_oneshot_wait);
    [~,motor_v2] = VREP.simxGetObjectHandle(ClientID,'OmniWheel_regularRotation#0',VREP.simx_opmode_oneshot_wait);
    [~,motor_v3] = VREP.simxGetObjectHandle(ClientID,'OmniWheel_regularRotation#2',VREP.simx_opmode_oneshot_wait);
    [~,motor_v4] = VREP.simxGetObjectHandle(ClientID,'OmniWheel_regularRotation',VREP.simx_opmode_oneshot_wait);

    while VREP.simxGetConnectionId(ClientID)>-1
        tic
        [~,position] = VREP.simxGetObjectPosition(ClientID,Omni,-1,VREP.simx_opmode_streaming);
        [~,orientation] = VREP.simxGetObjectOrientation(ClientID,Omni,-1,VREP.simx_opmode_streaming);
        [~,linV,angV] = VREP.simxGetObjectVelocity(ClientID,Omni,-1);
    
        p = [position(1) position(2) orientation(3)]';
        
        [~,~,img] = VREP.simxGetVisionSensorImage2(ClientID,camera,0,VREP.simx_opmode_streaming);
        TF = isempty(img);

        if ~TF
            seg = segmentacion(img);
            edges = edge(seg,'prewitt');%Contorno del círculo

            if sum(sum(edges)) ~= 0
                [y,x] = find(edges==1);
                [xc,yc,r] = centros_y_radio(x,y);
                area = pi*r^2;

                subplot(1,2,1);imshow(img);
                subplot(1,2,2);imshow(edges);hold on;
                plot(xc,yc,'.r','MarkerSize',10);
                plot(c_d(1),c_d(2),'.g','MarkerSize',10);
                drawnow
            end

            e = [a_d - area ; c_d - [xc yc]'];%lineal;angular
            e_sum = e*toc + e_sum;

            u = kp.*e + ki.*e_sum + kd.*(e-e_old)/toc;
            
            alpha = p(3) + pi/4;
            v = [sqrt(2)*sin(alpha) -sqrt(2)*cos(alpha) -(L+l);
           sqrt(2)*cos(alpha) sqrt(2)*sin(alpha) (L+l);
           sqrt(2)*cos(alpha) sqrt(2)*sin(alpha) -(L+l);
           sqrt(2)*sin(alpha) -sqrt(2)*cos(alpha) (L+l)]*u;
            

            if abs(e)*0.0001 <= [0.5;0.003;0.003]
                v = zeros(1,4)';
            end
       
       
            VREP.simxSetJointTargetVelocity(ClientID,motor_v1,v(1),VREP.simx_opmode_streaming);
           VREP.simxSetJointTargetVelocity(ClientID,motor_v2,-v(2),VREP.simx_opmode_streaming);
           VREP.simxSetJointTargetVelocity(ClientID,motor_v3,v(3),VREP.simx_opmode_streaming);
           VREP.simxSetJointTargetVelocity(ClientID,motor_v4,-v(4),VREP.simx_opmode_streaming);
   
            e_old = e;
        end
    end
end

function [xc,yc,r]=centros_y_radio(x,y)
    xc = sum(x)/length(x); %Centros
    yc = sum(y)/length(y);

    xr_l = xc - min(x);
    xr_r = max(x) - xc;

    yr_u = yc - min(y);
    yr_d = max(y) - yc;

    r = (xr_l+xr_r+yr_u+yr_d)/4; %Radio promediado de los 4 lados
end

function im_b = segmentacion(im)
    [n,m,~] = size(im);

    R_c = 0*ones(n,m);
    G_c = 200*ones(n,m);
    B_c = 0*ones(n,m);

    r = 100;

    im_b = (R_c - double(im(:,:,1))).^2 + (G_c - double(im(:,:,2))).^2 + (B_c - double(im(:,:,3))).^2;
    im_b = 255*(im_b <= r^2);
end