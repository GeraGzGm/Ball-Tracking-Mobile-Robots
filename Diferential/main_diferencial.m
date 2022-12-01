
function main()
clear all;clc;%close all;
    R = 0.0975;
    L = 0.381;
    
    c_d = [337 242]';
    a_d = 11000;
    
    e = [0 0 0]';
    e_sum = [0 0 0]';
    e_old = [0 0 0]';

    kp = [0.00002 0.001 0.001]';
    ki = [0.0 0.00001 0.00001]';
    kd = [0.0 0.0001 0.0001]';

    VREP = remApi('remoteApi');
    VREP.simxFinish(-1);
    ClientID = VREP.simxStart('127.0.0.1',19999,true,true,5000,5);

    if ClientID == -1
        disp('Failed connecting to remote API server');
        return
    end

    [~,camera] = VREP.simxGetObjectHandle(ClientID,'kinect_rgb',VREP.simx_opmode_oneshot_wait);
    [~,Pioneer] = VREP.simxGetObjectHandle(ClientID,'Pioneer_p3dx',VREP.simx_opmode_oneshot_wait);
    [~,motorL] = VREP.simxGetObjectHandle(ClientID,'Pioneer_p3dx_leftMotor',VREP.simx_opmode_oneshot_wait);
    [~,motorR] = VREP.simxGetObjectHandle(ClientID,'Pioneer_p3dx_rightMotor',VREP.simx_opmode_oneshot_wait);

    while VREP.simxGetConnectionId(ClientID)>-1
        tic
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
                plot(xc,yc,'.r','MarkerSize',10)
                plot(c_d(1),c_d(2),'.g','MarkerSize',10);
                drawnow
            end
            
            e = [a_d - area ; c_d - [xc yc]'];%lineal;angular
            e_sum = e*toc + e_sum;

            u = kp.*e + ki.*e_sum + kd.*(e-e_old)/toc;

            wr  = (2*u(1) + u(2)*L)/(2*R);
            wl  = (2*u(1) - u(3)*L)/(2*R);

            VREP.simxSetJointTargetVelocity(ClientID,motorL,wl,VREP.simx_opmode_streaming);
            VREP.simxSetJointTargetVelocity(ClientID,motorR,wr,VREP.simx_opmode_streaming);
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