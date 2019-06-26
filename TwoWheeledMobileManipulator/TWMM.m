% Make sure to have the server side running in V-REP: 
% in a child script of a V-REP scene, add following command
% to be executed just once, at simulation start:
%
% simRemoteApi.start(19999)
%
% then start simulation, and run this program.
%
% IMPORTANT: for each successful call to simxStart, there
% should be a corresponding call to simxFinish at the end!

function TWMM()
    disp('Program started');
    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);

    if (clientID>-1)
        disp('Connected to remote API server');
        %%%%%%%%%%% below are the main code %%%%%%%%%%%%%
        % synchronous mode
        vrep.simxSynchronous(clientID,true);
        % start the simulation:
        vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking); % vrep.simx_opmode_oneshot
        vrep.simxSynchronousTrigger(clientID);
        % get object handle
        [~,LeftMotor]=vrep.simxGetObjectHandle(clientID,'LeftMotor',vrep.simx_opmode_blocking);
        [~,RightMotor]=vrep.simxGetObjectHandle(clientID,'RightMotor',vrep.simx_opmode_blocking);
        [~,MobileBody]=vrep.simxGetObjectHandle(clientID,'MobileBody',vrep.simx_opmode_blocking);
        [~,Link1Motor]=vrep.simxGetObjectHandle(clientID,'Link1Motor',vrep.simx_opmode_blocking);
        
        % parameter initiation
        t=clock; startTime=t(6)+60*t(5); currentTime=t(6)+60*t(5);
        vrep.simxSetJointTargetVelocity(clientID,LeftMotor,0,vrep.simx_opmode_oneshot);
        vrep.simxSetJointTargetVelocity(clientID,RightMotor,0,vrep.simx_opmode_oneshot);
        vrep.simxSetJointTargetVelocity(clientID,Link1Motor,0,vrep.simx_opmode_oneshot);
        vrep.simxSetJointForce(clientID,LeftMotor,0,vrep.simx_opmode_oneshot);
        vrep.simxSetJointForce(clientID,RightMotor,0,vrep.simx_opmode_oneshot);
        z0 = 0.225-0.1; % initial distance between link0's CoM and body center
        z1 = 0.34-0.1; % initial distance between bracket's CoM and body center
        z2 = 0.435-0.35; % initial distance between link1's CoM and link1 joint
        zj1 = 0.35-0.1; % initial distance between link1 joint and body center
        m0 = 0.2813; % mass of the link0
        m1 = 0.2813; % mass of the link1
        mb = 0.01563; % mass of one bracket
        sum_error = 0; % integral of error for PID "I" part
        % first calls of the "Get" function 
        [res,V,~]=vrep.simxGetObjectVelocity(clientID,MobileBody,vrep.simx_opmode_streaming);
        [~,t1]= vrep.simxGetJointPosition(clientID,Link1Motor,vrep.simx_opmode_streaming);
        [~,accel_read]=vrep.simxGetObjectOrientation(clientID,MobileBody,-1,vrep.simx_opmode_streaming);
        % 
        while (currentTime-startTime<120)
            % read set value from UI
            [~, ~, SetValue, ~, ~]=vrep.simxCallScriptFunction(clientID,'MobileBody',vrep.sim_scripttype_childscript,'SetValueRead',[],[],[],[],vrep.simx_opmode_blocking);
            v_ref = SetValue(1);
            joint_velocity = SetValue(2);
            direction = SetValue(3);
            vrep.simxSetJointTargetVelocity(clientID,Link1Motor,joint_velocity,vrep.simx_opmode_oneshot);
            %% Attitude Control
            % read the GyroSensor, get angular acceleration
            [~, ~, gyro_read, ~, ~]=vrep.simxCallScriptFunction(clientID,'GyroSensor',vrep.sim_scripttype_childscript,'gyroRead',[],[],[],[],vrep.simx_opmode_blocking);
            vrep.simxSynchronousTrigger(clientID);
            omega = gyro_read(2); % angular velocity
            % get tilt angle
            [~,accel_read]=vrep.simxGetObjectOrientation(clientID,MobileBody,-1,vrep.simx_opmode_buffer);
            vrep.simxSynchronousTrigger(clientID);
            a = accel_read(1); b = accel_read(2); g = accel_read(3);
            x = cos(b)*cos(g);
            y = sin(a)*sin(b)*cos(g)+cos(a)*sin(g);
            z = -cos(a)*sin(b)*cos(g)+sin(a)*sin(g);
            t0 = -atan(z/sqrt(x^2+y^2));
            % get Link 1 joint angle
            [~,t1]= vrep.simxGetJointPosition(clientID,Link1Motor,vrep.simx_opmode_buffer);% the angle of the link1 joint /rad
            vrep.simxSynchronousTrigger(clientID);
            % compute the positon of the center of mass of link0,link1 and brackets
            xm = ( m0*z0*sin(0)+2*mb*z1*sin(0)+m1*(zj1*sin(0)+z2*sin(t1)) )/(m0+m1+mb);
            zm = ( m0*z0*cos(0)+2*mb*z1*cos(0)+m1*(zj1*cos(0)+z2*cos(t1)) )/(m0+m1+mb);
            angle_ref = -atan(xm/zm)/pi*180;
            t0 = t0/pi*180;% turn t0 into degree
            P_angle = 0.041; D_angle = 0.15; % P,D value of angle control 0.028 0.12
            AngleControl = P_angle*(t0-angle_ref) + D_angle*(omega); % PD control
            
            %% SpeedControl
            [res,V,~]=vrep.simxGetObjectVelocity(clientID,MobileBody,vrep.simx_opmode_buffer);
            vrep.simxSynchronousTrigger(clientID);
            v_now = sqrt(V(1)^2+V(2)^2+V(3)^2);%V(1)
            % decied the orientation of the velocity
            if([x,y]*[V(1);V(2)]<0)
                v_now = -v_now;
            end
%           
            Error = v_ref - v_now;
            sum_error = sum_error + Error;
            sum_error_threshold = 4;
            if(sum_error>sum_error_threshold)
                sum_error=sum_error_threshold;
            elseif(sum_error<-sum_error_threshold)
                sum_error=-sum_error_threshold;
            end
            
            P_speed = 0.12; I_speed = 0.06; % P,I value of speed control 0.015 0.0028
            if(abs(Error)<0.1)
                I_speed = 0.01;
            end
            SpeedControl = P_speed*(v_ref-v_now) + I_speed*sum_error;
            
            %% Direction Control
            DirectionControl = direction/4;
            %% Motor output
            LeftMotor_output = AngleControl - SpeedControl + DirectionControl;
            RightMotor_output = AngleControl - SpeedControl - DirectionControl;
            if LeftMotor_output<0
                Leftvelocity=-999999999;
            else
                Leftvelocity=999999999;
            end
            if RightMotor_output<0
                Rightvelocity=-999999999;
            else
                Rightvelocity=999999999;
            end
            
            vrep.simxSetJointForce(clientID,LeftMotor,abs(LeftMotor_output),vrep.simx_opmode_oneshot);
            vrep.simxSetJointForce(clientID,RightMotor,abs(RightMotor_output),vrep.simx_opmode_oneshot);
            vrep.simxSynchronousTrigger(clientID);
            vrep.simxSetJointTargetVelocity(clientID,LeftMotor,Leftvelocity,vrep.simx_opmode_oneshot);
            vrep.simxSetJointTargetVelocity(clientID,RightMotor,Rightvelocity,vrep.simx_opmode_oneshot);
            vrep.simxSynchronousTrigger(clientID);
            t=clock; currentTime=t(6)+60*t(5);
        end
        % stop the simulation:
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);
        %%%%%%%%%%% above are the main code %%%%%%%%%%%%%
        % Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        vrep.simxGetPingTime(clientID);
        % Now close the connection to V-REP:    
        vrep.simxFinish(clientID);
    else
        disp('Failed connecting to remote API server');
    end
    vrep.delete(); % call the destructor!
    
    disp('Program ended');
end
