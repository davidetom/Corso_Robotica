classdef SimProjectUtils < handle
    %SimProjectUtils Class to handle simulator scene    
    properties
        robot %handle for robot
        object %vectors of handles for objects
        No = 0; % number of objects to be controlled in the scene
        

        %%%%% Simulator parameters
        simulator %Simulator instance
        client %Simulator client
        Mode %Simulator mode
        
        %%%%% Static parameters        
        Port = 19998; %Communication port
        Timeout = 5000; %timeout for response (ms)
        SampleTimeComm =  50; %ms
        IP = '127.0.0.1';
        
    end %end properties
    
    methods
              
        function obj = SimProjectUtils(path_lib_matlab, path_lib_so, No)
            
            addpath(path_lib_matlab)
            addpath(path_lib_so)


            if exist('No','var') obj.No=No;   else obj.No=0; end
           
            simulator=remApi('remoteApi');%,'extApi.h');            
            client = simulator.simxStart(obj.IP, obj.Port, true, true, obj.Timeout, obj.SampleTimeComm);
            Mode = simulator.simx_opmode_oneshot; %communication mode (see manual remote API)
            if (client<=-1)
                error('Error simulator Client');
            end
            pause(0.2);
            
            obj.simulator=simulator;
            obj.client=client;
            obj.Mode=Mode;
            try
                obj = obj.initRobot(); 
                
                %% Initialization handles for objects
                for j=1:obj.No
                    toConc=num2str(j);
                    
                    [err, obj.object(j).Handle] = simulator.simxGetObjectHandle(client, ['hat',toConc],  simulator.simx_opmode_oneshot_wait);
                    if(err)
                        error(['Err reading pointer to object']);
                    end
                    
                    [err, obj.object(j).Position]=simulator.simxGetObjectPosition(client, obj.object(j).Handle, -1,  simulator.simx_opmode_oneshot_wait);
                    if(err)
                        error(['Err reading pointer to object']);
                    end
                    
                    [err, obj.object(j).Orientation]=simulator.simxGetObjectOrientation(client, obj.object(j).Handle, -1,  simulator.simx_opmode_oneshot_wait);
                    if(err)
                        error(['Err reading pointer to object']);
                    end
                end

                %% Simulator start
                err=simulator.simxStartSimulation(client, simulator.simx_opmode_oneshot_wait);
                if(err>1)
                    error(['Err starting simulation']);
                end
            catch
                %Stop simulator connection
                obj.StopConnection();
                %Display the error
                error(lasterr);
            end

           
        end %end InitConnection
        
        function obj = initRobot(obj)
            list_joints = {'shoulder_pitch', 'shoulder_roll', 'elbow_yaw', 'elbow_pitch', 'wrist_roll', 'wrist_pitch', 'wrist_yaw'}; 
            for ii=1:length(list_joints)
                [err, obj.robot.LeftJointHandle(ii)] = obj.simulator.simxGetObjectHandle(obj.client, ['l_', list_joints{ii}],  obj.simulator.simx_opmode_oneshot_wait);
                if(err)
                    error(['Err reading pointer to revolute joint of the manipulator. Left Joint: ', num2str(ii), '. Check that Simulator is active and the connection established']);
                end
                [err, obj.robot.RightJointHandle(ii)] = obj.simulator.simxGetObjectHandle(obj.client, ['r_', list_joints{ii}],  obj.simulator.simx_opmode_oneshot_wait);
                if(err)
                    error(['Err reading pointer to revolute joint of the manipulator. Right Joint: ', num2str(ii), '. Check that Simulator is active and the connection established']);
                end
            end

            [err, obj.robot.GraspHandle] = obj.simulator.simxGetObjectHandle(obj.client, ['l_EE_FRAME'],  obj.simulator.simx_opmode_oneshot_wait);
            if(err)
                error(['Err reading pointer to robot hand']);
            end 


        end %end initRobot
        
        function obj = StopConnection(obj)
            if( ~isempty(obj.simulator) )%&& ~isempty(obj.simulator) )
                disp('Disconnecting Simulator')
                obj.simulator.simxStopSimulation(obj.client, obj.simulator.simx_opmode_oneshot_wait);
                obj.simulator.simxFinish(obj.client);
                clear obj.simulator;
            end
        end %end StopConnection

        function [err ] = moveLeftArm(obj, q)
            % INPUT: 
            %        q - joint position in Re^7 
            err(1:7) = 0;
            coppelia_offset = [0, pi/2, pi/2, -pi/2, 0, pi/2, 0];
            for i=1:7
                err(i) = obj.simulator.simxSetJointPosition(obj.client, obj.robot.LeftJointHandle(i), q(i)+coppelia_offset(i), obj.Mode);
                if err
                    warning('Problem positionig joint simulator')
                end
            end
        end %end 

        function [err ] = moveRightArm(obj, q)
            % INPUT: 
            %        q - joint position in Re^7 
            err(1:7) = 0;
            for i=1:7
                err(i) = obj.simulator.simxSetJointPosition(obj.client, obj.robot.RightJointHandle(i), q(i), obj.Mode);
                if err
                    warning('Problem positionig joint simulator')
                end
            end

   
    
        end %end

        function [ ] = graspObject( obj)   
            % Moves the ind_obj-th object          
            % INPUT: color_string - string of the color of the object to move
            ind_o = 1; 

            keep_in_place = 1; 
            [err] = obj.simulator.simxSetObjectParent(obj.client, obj.object(ind_o).Handle, obj.robot.GraspHandle, keep_in_place, obj.Mode); 


        end %end moveObject       

        
        function [ ] = releaseObject( obj)   
            % Moves the ind_obj-th object          
            % INPUT: color_string - string of the color of the object to move
            ind_o = 1; 

            keep_in_place = 1; 
            [err] = obj.simulator.simxSetObjectParent(obj.client, obj.object(ind_o).Handle, -1, keep_in_place, obj.Mode); 


        end %end moveObject  
        
 
        function delete(obj)
            % Desctructor method
            obj.StopConnection();
        end 
    end  % end public methods 
    
    methods(Access='private')
        function [X] = sk(obj, x)            
            X=[0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ];
        end
        function [ angles ] = rot2SimulatorAngles( obj, R )
            %ROT2SimulatorANGLES it calculates the XYZ Euler angles convention (Simulator convention)
            sy = sqrt(R(1,1) * R(1,1) +  R(2,1) * R(2,1) );
            angles(1) = atan2(R(3,2) , R(3,3));
            angles(2) = atan2(-R(3,1), sy);
            angles(3) = atan2(R(2,1), R(1,1));
        end
        
        
    end % end private methods 
    
end %end class


