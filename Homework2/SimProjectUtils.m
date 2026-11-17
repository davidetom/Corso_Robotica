classdef SimProjectUtils < handle
    %SimProjectUtils Class to handle simulator scene    
    properties
        robot %handle for robot
        object %vectors of handles for objects
        No = 0; % number of objects to be controlled in the scene
        
        sim % ZMQ sim object
    end %end properties
    
    methods
              
        function obj = SimProjectUtils(sim, No)
            % Inizializza il numero di oggetti
            if exist('No','var') && ~isempty(No)
                obj.No = No;   
            else 
                obj.No = 0; 
            end
           
            % Salva l'oggetto sim di ZMQ
            obj.sim = sim;
            
            try
                obj = obj.initRobot(); 
                
                %% Initialization handles for objects
                for j = 1:obj.No
                    obj.initObject(j);
                end

                %% Simulator start
                obj.sim.startSimulation();
            catch ME
                % Stop simulator connection in caso di errore
                obj.StopConnection();
                rethrow(ME);
            end
           
        end %end Constructor
        
        function obj = initRobot(obj)
            list_joints = {'shoulder_pitch', 'shoulder_roll', 'elbow_yaw', 'elbow_pitch', 'wrist_roll', 'wrist_pitch', 'wrist_yaw'}; 
            for ii = 1:length(list_joints)
                % Nelle versioni recenti di CoppeliaSim si usa il path assoluto con '/'
                obj.robot.LeftJointHandle(ii) = obj.sim.getObject(['/l_', list_joints{ii}]);
                obj.robot.RightJointHandle(ii) = obj.sim.getObject(['/r_', list_joints{ii}]);
            end
            obj.robot.GraspHandle = obj.sim.getObject('/l_EE_FRAME');
        end %end initRobot
        
        function obj = StopConnection(obj)
            if ~isempty(obj.sim)
                disp('Disconnecting Simulator')
                obj.sim.stopSimulation();
            end
        end %end StopConnection

        function [err] = moveLeftArm(obj, q)
            % INPUT: q - joint position in Re^7 
            err = zeros(1,7);
            % Tabella DH con x4 verso il polso (prima: [0, pi/2, pi/2, pi/2, 0, -pi/2, 0])
            coppelia_offset = [0, pi/2, pi/2, -pi/2, 0, pi/2, 0]; 
            for i=1:7
                % ZMQ: Imposta direttamente la posizione del giunto
                obj.sim.setJointPosition(obj.robot.LeftJointHandle(i), q(i) + coppelia_offset(i));
                
                % Nota: se il robot usa giunti dinamici (motori abilitati nel simulatore), 
                % potresti dover usare: obj.sim.setJointTargetPosition(...) invece di setJointPosition
            end
        end %end 

        function [err] = moveRightArm(obj, q)
            % INPUT: q - joint position in Re^7 
            err = zeros(1,7);
            for i=1:7
                obj.sim.setJointPosition(obj.robot.RightJointHandle(i), q(i));
            end
        end %end 

        function [] = graspObject(obj, ind_o)
            % Attacca l'oggetto ind_o-esimo (default 1, es. '/hat1') all'end-effector sinistro
            if nargin < 2 || isempty(ind_o)
                ind_o = 1;
            end
            obj.ensureObject(ind_o);

            keep_in_place = true;
            obj.sim.setObjectParent(obj.object(ind_o).Handle, obj.robot.GraspHandle, keep_in_place);
        end %end graspObject

        function [] = releaseObject(obj, ind_o)
            % Rilascia l'oggetto ind_o-esimo (default 1, es. '/hat1') dall'end-effector
            if nargin < 2 || isempty(ind_o)
                ind_o = 1;
            end
            obj.ensureObject(ind_o);

            keep_in_place = true;
            obj.sim.setObjectParent(obj.object(ind_o).Handle, -1, keep_in_place);
        end %end releaseObject
        
        function delete(obj)
            % Destructor method
            obj.StopConnection();
        end 
    end  % end public methods 
    
    methods(Access='private')
        function initObject(obj, j)
            % Inizializza handle, posizione e orientamento dell'oggetto j-esimo ('hat1', 'hat2', ...)
            toConc = num2str(j);
            obj.object(j).Handle = obj.sim.getObject(['/hat', toConc]);
            pos = obj.sim.getObjectPosition(obj.object(j).Handle, obj.sim.handle_world);
            ori = obj.sim.getObjectOrientation(obj.object(j).Handle, obj.sim.handle_world);
            if iscell(pos), pos = cell2mat(pos); end
            if iscell(ori), ori = cell2mat(ori); end
            obj.object(j).Position = pos;
            obj.object(j).Orientation = ori;
        end

        function ensureObject(obj, ind_o)
            % Carica l'handle di '/hat<ind_o>' su richiesta se No era 0 alla creazione
            if isempty(obj.object) || numel(obj.object) < ind_o || ~isfield(obj.object(ind_o), 'Handle') || isempty(obj.object(ind_o).Handle)
                obj.initObject(ind_o);
                if obj.No < ind_o
                    obj.No = ind_o;
                end
            end
        end

        function [X] = sk(~, x)            
            X=[0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ];
        end
        function [ angles ] = rot2SimulatorAngles( ~, R )
            %ROT2SimulatorANGLES it calculates the XYZ Euler angles convention
            sy = sqrt(R(1,1) * R(1,1) +  R(2,1) * R(2,1) );
            angles(1) = atan2(R(3,2) , R(3,3));
            angles(2) = atan2(-R(3,1), sy);
            angles(3) = atan2(R(2,1), R(1,1));
        end
    end % end private methods 
    
end %end class