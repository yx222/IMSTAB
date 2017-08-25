classdef HexapodController < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Need to define the inputs and outputs of the controller
        
        % Controller properties
        % 6x6 feedback matrix, mapping ipd to control
        % [x,y,z,u,v,w]' = fb_matrix*[int_ex; ex; der_ex; int_ez; ez; der_ez]
                            
        fb_matrix =     [2,   0.3,  0.01,  0,   0,    0; 
                         0,   0,    0,     0,   0,    0; 
                         0,   0,    0,     2,   0.3,  0.01; 
                         0,   0,    0,     2,   0.3,  0.01; 
                         0,   0,    0,     0,   0,    0; 
                         2,   0.3,  0.01,  0,   0,    0]
                     
        control_gain = [2, 0, 2, 1.5/4, 0, 1.5/4]'
                    
        time_step           % [s]
        
        % bound on the control values
        % Ideally we get them from the plant
%         int_bound = [40, 20];
        u_bound = [45, 45, 25, 16, 16, 34]';
        
        N_dim = 2;          % target dimension
        
        % states and memories?
        e_states            %[integral error; error], 2*N_dim
        target              % control target
        
    end
    
    methods
        % Constructor
        function obj = HexapodController(time_step, mode)
            if nargin == 0
                obj.time_step = 0.1;
            else
                obj.time_step = time_step;
            end
            
            % Initialise contorller states
            obj.e_states = zeros(2, obj.N_dim);
            
            % what DOF of control do we use?
            switch mode
                case 'xyz'
                    obj.control_gain = obj.control_gain.*[1, 1, 1, 0, 0, 0]';
                case 'uvw'
                    obj.control_gain = obj.control_gain.*[0, 0, 0, 1, 1, 1]';
                case 'hybrid'
                    % do nothing
                    rotation = 1;
                    translation = 1-rotation;
                    split = [ones(3,1)*translation; ones(3,1)*rotation];
                    obj.control_gain = obj.control_gain.*split;

            end
            
            % scale feedback gain
            obj.fb_matrix = bsxfun(@times, obj.fb_matrix, obj.control_gain);
            
        end
            
            
        % Destructor
        function delete(obj)
            
        end
        
        %% Main external methods
        
        % Get control inputs
        function u = step(obj, y, y_target)
            % Accepts y and y_targets as row vectors and returns control u
            % as a row vector
            
            % error signal
            if nargin < 3
                y_target = obj.target;
            end
            e = y_target - y;
            
            % update derivative before state is overwritten (fwd diff)
            e_der = (e - obj.e_states(2,:))/obj.time_step;
              
            % updates the states
            obj.e_states = [obj.e_states(1,:) + e*obj.time_step; ...
                            e];
                        
            % saturate the integral contributions to avoid integrator
            % wind-up
            % In the sparse feedback matrix, no controls have contribution
            % from more than one integral error, which makes bounding
            % easier (but code dirtier, with hard-coded dimension index)
            
            int_ex_max = min(obj.u_bound./sum(obj.fb_matrix(:,1)+eps));
            int_ex_min = max(-obj.u_bound./sum(obj.fb_matrix(:,1)+eps));
            int_ez_max = min(obj.u_bound./sum(obj.fb_matrix(:,4)+eps));
            int_ez_min = max(-obj.u_bound./sum(obj.fb_matrix(:,4)+eps));
            
            int_e_max = [int_ex_max, int_ez_max];
            int_e_min = [int_ex_min, int_ez_min];
            
            obj.e_states(1,:) = min(obj.e_states(1,:), int_e_max);
            obj.e_states(1,:) = max(obj.e_states(1,:), int_e_min);
            
            % Compute control signal            
            ipd = [obj.e_states; e_der]; % 3*N_dim
            
            % convert ipd to vector
            ipd = ipd(:); %[int_ex; ex; der_ex; int_ez; ez; der_ez]
            
            u = obj.fb_matrix*ipd; % 1*N_dim      
            
            % invert x translational control
            u(1) = -u(1);
        end
        
    end
    
end

