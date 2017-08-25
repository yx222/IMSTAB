classdef HexapodController < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Need to define the inputs and outputs of the controller
        
        % Controller properties
        P = 0.1;
        I = 2;
        D = 0.01;
        fb_matrix = [];
        
        
        % scaling of each control dimensoin
        control_gain = [2, 0, 2, 1.5/4, 0, 1.5/4]'
                    
        time_step           % [s]
        
        % bound on the control values
        % Ideally we get them from the plant
        u_bound = [45, 45, 25, 16, 16, 34]';
        
        N_dim = 2;          % target dimension
        
        % split of control in hybrid mode
        r_rotation = 0.9;
        
        % states and memories?
        e_states            %[integral error; error], 2*N_dim
        target              % control target
        int_bound           % integrator bound to avoid wind-up
        
    end
    
    methods
        % Constructor
        function obj = HexapodController(time_step, mode)
            if nargin == 0
                obj.time_step = 0.1;
                mode = 'hybrid';
            else
                obj.time_step = time_step;
            end
            
            % set feedback matrix
            obj.set_fb_matrix(obj.I, obj.P, obj.D)
            
            
            % Initialise contorller states
            obj.e_states = zeros(2, obj.N_dim);
            
            % what DOF of control do we use?
            switch mode
                case 'xyz'
                    obj.control_gain = obj.control_gain.*[1, 1, 1, 0, 0, 0]';
                case 'uvw'
                    obj.control_gain = obj.control_gain.*[0, 0, 0, 1, 1, 1]';
                case 'hybrid'
                    % What's the split between rotation and translation?
                    
                    % apply weighted slit on the gian.
                    split = [ones(3,1)*(1-obj.r_rotation); ones(3,1)*obj.r_rotation];
                    obj.control_gain = obj.control_gain.*split;

            end
            
            % scale feedback gain
            obj.fb_matrix = bsxfun(@times, obj.fb_matrix, obj.control_gain);
            
            % acquire integrator saturation values
            obj.set_int_bound();
            
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
            obj.e_states(1,:) = min(obj.e_states(1,:), obj.int_bound(2,:));
            obj.e_states(1,:) = max(obj.e_states(1,:), obj.int_bound(1,:));
            
            % Compute control signal            
            ipd = [obj.e_states; e_der]; % 3*N_dim
            
            % convert ipd to vector
            ipd = ipd(:); %[int_ex; ex; der_ex; int_ez; ez; der_ez]
            
            u = obj.fb_matrix*ipd; % 1*N_dim      
        end
        
        % function to build fb matrix
        function set_fb_matrix(obj, I, P, D)
            % 6x6 feedback matrix, mapping ipd to control
            % [x,y,z,u,v,w]' = fb_matrix*[int_ex; ex; der_ex; int_ez; ez; der_ez]
            
            % x to x control needs to be inverted due to sign convention. So the first row

            obj.fb_matrix = [-I,  -P, -D,  0,   0,    0;
                              0,   0,   0,  0,   0,    0;
                              0,   0,   0,  I,   P,    D;
                              0,   0,   0,  I,   P,    D;
                              0,   0,   0,  0,   0,    0;
                              I,   P,   D,  0,   0,    0];
        end
        
        % method to set integrator bound
        function set_int_bound(obj)
            % In the sparse feedback matrix, no controls have contribution
            % from more than one integral error, which makes bounding
            % easier (but code dirtier, with hard-coded dimension index)
            
            
            % use the largest non-inf value as the bound (so at least we
            % make sure this integrator bound will be sufficient for all
            % real control dimensions to reach saturation. But of course
            % this means that some control dimensions can exceed the
            % control bound on its own.
%             int_ex_bound = abs(obj.u_bound./obj.fb_matrix(:,1));
%             int_ex_bound = max(int_ex_bound(~isinf(int_ex_bound)));
%             int_ez_bound = abs(obj.u_bound./obj.fb_matrix(:,4));
%             int_ez_bound = max(int_ez_bound(~isinf(int_ez_bound)));
            
            % or just get the minimum...
            int_ex_bound = min(abs(obj.u_bound./(obj.fb_matrix(:,1)+eps)));
            int_ez_bound = min(abs(obj.u_bound./(obj.fb_matrix(:,4)+eps)));
            
            obj.int_bound = [-int_ex_bound, -int_ez_bound; ...
                              int_ex_bound,  int_ez_bound];
        end
        
    end
    
end

