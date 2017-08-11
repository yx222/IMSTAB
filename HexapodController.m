classdef HexapodController < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Need to define the inputs and outputs of the controller
        
        % Controller properties
        gain = [0.001, 0.2, 0]; % [i, p, d]
        time_step           % [s]
        
        N_dim = 2;          % target dimension
        
        % states and memories?
        e_states            %[integral error; error], 2*N_dim
        target              % control target
        
    end
    
    methods
        % Constructor
        function obj = HexapodController(time_step)
            if nargin == 0
                obj.time_step = 0.1;
            else
                obj.time_step = time_step;
            end
            
            % Initialise contorller states
            obj.e_states = zeros(2, obj.N_dim);
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
            
            % Compute control signal            
            ipd = [obj.e_states; e_der]; % 3*N_dim
            u = obj.gain*ipd; % 1*N_dim      
            
            % #TODO# We need to invert the x axis
            u(1) = -u(1);
        end
        
    end
    
end

