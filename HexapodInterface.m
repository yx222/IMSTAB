classdef HexapodInterface < handle
    %HexapodInterface Wrapper for controlling the hexapod
    %   Detailed explanation goes here
    
    properties
        % Properties of hexapod motion limits
        % M-840
        limits = struct('x', [-50, 50]*0.9, ... % mm
                        'y', [-50, 50]*0.9, ...
                        'z', [-25, 25]*0.9, ...
                        'u', [-16, 16], ... % deg
                        'v', [-16, 16], ...
                        'w', [-34, 34]);
                    
        % Properties of the serial port
        port
        
        % Motion properties
        cycle_time = 0.1; % [s]
        
        % Other properties?

    end
    
    methods
        % Constructor
        function obj = HexapodInterface(port_name)
                % test mode
                if nargin == 0 
                    port_name = [];
                    b_test = true;
                else 
                    b_test = false;
                end
 
                % Connec tto given port
                obj.connect_port(port_name);
                
                % Initialise
                obj.initialise();
                
                % Run test if called with an empty port_name
                if b_test
                    obj.run_test();
                end
        end
        
        % Destructor
        function delete(obj)
            % reset the position
            obj.initialise();
            
            % close the serial port connection
            fclose(obj.port);
        end
        
        % Set up serial communication
        function connect_port(obj, port_name)
            % Open Port
            if ~isempty(port_name)
                % a real port is defined
                s= serial(port_name); % eg: COM4
                set(s,'BaudRate',57600);
                %s.BaudRate=115200;
                
                set(s,'Terminator','LF');
                fopen(s);
            else
                % no ports defined. use a fake one
                s = 1;
            end
            
            % assign to object property
            obj.port = s;            
        end
        
        % Initialise hexapod
        function initialise(obj)
            send_command(obj.port,'INI\n');
            
            % pause for two second
            pause(2);
            
            % Set move timing
            send_command(obj.port, 'SCT T%i\n', obj.cycle_time*1000);
        end
        
        % Move the hexapod to a specified absolute location
        % #TODO# Need some protection layer
        function move(obj, pos)
            % pos = [x, y, z, u, v, w] in [mm, mm, mm, deg, deg, deg]
            
            % limit to hardware range
            pos = obj.saturate(pos);
            
            % send command
            send_command(obj.port, 'MOV! X%.3f Y%.3f Z%.3f U%.3f V%.3f W%.3f\n', pos);
        end
        
        
        %% Helper methods
        function new_val = saturate(obj, val)
            names = {'x', 'y', 'z', 'u', 'v', 'w'};
            N_val = length(val);
            new_val = val;
            
            for ii = 1:N_val
                new_val(ii) = min(obj.limits.(names{ii})(2), ...
                    max(obj.limits.(names{ii})(1), val(ii))...
                    );
            end       
        end %  new_val = saturate(obj, val)
        
        %% Test methods
        function run_test(obj)
            N_test = 100;
            time_step = 0.1;
            period = 2;
            time = (1:N_test)*time_step;
            
            x = 10*sin(2*pi*time/period)';
            z = 5*cos(2*pi*time/period + pi/6)';

            xyzuvw = [x, x*0, z, x*0, x*0, x*0];
            
            for ii = 1:N_test
                obj.move(xyzuvw(ii, :))
                pause(0.1);
            end
            
        end
        
    end
    
end

%% Helper function
% works like an fprintf, but does it in two stages. construct and send
% Also prints it to the command line
function s = send_command(port, varargin)
s = sprintf(varargin{:});
fprintf(port, s);

% if is a serial port, then print to command line. If not, this would be
% automatically done by fprintf

% #TODO# use a nicer logger instead of printing to the command line.

if ~isa(port, 'double') && isvalid(port)
    fprintf(s);
end

end

