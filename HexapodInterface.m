classdef HexapodInterface < handle
    %HexapodInterface Wrapper for controlling the hexapod
    %   Detailed explanation goes here
    
    properties
        % Properties of hexapod motion limits
        % M-840
        limits = struct('x', [-50, 50], ... % mm
                        'y', [-50, 50], ...
                        'z', [-25, 25], ...
                        'theta_x', [-15, 15], ... % deg
                        'theta_y', [-15, 15], ...
                        'theta_z', [-30, 30]);
                    
        % M-824
%         limits = struct('x', [-22.5, 22.5], ... % mm
%                         'y', [-22.5, 22.5], ...
%                         'z', [-12.5, 12.5], ...
%                         'theta_x', [-7.5, 7.5], ... % deg
%                         'theta_y', [-7.5, 7.5], ...
%                         'theta_z', [-12.5, 12.5]);
                    
        % Properties of the serial port
        port
        
        % Motion properties
        cycle_time = 100; % [ms]
        
        % Other properties?

    end
    
    methods
        % Constructor
        function obj = HexapodInterface(port_name)
                % test mode
                if nargin == 0 || isempty(port_name)
                    obj.connect_port();
                else
                    obj.connect_port(port_name);
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
            if nargin > 1
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
            
            % Initialise
            obj.initialise();
            
        end
        
        % Initialise hexapod
        function initialise(obj)
            send_command(obj.port,'INI\n');
            
            % pause for two second
            pause(2);
            
            % Set move timing
            send_command(obj.port, 'SCT T%i\n', obj.cycle_time);
        end
        
        % Move the hexapod to a specified absolute location
        % #TODO# Need some protection layer
        function move(obj, pos)
            % pos = [x, y, z] in [mm, mm, mm]
            
            % limit to hardware range
            pos = obj.saturate(pos);
            
            % send command
            send_command(obj.port, 'MOV! X%.2f Y%.2f Z%.2f\n', pos);
        end
        
        
        %% Helper methods
        function new_val = saturate(obj, val)
            names = {'x', 'y', 'z'};
            N_val = length(val);
            new_val = val;
            
            for ii = 1:N_val
                new_val(ii) = min(obj.limits.(names{ii})(2), ...
                    max(obj.limits.(names{ii})(1), val(ii))...
                    );
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

% if isvalid(port)
%     fprintf(s);
% end

end

