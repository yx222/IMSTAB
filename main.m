function main()
% main Main function for running the hexapod in closedloop control
%   Brainstorming for now

%% Initialisation of all involved parts
% Define some constnats, names, etc
port_name = []; % COM5


% Initialise the hexapod object
hexapod = HexapodInterface(port_name);

% Initialise the face dector

% Initialise the timer - the timer exeucation function will be a full step
% of detecting face location, getting controller outputs, sending outputs
% to hexapods

%% 


end

%% Helper function

% The most important function, one that is executed at ever step in the
% timer
function timer_func(h_timer, event, detector, controller, hex)
% 1) Get location of the face
pos_image = detector.get_position();

% 2) Get control signal from controller
% control signal = target absolute position [x, y, z]

% We rely on the controller being an object to handle its own internal
% states.
%
% At the moment, we assume the target image position should be something
% that's a constant, for example, fixed to the initial positition when it
% was first detected or fixed to the centre of the frame.
pos_hex_target = controller.get_control(pos_image);


% 3) Send control signal to the hexapod
hex.move(pos_hex_target)

end
