function main()
% main Main function for running the hexapod in closedloop control
%   Brainstorming for now

%% Initialisation of all involved parts
% Some constants
time_step = 0.05; % [s]
mode = 'hybrid';

% Hexapod or offline test?
b_offline = false;
if b_offline
    port_name = [];
    camera_name = 'webcam';
else
    port_name = 'COM5';
    camera_name = 'chameleon';
end

%% Initialise the hexapod object
hexapod = HexapodInterface(port_name);
hexapod.cycle_time = time_step;

%% Initialise the face de\qctor
detector = FaceDetector(camera_name);


%% Initialise the controller
controller = HexapodController(time_step, mode);

%% Initialise the timer
% the timer exeucation function will be a full step of detecting face 
% location, getting controller outputs, sending outputs to hexapods
t = timer;

t.StartFcn = @(~,thisEvent)disp([thisEvent.Type ' executed '...
    datestr(thisEvent.Data.time,'dd-mmm-yyyy HH:MM:SS.FFF')]);


t.StopFcn = @(h, event) stop_func(h, event, detector, controller, hexapod);

t.Period = hexapod.cycle_time;

t.TasksToExecute = ceil(30/time_step);
t.ExecutionMode = 'fixedRate';

%% 
ha = create_gui(t, detector, controller);

t.TimerFcn = @(h, event) timer_func(h, event, detector, controller, hexapod, ha);



end

%% Helper function

% The most important function, one that is executed at ever step in the
% timer
function timer_func(h_timer, event, detector, controller, hex, ha)
% 1) Get location of the face
[pos_image, frame] = detector.capture();

% 2) Get control signal from controller
% control signal = target absolute position [x, y, z]

% We rely on the controller being an object to handle its own internal
% states.
%
% At the moment, we assume the target image position should be something
% that's a constant, for example, fixed to the initial positition when it
% was first detected or fixed to the centre of the frame.

pos_hex_target = controller.step(pos_image);
fprintf('error: intx=%.1f, x=%.1f, intz=%.1f, z=%.1f \n', controller.e_states(:));

% convert to 6d from [x, z] to [x, y, z, u, v, w];
% pos_hex_target = [pos_hex_target(1), 0, pos_hex_target(2), 0, 0, 0];

% convert to 6d from [u, v] to [x, y, z, u, v, w];
% pos_hex_target = [0, 0, 0, pos_hex_target(2), 0, pos_hex_target(1)];

% 3) Send control signal to the hexapod
hex.move(pos_hex_target)

% 4) display video
% Insert controller target
frame = insertMarker(frame, controller.target, 'o', 'Color', 'r', 'size', 8);
show_frame_on_axis(ha, frame);
end

% Stop fun, clean up
function stop_func(h_timer, event, detector, controller, hex)
delete(detector);
delete(controller);
delete(hex);

disp([event.Type ' executed '...
    datestr(event.Data.time,'dd-mmm-yyyy HH:MM:SS.FFF')]);
end

%% GUI functions
function ha = create_gui(timer_object, detector, controller)
hf = figure(104);
ha = axes(hf);

insertButtons(hf, ha, timer_object, detector, controller)
end

function insertButtons(hf, ha, timer_object, detector, controller)

% Initialise button 
uicontrol(hf,'unit','pixel','style','pushbutton','string','Initialise',...
    'position',[10 10 75 25], 'tag','PBButton123','callback',...
    {@init_callback, detector, controller, ha});

% Play button with text Start/Pause/Continue
uicontrol(hf,'unit','pixel','style','pushbutton','string','Start',...
    'position',[100 10 50 25], 'tag','PBButton123','callback',...
    {@start_callback, timer_object});

% Exit button with text Exit
uicontrol(hf,'unit','pixel','style','pushbutton','string','Exit',...
    'position',[210 10 50 25],'callback', ...
    {@stop_callback, timer_object, hf});
end

function init_callback(~, ~, detector, controller, ha)
% capture one frame to get the target location at the start
[pos, frame] = detector.capture();
frame = insertMarker(frame, pos, 'o', 'Color', 'r', 'size', 8);
% set control target
controller.target = pos;

showFrameOnAxis(ha, frame)
end

function start_callback(~, ~, timer_object)
start(timer_object);
end

function stop_callback(h, event, timer_object, hf)
stop(timer_object);
close(hf);
end
