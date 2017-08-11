function main()
% main Main function for running the hexapod in closedloop control
%   Brainstorming for now

%% Initialisation of all involved parts
% Some constants
time_step = 0.05; % [s]

%% Initialise the hexapod object
port_name = []; %'COM5'; % COM5
hexapod = HexapodInterface(port_name);
hexapod.cycle_time = time_step;

%% Initialise the face dector
camera_name = {'webcam', 'chameleon'};
detector = FaceDetector(camera_name{1});

% capture one frame to get size
[pos, frame] = detector.capture();

frame_size = size(frame);
pos_target = frame_size(1:2)/2;

%% Initialise the Video Player
% Create the video player object.
video_player = vision.VideoPlayer(...
    'Position', [100 100 [frame_size(2), frame_size(1)]+30]);

%% Initialise the controller
controller = HexapodController(time_step);

% set control target
controller.target = pos_target;

%% Initialise the timer
% the timer exeucation function will be a full step of detecting face 
% location, getting controller outputs, sending outputs to hexapods
t = timer;
t.StartFcn = @(~,thisEvent)disp([thisEvent.Type ' executed '...
    datestr(thisEvent.Data.time,'dd-mmm-yyyy HH:MM:SS.FFF')]);

t.TimerFcn = @(h, event) timer_func(h, event, detector, controller, hexapod, video_player);

t.StopFcn = @(h, event) stop_func(h, event, detector, controller, hexapod, video_player);

t.Period = hexapod.cycle_time;

t.TasksToExecute = 200;
t.ExecutionMode = 'fixedRate';



%% 
create_gui(t)

end

%% Helper function

% The most important function, one that is executed at ever step in the
% timer
function timer_func(h_timer, event, detector, controller, hex, video_player)
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

% convert to 3d from [x, z] to [x, y, z];
pos_hex_target = [pos_hex_target(1), 0, pos_hex_target(2)];


% 3) Send control signal to the hexapod
hex.move(pos_hex_target)

% 4) display video
step(video_player, frame);

end

% Stop fun, clean up
function stop_func(h_timer, event, detector, controller, hex, video_player)
delete(detector);
delete(controller);
delete(hex);
release(video_player);

disp([event.Type ' executed '...
    datestr(event.Data.time,'dd-mmm-yyyy HH:MM:SS.FFF')]);
end

%% GUI functions
function create_gui(timer_object)
hf = figure(104);

insertButtons(hf, timer_object)
end

function insertButtons(hf, timer_object)

% Play button with text Start/Pause/Continue
uicontrol(hf,'unit','pixel','style','pushbutton','string','Start',...
    'position',[10 10 75 25], 'tag','PBButton123','callback',...
    {@start_callback, timer_object});

% Exit button with text Exit
uicontrol(hf,'unit','pixel','style','pushbutton','string','Exit',...
    'position',[100 10 50 25],'callback', ...
    {@stop_callback, timer_object});
end

function start_callback(~, ~, timer_object)
start(timer_object);
end

function stop_callback(~, ~, timer_object)
stop(timer_object);
end
