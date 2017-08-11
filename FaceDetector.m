classdef FaceDetector < handle
    %FaceDetector wrapper for some quick and dirty face detection
    %   Detailed explanation goes here
    
    properties
        camera_spec = struct('webcam', struct('adaptor', {'winvideo'}, ...
                                         'input', {'MJPG_320x240'}), ...
                             'chameleon', struct('adaptor', {'pointgrey'}, ...
                                         'input', {'F7_Mono12_512x384_Mode5'})...
                              );
        camera_name = 'webcam';
        N_test_frame = 400
        
        % other objects
        vid
        detector
        tracker
        
        % dynamic properties
        frame_size
        
        % tracking properties
        num_pts = 0 % number of identified points
        tracking_pts  % the tracking points
        bbox_points   % bounding box points
        
    end
    
    methods
        % Constructor
        function obj = FaceDetector(camera_name)
            if nargin == 0
                b_test = true;
            else
                b_test = false;
                obj.camera_name = camera_name;
            end
            
            % reset devices
            imaqreset()
            
            % Create the face detector object.
            obj.detector = vision.CascadeObjectDetector();
            
            % Create the point tracker object.
            obj.tracker = vision.PointTracker('MaxBidirectionalError', 2);
            
            % Create video input
            obj.get_vid();
            
            % start the video input
            start(obj.vid);
            
            % Capture one frame to get its size.
            obj.frame_size = size(obj.get_im_gray());
            
            % Run test mode
            if b_test
                obj.run_test();
            end
            
        end
        
        % Destructor
        function delete(obj)
            % Clean up.
            delete(obj.vid);
            release(obj.tracker);
            release(obj.detector);
            
            fprintf('all resources released upon destruction \n');
        end
        
        %% External Methods
        
        % Method to get the position of the face
        function [pos, frame] = capture(obj)          
            % Get the next frame.
            frame = obj.get_im_gray();
            
            
            if obj.num_pts < 10
                % Detection mode.
                bbox = obj.detector.step(frame);
                
                if ~isempty(bbox)
                    % Find corner points inside the detected region.
                    % ROI - Region of Interest?
                    points = detectMinEigenFeatures(frame, 'ROI', bbox(1, :));
                    
                    % Re-initialize the point tracker.
                    obj.tracking_pts = points.Location;
                    obj.num_pts = size(obj.tracking_pts,1);
                    release(obj.tracker);
                    initialize(obj.tracker, obj.tracking_pts, frame);
                                        
                    % Convert the rectangle represented as [x, y, w, h] into an
                    % M-by-2 matrix of [x,y] coordinates of the four corners. This
                    % is needed to be able to transform the bounding box to display
                    % the orientation of the face.
                    obj.bbox_points = bbox2points(bbox(1, :));
                    
                    % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
                    % format required by insertShape.
                    bboxPolygon = reshape(obj.bbox_points', 1, []);
                    
                    % Display a bounding box around the detected face.
                    frame = insertShape(frame, 'Polygon', bboxPolygon, 'LineWidth', 3);
                    
                    % Display detected corners.
                    frame = insertMarker(frame, obj.tracking_pts, '+', 'Color', 'white');
                end
                
            else
                % Tracking mode.
                [xyPoints, isFound] = step(obj.tracker, frame);
                visiblePoints = xyPoints(isFound, :);
                oldInliers = obj.tracking_pts(isFound, :);
                
                obj.num_pts = size(visiblePoints, 1);
                
                if obj.num_pts >= 10
                    % Estimate the geometric transformation between the old points
                    % and the new points.
                    [xform, oldInliers, visiblePoints] = estimateGeometricTransform(...
                        oldInliers, visiblePoints, 'similarity', 'MaxDistance', 4);
                    
                    % Apply the transformation to the bounding box.
                    obj.bbox_points = transformPointsForward(xform, obj.bbox_points);
                    
                    % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
                    % format required by insertShape.
                    bboxPolygon = reshape(obj.bbox_points', 1, []);
                    
                    % Display a bounding box around the face being tracked.
                    frame = insertShape(frame, 'Polygon', bboxPolygon, 'LineWidth', 3);
                    
                    % Display tracked points.
                    frame = insertMarker(frame, visiblePoints, '+', 'Color', 'white');
                    
                    % Reset the points.
                    obj.tracking_pts = visiblePoints;
                    setPoints(obj.tracker, obj.tracking_pts);
                end
            end
            
            % Make sure the format of frame doesn't change betwen grayscale
            % and rgb
            if length(size(frame))==2
                % convert to rgb
                frame = gray2rgb(frame);
            end
            
            if ~isempty(obj.bbox_points)
                pos = mean(obj.bbox_points);
            else
                pos = [0, 0];
            end
        end
        
        %% Internal Methods
        % Function to get video input
        function get_vid(obj)
            adaptor = obj.camera_spec.(obj.camera_name).adaptor;
            input = obj.camera_spec.(obj.camera_name).input;
            obj.vid = videoinput(adaptor, 1, input);
            
            % Set the trigger to manual: only starts when asked so
            triggerconfig(obj.vid, 'manual');
        end
        
        % Function to get grayscale image
        function im = get_im_gray(obj)
            switch obj.camera_name
                case 'webcam'
                    im = rgb2gray(getsnapshot(obj.vid));
                case'chameleon'
                    im = uint8(255*mat2gray(getsnapshot(obj.vid)));
            end
            
        end
        
        %% Methods for testing
        function run_test(obj)
            % Create the video player object.
            videoPlayer = vision.VideoPlayer(...
                'Position', [100 100 [obj.frame_size(2), obj.frame_size(1)]+30]);
            
            % Do a tracking demo
            % Figure to record the trajectory
            hf = figure(104); clf; ha = axes(hf);
            hold on
            
            runLoop = true;
            obj.num_pts = 0;
            frame_count = 0;
            N_frame = obj.N_test_frame;
            
            trajectory = zeros(N_frame, 2);
            
            while runLoop && frame_count < N_frame
                tic();
                frame_count = frame_count + 1;
                [pos, frame] = obj.capture();
                                
                % Display a centre point for tracking
                frame = insertMarker(frame, pos, 'x', 'Color', 'r', ...
                    'size', 8);
                plot(ha, pos(1), pos(2), 'bx')
                
                % collect traecotry
                trajectory(frame_count, :) = pos;
                
                % Display the annotated video frame using the video player object.
                step(videoPlayer, frame);
                
                % Check whether the video player window has been closed.
                runLoop = isOpen(videoPlayer);
                
                toc()
            end % while
            
            % release resource
            release(videoPlayer);
        end
    end
    
end


%% Helper functions
function rgb = gray2rgb(gray)
rgb = cat(3, gray, gray, gray);
end
