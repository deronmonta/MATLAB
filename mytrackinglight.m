
%% Motion-Based Multiple Object Tracking
% This example shows how to perform automatic detection and motion-based
% tracking of moving objects in a video from a stationary camera.
%
%   Copyright 2014 The MathWorks, Inc.

%%
% Detection of moving objects and motion-based tracking are important 
% components of many computer vision applications, including activity
% recognition, traffic monitoring, and automotive safety.  The problem of
% motion-based object tracking can be divided into two parts:
%
% # detecting moving objects in each frame 
% # associating the detections corresponding to the same object over time
%
% The detection of moving objects uses a background subtraction algorithm
% based on Gaussian mixture models. Morphological operations are applied to
% the resulting foreground mask to eliminate noise. Finally, blob analysis
% detects groups of connected pixels, which are likely to correspond to
% moving objects. 
%
% The association of detections to the same object is based solely on
% motion. The motion of each track is estimated by a Kalman filter. The
% filter is used to predict the track's location in each frame, and
% determine the likelihood of each detection being assigned to each 
% track.
%
% Track maintenance becomes an important aspect of this example. In any
% given frame, some detections may be assigned to tracks, while other
% detections and tracks may remain unassigned.The assigned tracks are
% updated using the corresponding detections. The unassigned tracks are 
% marked invisible. An unassigned detection begins a new track. 
%
% Each track keeps count of the number of consecutive frames, where it
% remained unassigned. If the count exceeds a specified threshold, the
% example assumes that the object left the field of view and it deletes the
% track.  
%
% For more information please see
% <matlab:helpview(fullfile(docroot,'toolbox','vision','vision.map'),'multipleObjectTracking') Multiple Object Tracking>.
%
% This example is a function with the main body at the top and helper 
% routines in the form of 
% <matlab:helpview(fullfile(docroot,'toolbox','matlab','matlab_prog','matlab_prog.map'),'nested_functions') nested functions> 
% below.

function tracks =  mytrackinglight()

% Create System objects used for reading video, detecting moving objects,
% and displaying the results.

prompt = 'Enter number of plate\n';
plate = input(prompt, 's');




obj = setupSystemObjects();

tracks = initializeTracks(); % Create an empty array of tracks.

nextId = 1; % ID of the next track

disp('Total number of frames');
disp(obj.reader2.NumberOfFrames);

% Detect moving objects, and track them across video frames.
%while ~isDone(obj.reader)

  for i = 1: obj.reader2.NumberOfFrames;



    frame = readFrame(plate);
    
    [centroids, bboxes, mask] = detectObjects(frame);
    predictNewLocationsOfTracks();
    [assignments, unassignedTracks, unassignedDetections] = ...
        detectionToTrackAssignment();
    
    if isempty(tracks)%Only create tracks when it is empty, therefore only creating one
        
        
         createNewTracks();
         
    end
   
    updateAssignedTracks();
    updateUnassignedTracks();
%     deleteLostTracks();
%    
      displayTrackingResults();
       flyanalysis();
       %save('MVI_0561-D','tracks');
    
end


%% Create System Objects
% Create System objects used for reading the video frames, detecting
% foreground objects, and displaying results.

    function obj = setupSystemObjects()
        % Initialize Video I/O
        % Create objects for reading a video from a file, drawing the tracked
        % objects in each frame, and playing the video.
        
        % Create a video file reader.
        obj.reader = vision.VideoFileReader('MVI_0561.mp4');% Input video file name here
        obj.reader2 = VideoReader('MVI_0561.mp4');
        totalframe = obj.reader2.NumberOfFrames;
      
        % Create two video players, one to display the video,
        % and one to display the foreground mask.
        obj.videoPlayer = vision.VideoPlayer('Position', [20, 200, 600, 600]);
        obj.maskPlayer = vision.VideoPlayer('Position', [740, 200, 600, 600]);
        
        % Create System objects for foreground detection and blob analysis
        
        % The foreground detector is used to segment moving objects from
        % the background. It outputs a binary mask, where the pixel value
        % of 1 corresponds to the foreground and the value of 0 corresponds
        % to the background. 
        
        obj.detector = vision.ForegroundDetector('NumGaussians', 3, ...
            'NumTrainingFrames', 200, 'MinimumBackgroundRatio', 0.4,'LearningRate',0.005);
        
        % Connected groups of foreground pixels are likely to correspond to moving
        % objects.  The blob analysis System object is used to find such groups
        % (called 'blobs' or 'connected components'), and compute their
        % characteristics, such as area, centroid, and the bounding box.
        
        obj.blobAnalyser = vision.BlobAnalysis('BoundingBoxOutputPort', true, ...
            'AreaOutputPort', true, 'CentroidOutputPort', true, 'LabelMatrixOutputPort', true , 'MinimumBlobArea', 10, 'ExtentOutputPort',true ,'MaximumCount', 1, ...
             'MajorAxisLengthOutputPort',true,'MinorAxisLengthOutputPort',true,'OrientationOutputPort',true);
            
         
        
        
               
    end
    

    

%% Initialize Tracks
% The |initializeTracks| function creates an array of tracks, where each
% track is a structure representing a moving object in the video. The
% purpose of the structure is to maintain the state of a tracked object.
% The state consists of information used for detection to track assignment,
% track termination, and display. 
%
% The structure contains the following fields:
%
% * |id| :                  the integer ID of the track
% * |bbox| :                the current bounding box of the object; used
%                           for display
% * |kalmanFilter| :        a Kalman filter object used for motion-based
%                           tracking
% * |age| :                 the number of frames since the track was first
%                           detected
% * |totalVisibleCount| :   the total number of frames in which the track
%                           was detected (visible)
% * |consecutiveInvisibleCount| : the number of consecutive frames for 
%                                  which the track was not detected (invisible).
%
% Noisy detections tend to result in short-lived tracks. For this reason,
% the example only displays an object after it was tracked for some number
% of frames. This happens when |totalVisibleCount| exceeds a specified 
% threshold.    
%
% When no detections are associated with a track for several consecutive
% frames, the example assumes that the object has left the field of view 
% and deletes the track. This happens when |consecutiveInvisibleCount|
% exceeds a specified threshold. A track may also get deleted as noise if 
% it was tracked for a short time, and marked invisible for most of the of 
% the frames.        

    function tracks = initializeTracks()
        
       
        % create an empty array of tracks
        % Tracks obj includes various features such as the following
        tracks = struct(...   
             'id', {}, ...
             'bbox', {}, ...
             'kalmanFilter', {}, ...
             'age', {}, ...
             'totalVisibleCount', {}, ...
             'consecutiveInvisibleCount', {} ,...
             'active',{}, ...
              'wholepos',{},...
             'dis2center',{},...
             'speed',{},...
             'angle',{},...
             'zonetime',{},...
             'totalframe',{});
       
             
            
    end

%% Read a Video Frame
% Read the next video frame from the video file.
% Aplly video processing alogrithm  here
    function frame = readFrame(plate)


        frame = obj.reader.step;

        frame = im2uint8(frame);
        frame = rgb2gray(frame);

        if (plate =='1')
        frame = imcrop(frame,[425 0 500 500]);% This is the upper left corner plate
      elseif (plate =='2')
         frame = imcrop(frame, [950, 0, 500, 500]); % This is the upper right corner plate
      elseif (plate =='3')
         frame = imcrop(frame, [450, 500, 500, 500]); % This is the lower right corner plate
      elseif (plate =='4')
        frame = imcrop(frame, [950, 500, 500, 500]); % This is the lower right corner plate
       end
          
          
       
       
        
        frame = imsharpen(frame,'Amount', 2); % This sharpen the frames for further processing 
        %frame = fspecial('laplacian',0);
        %frame = imadd(frame,50);

        
        
    end

%% Detect Objects
% The |detectObjects| function returns the centroids and the bounding boxes
% of the detected objects. It also returns the binary mask, which has the 
% same size as the input frame. Pixels with a value of 1 correspond to the
% foreground, and pixels with a value of 0 correspond to the background.   
%
% The function performs motion segmentation using the foreground detector. 
% It then performs morphological operations on the resulting binary mask to
% remove noisy pixels and to fill the holes in the remaining blobs.  

    
        
    function [centroids, bboxes, mask] = detectObjects(frame)
        
        % Detect foreground.
        mask = obj.detector.step(frame);
        
        % Apply morphological operations to remove noise and fill in holes.
        
        mask = imopen(mask, strel('square',4));

        mask = imclose(mask, strel('rectangle', [3,3])); 
        mask = imfill(mask, 'holes');
        
         [ ~,centroids, bboxes ] = obj.blobAnalyser.step(mask);  % Perform blob analysis to find connected components.
         
         
        %Plotting the head and tail according to the max distance 
%         [rows, columns] = size(mask); 
%         axis on;
%         title('Binary Image');
%         set(gcf, 'Units', 'Normalized');
%         [labeledImage, numberOfRegions] = bwlabel(mask);
%        
%         
%         axis on;
%         hold on;
%          
%         measurements = regionprops(labeledImage, 'MajorAxisLength', 'Orientation', 'Centroid', 'Area');
%        
%         boundaries = bwboundaries(mask);
%         numberOfBoundaries = size(boundaries, 1);
%         
%         for blobIndex = 1 : numberOfBoundaries
% 	thisBoundary = boundaries{blobIndex};
% 	x = thisBoundary(:, 2); % x = columns.
% 	y = thisBoundary(:, 1); % y = rows.
% 	
% 	% Find which two bounary points are farthest from each other.
% 	maxDistance = -inf;
% 	for k = 1 : length(x)
%         axis([0 1000 0 1000]);
% 		distances = sqrt( (x(k) - x) .^ 2 + (y(k) - y) .^ 2 );
% 		[thisMaxDistance, indexOfMaxDistance] = max(distances);
% 		if thisMaxDistance > maxDistance
% 			maxDistance = thisMaxDistance;
% 			index1 = k;
% 			index2 = indexOfMaxDistance;
%             fprintf('%f\n',maxDistance);
%             
%             
%             %Find the mid point
%             
%             xMidPoint = mean([x(index1), x(index2)]);
%             yMidPoint = mean([y(index1), y(index2)]);
%             longSlope = (y(index1) - y(index2)) / (x(index1) - x(index2));
%             perpendicularSlope = -1/longSlope;
%             % Use point slope formula (y-ym) = slope * (x - xm) to get points
%             y1 = perpendicularSlope * (1 - xMidPoint) + yMidPoint;
%             y2 = perpendicularSlope * (columns - xMidPoint) + yMidPoint;
% 	
%             % Get the profile perpendicular to the midpoint so we can find out when if first enters and last leaves the object.
%             [cx,cy,c] = improfile(mask,[1, columns], [y1, y2], 1000);
%             % Get rid of NAN's that occur when the line's endpoints go above or below the image.
%             c(isnan(c)) = 0;
%             firstIndex = find(c, 1, 'first');
%             lastIndex = find(c, 1, 'last');
%             % Compute the distance of that perpendicular width.
%             perpendicularWidth = sqrt( (cx(firstIndex) - cx(lastIndex)) .^ 2 + (cy(firstIndex) - cy(lastIndex)) .^ 2 );
%             % Get the average perpendicular width.  This will approximately be the area divided by the longest length.
%             averageWidth = measurements(blobIndex).Area / maxDistance;
% 
%             % Plot the boundaries, line, and midpoints over the two images.
%             % Plot the boundary over the gray scale image
%            
%             % For this blob, put a line between the points farthest away from each other.
%             line([x(index1), x(index2)], [y(index1), y(index2)], 'Color', 'r', 'LineWidth', 3);
%             % Plot perpendicular line.  Make it green across the whole image but magenta inside the blob.
%             line([1, columns], [y1, y2], 'Color', 'g', 'LineWidth', 3);	
%             line([cx(firstIndex), cx(lastIndex)], [cy(firstIndex), cy(lastIndex)], 'Color', 'm', 'LineWidth', 3);
% 
%           % First for point 1
%          plot(x(index1), y(index1), 'y+');
%          boxWidth = 60; % half Width
%         % Draw the box
%         xb1 = x(index1) - boxWidth;
%         xb2 = x(index1) + boxWidth;
%         yb1 = y(index1) - boxWidth;
%         yb2 = y(index1) + boxWidth;
%         % Make sure all coordinates are in side the image.
%         xb1 = min([xb1, columns]);
%         xb1 = max([xb1, 1]);
%         xb2 = min([xb2, columns]);
%         xb2 = max([xb2, 1]);
%         yb1 = min([yb1, rows]);
%         yb1 = max([yb1, 1]);
%         yb2 = min([yb2, rows]);
%         yb2 = max([yb2, 1]);
%         % Draw the box.
%         plot([xb1, xb2, xb2, xb1, xb1], [yb1, yb1, yb2, yb2, yb1], 'y-');
%         subImage1 = mask(yb1:yb2, xb1:xb2);
%         numWhitePixels1 = sum(subImage1(:));
% 
%         % Next for point 2
%         plot(x(index2), y(index2), 'g+');
%         % Draw the box
%         xb1 = x(index2) - boxWidth;
%         xb2 = x(index2) + boxWidth;
%         yb1 = y(index2) - boxWidth;
%         yb2 = y(index2) + boxWidth;
%         % Make sure all coordinates are in side the image.
%         xb1 = min([xb1, columns]);
%         xb1 = max([xb1, 1]);
%         xb2 = min([xb2, columns]);
%         xb2 = max([xb2, 1]);
%         yb1 = min([yb1, rows]);
%         yb1 = max([yb1, 1]);
%         yb2 = min([yb2, rows]);
%         yb2 = max([yb2, 1]);
%         % Draw the box.
%         plot([xb1, xb2, xb2, xb1, xb1], [yb1, yb1, yb2, yb2, yb1], 'y-');
%         subImage2 = mask(yb1:yb2, xb1:xb2);
%         numWhitePixels2 = sum(subImage2(:));
% 
%         % Put a label at the head
%          plot(x, y, 'r-', 'LineWidth', 1);
%          text(x(index2), y(index2), 'Head',  'Color', 'r');	
%             text(x(index1), y(index1), 'Tail',  'Color', 'r');
%             
%               set(gca,'YDir','Reverse');
%             
%     
%         end
%     end
%                 
%             hold off;
%         end
        
                   
            
        end

%% Predict New Locations of Existing Tracks
% Use the Kalman filter to predict the centroid of each track in the
% current frame, and update its bounding box accordingly.

    function predictNewLocationsOfTracks()
        for i = 1:length(tracks)
            bbox = tracks(i).bbox;
            
            % Predict the current location of the track.
            predictedCentroid = predict(tracks(i).kalmanFilter);
            
            % Shift the bounding box so that its center is at 
            % the predicted location.
            predictedCentroid = int32(predictedCentroid) - bbox(3:4) / 2;
            tracks(i).bbox = [predictedCentroid, bbox(3:4)];
        end
    end

%% Assign Detections to Tracks
% Assigning object detections in the current frame to existing tracks is
% done by minimizing cost. The cost is defined as the negative
% log-likelihood of a detection corresponding to a track.  
%
% The algorithm involves two steps: 
%
% Step 1: Compute the cost of assigning every detection to each track using
% the |distance| method of the |vision.KalmanFilter| System object(TM). The 
% cost takes into account the Euclidean distance between the predicted
% centroid of the track and the centroid of the detection. It also includes
% the confidence of the prediction, which is maintained by the Kalman
% filter. The results are stored in an MxN matrix, where M is the number of
% tracks, and N is the number of detections.   
%
% Step 2: Solve the assignment problem represented by the cost matrix using
% the |assignDetectionsToTracks| function. The function takes the cost 
% matrix and the cost of not assigning any detections to a track.  
%
% The value for the cost of not assigning a detection to a track depends on
% the range of values returned by the |distance| method of the 
% |vision.KalmanFilter|. This value must be tuned experimentally. Setting 
% it too low increases the likelihood of creating a new track, and may
% result in track fragmentation. Setting it too high may result in a single 
% track corresponding to a series of separate moving objects.   
%
% The |assignDetectionsToTracks| function uses the Munkres' version of the
% Hungarian algorithm to compute an assignment which minimizes the total
% cost. It returns an M x 2 matrix containing the corresponding indices of
% assigned tracks and detections in its two columns. It also returns the
% indices of tracks and detections that remained unassigned. 

    function [assignments, unassignedTracks, unassignedDetections] = ...
            detectionToTrackAssignment()
        
        nTracks = length(tracks);
        nDetections = size(centroids, 1);
        
        % Compute the cost of assigning each detection to each track.
        cost = zeros(nTracks, nDetections);
        for i = 1:nTracks
            cost(i, :) = distance(tracks(i).kalmanFilter, centroids);
        end
        
        % Solve the assignment problem.
        costOfNonAssignment = 20;
        [assignments, unassignedTracks, unassignedDetections] = ...
            assignDetectionsToTracks(cost, costOfNonAssignment);
    end

%% Update Assigned Tracks
% The |updateAssignedTracks| function updates each assigned track with the
% corresponding detection. It calls the |correct| method of
% |vision.KalmanFilter| to correct the location estimate. Next, it stores
% the new bounding box, and increases the age of the track and the total
% visible count by 1. Finally, the function sets the invisible count to 0. 

    function updateAssignedTracks()
        numAssignedTracks = size(assignments, 1);
        for i = 1:numAssignedTracks
            trackIdx = assignments(i, 1);
            detectionIdx = assignments(i, 2);
            centroid = centroids(detectionIdx, :);
            bbox = bboxes(detectionIdx, :);
            
            % Correct the estimate of the object's location
            % using the new detection.
            correct(tracks(trackIdx).kalmanFilter, centroid);
            
            % Replace predicted bounding box with detected
            % bounding box.
            tracks(trackIdx).bbox = bbox;
            
            % Update track's age.
            tracks(trackIdx).age = tracks(trackIdx).age + 1;
       
            
            % Update visibility.
            tracks(trackIdx).totalVisibleCount = ...
                tracks(trackIdx).totalVisibleCount + 1;
            tracks(trackIdx).consecutiveInvisibleCount = 0;
        end
    end

%% Update Unassigned Tracks
% Mark each unassigned track as invisible, and increase its age by 1.

    function updateUnassignedTracks()
        for i = 1:length(unassignedTracks)
            ind = unassignedTracks(i);
            tracks(ind).age = tracks(ind).age + 1;
            tracks(ind).consecutiveInvisibleCount = ...
                tracks(ind).consecutiveInvisibleCount + 1;             
        end
    end

%% Delete Lost Tracks
% The |deleteLostTracks| function deletes tracks that have been invisible
% for too many consecutive frames. It also deletes recently created tracks
% that have been invisible for too many frames overall.     

%     function deleteLostTracks()
%         if isempty(tracks)
%             return;
%         end
%         
%                ageThreshold = 100;
%         
%         % Compute the fraction of the track's age for which it was visible.
%         ages = [tracks(:).age];
%         totalVisibleCounts = [tracks(:).totalVisibleCount];
%         visibility = totalVisibleCounts ./ ages;
%         
%         % Find the indices of 'lost' tracks.
%         
%          lostInds = find(ages < ageThreshold & visibility < 0.9);
%             for ii=1:length(lostInds)
%                 tracks(lostInds(ii)).active = false;
%            end
%        
%     end

%% Create New Tracks
% Create new tracks from unassigned detections. Assume that any unassigned
% detection is a start of a new track. In practice, you can use other cues
% to eliminate noisy detections, such as size, location, or appearance.

    function createNewTracks()
        
     
        
        
        centroids = centroids(unassignedDetections, :);
        bboxes = bboxes(unassignedDetections, :);
        
         for i = 1:size(centroids, 1)
            
            centroid = centroids(i,:);
            bbox = bboxes(i, :);
        
            
            % Create a Kalman filter object.
            kalmanFilter = configureKalmanFilter('ConstantVelocity', ...
                centroid, [200, 50], [100, 25], 500);
            
            timeline = animatedline('color','r');
             subplot(3,2,3);
             title('Track results');
              set(gca,'YDir','Reverse');
               
                        wholepos =[];
                 
           
            % Create a new track.
            newTrack = struct(...
                'id', nextId, ...
                'bbox', bbox, ...
                'kalmanFilter', kalmanFilter, ...
                'age', 1, ...
                'totalVisibleCount', 1, ...
                'consecutiveInvisibleCount', 0,...
                 'active',true, ...
                 'wholepos',[],... % This is an array that contains the position of the fly after it is being tracked
                 'dis2center',[],... %This is the distance of the fly to the food
                 'speed',[],...
                 'angle',[],...
                 'zonetime',timeline,...
                 'totalframe',obj.reader2.NumberOfFrames);
                 
                    
                                        
           
            % Add it to the array of tracks.
            tracks(end + 1) = newTrack;
            
            % Increment the next id.
            nextId = nextId + 1;
          
          end       
                
                            
%             set(gca,'YDir','Reverse');
  
    end

          
            
    
%% Display Tracking Results
% The |displayTrackingResults| function draws a bounding box and label ID 
% for each track on the video frame and the foreground mask. It then 
% displays the frame and the mask in their respective video players. 

    function displayTrackingResults()
        % Convert the frame and the mask to uint8 RGB.
        frame = im2uint8(frame);
        mask = uint8(repmat(mask, [1, 1, 3])) .* 255;
          frame = insertShape(frame, 'circle', [260 250 1], 'LineWidth', 5); 
          
          
          
        minVisibleCount = 20;
        

               
                
        if ~isempty(tracks)
              
            % Noisy detections tend to result in short-lived tracks.
            % Only display tracks that have been visible for more than 
            % a minimum number of frames.
            reliableTrackInds = ...
                [tracks(:).totalVisibleCount] > minVisibleCount & [tracks(:).active];
            reliableTracks = tracks(reliableTrackInds);
            
            
            % Display the objects. If an object has not been detected
            % in this frame, display its predicted bounding box.
            if ~isempty(reliableTracks)
                % Get bounding boxes.
                bboxes = cat(1, reliableTracks.bbox); % 
                
                % Get ids.
                ids = int32([reliableTracks(:).id]);
                
                % Create labels for objects indicating the ones for 
                % which we display the predicted rather than the actual 
                % location.
                labels = cellstr(int2str(ids'));
                predictedTrackInds = ...
                [reliableTracks(:).consecutiveInvisibleCount] > 0;
                isPredicted = cell(size(labels));
                isPredicted(predictedTrackInds) = {'predicted'};
                labels = strcat(labels, isPredicted);
                
                % Draw the objects on the frame.
                frame = insertObjectAnnotation(frame, 'rectangle',  bboxes, labels,'Color','cyan');
                
                % Draw the objects on the mask.
                mask = insertObjectAnnotation(mask, 'rectangle', bboxes, labels);
                     
                                            
               % Extracting centroid position 
               x = double(bboxes(:,1)+bboxes(:,3)/2);
                y = double(bboxes(:,2)+bboxes(:,4)/2);
                
                
                  
                    
                
                 
                for  ii=1:length(reliableTracks)
                    
              
                trackage = tracks(:).age;     
                                  
                            
                end
                                
                                                                                                                           
                   
              
                
                  
                  
                  
                
        
            end
                     
       
       
        
               
                   
        % Display the mask and the frame.
        obj.maskPlayer.step(mask);        
        obj.videoPlayer.step(frame);
             
         
    end
    end
    
  
                   
                   

            
    function flyanalysis()
        

             
         minVisibleCount = 5;
       if ~isempty(tracks)
              
         
            reliableTrackInds = ...
                [tracks(:).totalVisibleCount] > minVisibleCount & [tracks(:).active];
            reliableTracks = tracks(reliableTrackInds);
            
            
            % Display the objects. If an object has not been detected
            % in this frame, display its predicted bounding box.
            if ~isempty(reliableTracks)
                % Get bounding boxes.
                bboxes = cat(1, reliableTracks.bbox); % 
                
                                                 
               % Extracting centroid position 
               x = double(bboxes(:,1)+bboxes(:,3)/2);
                y = double(bboxes(:,2)+bboxes(:,4)/2);
                
                 
                
                   trackage = tracks(:).age;     
                   %fprintf('%d\n',tracks(:).age);
                   
                    newpos = [x y];%new position of the track
                    tracks(1).wholepos = cat(1, tracks(1).wholepos, newpos);%add the new pos to the existing array of pos
%                     disp(tracks(1).wholepos); %Display the results
                    
                     center = [260 250];% This is the center of the dish(or the location of the food)here
                                    
                     
                    
                      newdis = sqrt(sum((newpos(1,:)-center(1,:)).^2)); %the distance from fly to food
                     tracks(1).dis2center = cat(1, tracks(1).dis2center, newdis);% add it to the array
                     
                     
                    
                    
                     
                     
                     if newdis < 200
                        
%                           tracks.zonetime = tracks.zonetime + 1;
                         
                   
                     end
                     
                     
                 
                     %disp(tracks.zonetime)
                     
                     
                     lastpos = tracks(1).wholepos(end,:);% Find the last element of the wholepos array
                     
                                          
                         
                     %fprintf('last element of array:');
%                       disp(lastpos);
                      
                      
                        if size(tracks(1).wholepos,1) > 2
                        %fprintf('Second last element of array:');
                        
                        secondlast = tracks(1).wholepos(end-1,:);
%                         disp(secondlast);%Display second last pos
                        
                         speednow = sqrt(sum((lastpos(1,:)-secondlast(1,:)).^2));
                      tracks(1).speed = cat(1,tracks(1).speed,speednow);
                      %fprintf('Speed:');
%                       disp(tracks(1).speed(end));% Display speed
                      
                      
                      tenth = []; % tenth element 
                      secondtenth = [];
                      flyvec = [];
                      
                       
                      if size(tracks(1).wholepos,1) > 10
                          tenth = tracks(1).wholepos(1:10:end,:);
                          
                          if size(tracks(1).wholepos,1) >  2
                              
                              secondtenth = tenth(end-1,:);
                             
                              
                              %fprintf('secondtenth:');
%                              disp(secondtenth);
                                
                          flyvec = tenth(end,:) - secondtenth;
                              
                          %fprintf('flyvec');
                          
                           %disp(flyvec);
                          
                         vec2center = secondtenth(1,1) - [890 351];
                              
                         
%%                           subplot(3,2,4); 
%%                            title('Angle');
                     
                     
%                      plot(tracks(1).angle); %%
%                          quiver(flyvec(end,:),flyvec());%%
                         
%                         disp(vec2center);%%
                          
%%                       CosTheta = dot(flyvec,vec2center)/(norm(flyvec)*norm(vec2center));% Angle between fly vector and the food 
%%                         ThetaInDegrees = acosd(CosTheta);
                                 %fprintf('angle');
 
%%                                 tracks(1).angle(end+1,:) = ThetaInDegrees;
%                                  disp(tracks(1).angle);%%
                       

                          end
                    
                      end
                      
                        
                                                                 
                  
                        end
       end
                                        
                      hold on
                    
%%                      subplot(3,2,5);
%%                     plot(tracks.age,tracks(1).zonetime);
%%                    title('Zone time');
                      
                     subplot(3,2,2);
                     plot(tracks(1).dis2center);
                     title('Distance to Src');
                     
                     
                      subplot(3,2,1);
                       plot(tracks(1).speed);
                         title('Speed');
                         
                         
                   
                    
                      hold off 
                   
                     
                  
            end
            end
    
               
        
    








%% Summary
% This example created a motion-based system for detecting and
% tracking multiple moving objects. Try using a different video to see if
% you are able to detect and track objects. Try modifying the parameters
% for the detection, assignment, and deletion steps.  
%
% The tracking in this example was solely based on motion with the
% assumption that all objects move in a straight line with constant speed.
% When the motion of an object significantly deviates from this model, the
% example may produce tracking errors. Notice the mistake in tracking the
% person labeled #12, when he is occluded by the tree. 
%
% The likelihood of tracking errors can be reduced by using a more complex
% motion model, such as constant acceleration, or by using multiple Kalman
% filters for every object. Also, you can incorporate other cues for
% associating detections over time, such as size, shape, and color. 


%             
% 		


   
end




