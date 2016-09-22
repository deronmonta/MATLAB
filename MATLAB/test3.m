% Demo macro to extract frames and get frame means from an avi movie
% and save individual frames to separate image files.
% Then rebuilds a new movie by recalling the saved images from disk.
% Also computes the mean gray value of the color channels
% And detects the difference between a frame and the previous frame.
clc;    % Clear the command window.
close all;  % Close all figures (except those of imtool.)
imtool close all;  % Close all imtool figures.
clear;  % Erase all existing variables.
workspace;  % Make sure the workspace panel is showing.
fontSize = 14;

% Change the current folder to the folder of this m-file.
% (The line of code below is from Brett Shoelson of The Mathworks.)
if(~isdeployed)
	cd(fileparts(which(mfilename)));
end

% Open the rhino.avi demo movie that ships with MATLAB.
folder = fullfile(matlabroot, '\toolbox\images\imdemos');
movieFullFileName = fullfile(folder, 'rhinos.avi');
% Check to see that it exists.
if ~exist(movieFullFileName, 'file')
	strErrorMessage = sprintf('File not found:\n%s\nYou can choose a new one, or cancel', movieFullFileName);
	response = questdlg(strErrorMessage, 'File not found', 'OK - choose a new movie.', 'Cancel', 'OK - choose a new movie.');
	if strcmpi(response, 'OK - choose a new movie.')
		[baseFileName, folderName, FilterIndex] = uigetfile('*.avi');
		if ~isequal(baseFileName, 0)
			movieFullFileName = fullfile(folderName, baseFileName);
		else
			return;
		end
	else
		return;
	end
end

try
	videoObject = VideoReader(movieFullFileName)
	% Determine how many frames there are.
	numberOfFrames = videoObject.NumberOfFrames;
	vidHeight = videoObject.Height;
	vidWidth = videoObject.Width;
	
	numberOfFramesWritten = 0;
	% Prepare a figure to show the images in the upper half of the screen.
	figure;
	% 	screenSize = get(0, 'ScreenSize');
	% Enlarge figure to full screen.
	set(gcf, 'units','normalized','outerposition',[0 0 1 1]);
		
	% Loop through the movie, writing all frames out.
	% Each frame will be in a separate file with unique name.
	meanGrayLevels = zeros(numberOfFrames, 1);
	meanRedLevels = zeros(numberOfFrames, 1);
	meanGreenLevels = zeros(numberOfFrames, 1);
	meanBlueLevels = zeros(numberOfFrames, 1);
	for frame = 1 : numberOfFrames
		% Extract the frame from the movie structure.
		thisFrame = read(videoObject, frame);
		
		% Display it
		hImage = subplot(2, 2, 1);
		image(thisFrame);
		caption = sprintf('Frame %4d of %d.', frame, numberOfFrames);
		title(caption, 'FontSize', fontSize);
		drawnow; % Force it to refresh the window.
			
		% Calculate the mean gray level.
		grayImage = rgb2gray(thisFrame);
		meanGrayLevels(frame) = mean(grayImage(:));
		
		% Calculate the mean R, G, and B levels.
		meanRedLevels(frame) = mean(mean(thisFrame(:, :, 1)));
		meanGreenLevels(frame) = mean(mean(thisFrame(:, :, 2)));
		meanBlueLevels(frame) = mean(mean(thisFrame(:, :, 3)));
		
		% Plot the mean gray levels.
		hPlot = subplot(2, 2, 2);
		hold off;
		plot(meanGrayLevels, 'k-', 'LineWidth', 2);
		hold on;
		plot(meanRedLevels, 'r-');
		plot(meanGreenLevels, 'g-');
		plot(meanBlueLevels, 'b-');
		grid on;
		
		% Put title back because plot() erases the existing title.
		title('Mean Gray Levels', 'FontSize', fontSize);
		if frame == 1
			xlabel('Frame Number');
			ylabel('Gray Level');
			% Get size data later for preallocation if we read
			% the movie back in from disk.
			[rows, columns, numberOfColorChannels] = size(thisFrame);
		end
		
		% Update user with the progress.  Display in the command window.
		progressIndication = sprintf('Processed frame %4d of %d.', frame, numberOfFrames);
		disp(progressIndication);
		% Increment frame count (should eventually = numberOfFrames
		% unless an error happens).
		numberOfFramesWritten = numberOfFramesWritten + 1;
		
		% Now let's do the differencing
		alpha = 0.5;
		if frame == 1
			Background = thisFrame;
		else
			% Change background slightly at each frame
			% 			Background(t+1)=(1-alpha)*I+alpha*Background
			Background = (1-alpha)* thisFrame + alpha * Background;
		end
		% Display the changing/adapting background.
		subplot(2, 2, 3);
		imshow(Background);
		title('Adaptive Background', 'FontSize', fontSize);
		% Calculate a difference between this frame and the background.
		differenceImage = thisFrame - uint8(Background);
		% Threshold with Otsu method.
		grayImage = rgb2gray(differenceImage); % Convert to gray level
		thresholdLevel = graythresh(grayImage); % Get threshold.
		binaryImage = im2bw( grayImage, thresholdLevel); % Do the binarization
		% Get rid of small blobs
		binaryImage = bwareaopen(binaryImage, 800);
		binaryImage = imfill(binaryImage, 'holes');
		% Plot the binary image.
		subplot(2, 2, 4);
		imshow(binaryImage);
		title('Binarized Difference Image', 'FontSize', fontSize);
		
		% Label the blobs.
		labeledImage = bwlabel(binaryImage, 8);     % Label each blob so we can make measurements of it
		% Get all the blob properties.  Can only pass in originalImage in version R2008a and later.
		blobMeasurements = regionprops(labeledImage, 'Centroid');
		numberOfBlobs = size(blobMeasurements, 1);
		if numberOfBlobs >= 1
			blobMeasurementsCentroids = [blobMeasurements.Centroid];
			xCenters = blobMeasurementsCentroids(1:2:end);
			yCenters = blobMeasurementsCentroids(2:2:end);
			hold on;
			% Mark centroid with a star inside a circle.
			plot(xCenters, yCenters, 'r*', 'LineWidth', 2, 'MarkerSize', 10);
			plot(xCenters, yCenters, 'ro', 'LineWidth', 2, 'MarkerSize', 10);
			hold off;	titleBarCaption = 'Continue?';
			button = questdlg(promptMessage, titleBarCaption, 'Continue', 'Cancel', 'Continue');
			if strcmpi(button, 'Cancel')
				return;
			end
		end
		
	end
	
	% Alert user that we're done.
	finishedMessage = sprintf('Done!  It processed %d frames of\n"%s"', numberOfFramesWritten, movieFullFileName);
	disp(finishedMessage); % Write to command window.
	uiwait(msgbox(finishedMessage)); % Also pop up a message box.
	
catch ME
	% Some error happened if you get here.
	strErrorMessage = sprintf('Error extracting movie frames from:\n\n%s\n\nError: %s\n\n)', movieFullFileName, ME.message);
	uiwait(msgbox(strErrorMessage));
end
