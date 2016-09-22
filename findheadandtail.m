clc;    % Clear the command window.
close all;  % Close all figures (except those of imtool.)
imtool close all;  % Close all imtool figures if you have the Image Processing Toolbox.
clear;  % Erase all existing variables. Or clearvars if you want.
workspace;  % Make sure the workspace panel is showing.
format long g;
format compact;
fontSize = 22;

%===============================================================================
% Read in a standard MATLAB color demo image.
folder = 'C:\Users\Tommy\Documents\Temporary';
baseFileName = 'pic1.png';
% Get the full filename, with path prepended.
fullFileName = fullfile(folder, baseFileName);
if ~exist(fullFileName, 'file')
	% Didn't find it there.  Check the search path for it.
	fullFileName = baseFileName; % No path this time.
	if ~exist(fullFileName, 'file')
		% Still didn't find it.  Alert user.
		errorMessage = sprintf('Error: %s does not exist.', fullFileName);
		uiwait(warndlg(errorMessage));
		return;
	end
end
rgbImage = imread(fullFileName);
grayImage = rgb2gray(rgbImage);
binaryImage = grayImage > 128;
[rows, columns] = size(binaryImage);
% subplot(2, 2, 1); 
imshow(binaryImage);
axis on;
title('Binary Image', 'fontSize', fontSize);
% Enlarge figure to full screen.
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);

% Get rid of left edge.
% For some weird reason, the entire left most first column is 1.  Make it 0


% Label
[labeledImage, numberOfRegions] = bwlabel(binaryImage);
coloredLabels = label2rgb (labeledImage, 'hsv', 'k', 'shuffle'); % pseudo random color labels
% subplot(2, 2, 2); 
% imshow(coloredLabels);
axis on;
hold on;

% Make measurements.
measurements = regionprops(labeledImage, 'MajorAxisLength', 'Orientation', 'Centroid', 'Area');

% bwboundaries() returns a cell array, where each cell contains the row/column coordinates for an object in the image.
% Plot the borders of all the coins on the original grayscale image using the coordinates returned by bwboundaries.
boundaries = bwboundaries(binaryImage);
numberOfBoundaries = size(boundaries, 1);
for blobIndex = 1 : numberOfBoundaries
	thisBoundary = boundaries{blobIndex};
	x = thisBoundary(:, 2); % x = columns.
	y = thisBoundary(:, 1); % y = rows.
	
	% Find which two bounary points are farthest from each other.
	maxDistance = -inf;
	for k = 1 : length(x)
		distances = sqrt( (x(k) - x) .^ 2 + (y(k) - y) .^ 2 );
		[thisMaxDistance, indexOfMaxDistance] = max(distances);
		if thisMaxDistance > maxDistance
			maxDistance = thisMaxDistance;
			index1 = k;
			index2 = indexOfMaxDistance;
		end
	end
	
	% Find the midpoint of the line.
	xMidPoint = mean([x(index1), x(index2)]);
	yMidPoint = mean([y(index1), y(index2)]);
	longSlope = (y(index1) - y(index2)) / (x(index1) - x(index2))
	perpendicularSlope = -1/longSlope
	% Use point slope formula (y-ym) = slope * (x - xm) to get points
	y1 = perpendicularSlope * (1 - xMidPoint) + yMidPoint;
	y2 = perpendicularSlope * (columns - xMidPoint) + yMidPoint;
	
	% Get the profile perpendicular to the midpoint so we can find out when if first enters and last leaves the object.
	[cx,cy,c] = improfile(binaryImage,[1, columns], [y1, y2], 1000);
	% Get rid of NAN's that occur when the line's endpoints go above or below the image.
	c(isnan(c)) = 0;
	firstIndex = find(c, 1, 'first');
	lastIndex = find(c, 1, 'last');
	% Compute the distance of that perpendicular width.
	perpendicularWidth = sqrt( (cx(firstIndex) - cx(lastIndex)) .^ 2 + (cy(firstIndex) - cy(lastIndex)) .^ 2 );
	% Get the average perpendicular width.  This will approximately be the area divided by the longest length.
	averageWidth = measurements(blobIndex).Area / maxDistance;
	
	% Plot the boundaries, line, and midpoints over the two images.
	% Plot the boundary over the gray scale image
% 	subplot(2, 2, 3);
	plot(x, y, 'y-', 'LineWidth', 3);
	% For this blob, put a line between the points farthest away from each other.
	line([x(index1), x(index2)], [y(index1), y(index2)], 'Color', 'r', 'LineWidth', 3);
	plot(xMidPoint, yMidPoint, 'r*', 'MarkerSize', 15, 'LineWidth', 2);
	% Plot perpendicular line.  Make it green across the whole image but magenta inside the blob.
	line([1, columns], [y1, y2], 'Color', 'g', 'LineWidth', 3);	
	line([cx(firstIndex), cx(lastIndex)], [cy(firstIndex), cy(lastIndex)], 'Color', 'm', 'LineWidth', 3);
	
	message = sprintf('The longest line is red.\nPerpendicular to that, at the midpoint, is green.\nMax distance for blob #%d = %.2f\nPerpendicular distance at midpoint = %.2f\nAverage perpendicular width = %.2f (approximately\nArea = %d', ...
		blobIndex, maxDistance, perpendicularWidth, averageWidth, measurements(blobIndex).Area);
	fprintf('%s\n', message);
	uiwait(helpdlg(message));
end

% Make a mask centered at each endpoint.  Let's just say it's a box for simplicity.
% The box with the greatest area of white pixels will be the head.

% First for point 1
plot(x(index1), y(index1), 'y*');
boxWidth = 60; % half Width
% Draw the box
xb1 = x(index1) - boxWidth;
xb2 = x(index1) + boxWidth;
yb1 = y(index1) - boxWidth;
yb2 = y(index1) + boxWidth;
% Make sure all coordinates are in side the image.
xb1 = min([xb1, columns]);
xb1 = max([xb1, 1]);
xb2 = min([xb2, columns]);
xb2 = max([xb2, 1]);
yb1 = min([yb1, rows]);
yb1 = max([yb1, 1]);
yb2 = min([yb2, rows]);
yb2 = max([yb2, 1]);
% Draw the box.
plot([xb1, xb2, xb2, xb1, xb1], [yb1, yb1, yb2, yb2, yb1], 'y-');
subImage1 = binaryImage(yb1:yb2, xb1:xb2);
numWhitePixels1 = sum(subImage1(:))

% Next for point 2
plot(x(index2), y(index2), 'y*');
% Draw the box
xb1 = x(index2) - boxWidth;
xb2 = x(index2) + boxWidth;
yb1 = y(index2) - boxWidth;
yb2 = y(index2) + boxWidth;
% Make sure all coordinates are in side the image.
xb1 = min([xb1, columns]);
xb1 = max([xb1, 1]);
xb2 = min([xb2, columns]);
xb2 = max([xb2, 1]);
yb1 = min([yb1, rows]);
yb1 = max([yb1, 1]);
yb2 = min([yb2, rows]);
yb2 = max([yb2, 1]);
% Draw the box.
plot([xb1, xb2, xb2, xb1, xb1], [yb1, yb1, yb2, yb2, yb1], 'y-');
subImage2 = binaryImage(yb1:yb2, xb1:xb2);
numWhitePixels2 = sum(subImage2(:))

% Put a label at the head
if numWhitePixels2 > numWhitePixels1
	text(x(index2), y(index2), 'Point 2 = Head', 'FontSize', fontSize, 'Color', 'r');	
	text(x(index1), y(index1), 'Point 1 = Tail', 'FontSize', fontSize, 'Color', 'r');
else
	text(x(index1), y(index1), 'Point 1 = Head', 'FontSize', fontSize, 'Color', 'r');	
	text(x(index2), y(index2), 'Point 2 = Tail', 'FontSize', fontSize, 'Color', 'r');
end