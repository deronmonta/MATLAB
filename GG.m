tic
% Read in sample demo video shipped with MATLAB.
vidobj = VideoReader('rhinos.avi');
NumFrames = vidobj.NumberOfFrames;

% Prepare a figure to show the images
% in the upper portion of the screen.
figure;
screenSize = get(0, 'ScreenSize');
newWindowPosition = [1 screenSize(4)/2 - 70 screenSize(3)
screenSize(4)/2];
set(gcf, 'Position', newWindowPosition); % Maximize figure.

% Find the max gray level over all frames.
level = -1;
for m=1:NumFrames
thisFrame = read(vidobj, m);
if ndims(thisFrame) > 1
thisFrame = rgb2gray(thisFrame);
end
subplot(1,2,1);
imshow(thisFrame);
drawnow; % Force display to update.
    level = max([level, max(thisFrame(:))]);
fprintf('After frame %d, the max gray level is %d.\n', m, level);
if level == intmax(class(thisFrame))
% No need to check the rest of the frames,
% because we're already at the limit.
break;
end
end
message = sprintf('The max gray level is %.3f', level);
uiwait(msgbox(message));

for frameNumber = 1:NumFrames
fprintf('Processing frame %d. ', frameNumber);
thisFrame = read(vidobj, frameNumber);
if ndims(thisFrame) > 1
thisFrame = rgb2gray(thisFrame);
end
subplot(1,2,1);
cla reset;	% Get rid of prior images and text.
imshow(thisFrame);
caption = sprintf('Frame #%d', frameNumber);
title(caption, 'FontSize', fontSize);

% Threshold the image.
binaryImage = thisFrame > level-10;
% Note: no need for the "& thisFrame <= level+10" criteria
% since level is already the max value.
binaryImage = bwareaopen(binaryImage,15);
subplot(1,2,2);
cla reset;	% Get rid of prior images and text.
imshow(binaryImage)
title(caption, 'FontSize', fontSize);
drawnow; % Force display to update.
hold on

% Label the image.
labeledImage = bwconncomp(binaryImage,8);
measurements = regionprops(labeledImage,'BoundingBox','Centroid');
if length(measurements) == 0
% No blobs found.
fprintf('Done!\n');
continue; % Skip to the next frame.
end

totalNumberOfBlobs = length(measurements);
for blobNumber = 1:totalNumberOfBlobs
bb = measurements(blobNumber).BoundingBox;
bco = measurements(blobNumber).Centroid;
rectangle('Position',bb,'EdgeColor','r','LineWidth',2)
plot(bco(1),bco(2),'-m+')
myCaption = sprintf('(x=%.1f, y=%.1f)', bco(1), bco(2));
text(bco(1)+15,bco(2), myCaption,...
'FontName','Arial','FontWeight','normal',...
'FontSize',12,'Color','blue');
set(gca,'xdir','normal')
set(gca,'ydir','reverse')
axis on;
end
drawnow; % Force display to update.
fprintf('Done!\n');
end

hold off
toc
uiwait(msgbox('Execution Ended'));