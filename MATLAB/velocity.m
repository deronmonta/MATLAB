videoReader = vision.VideoFileReader('viptraffic.avi','ImageColorSpace','Intensity','VideoOutputDataType','uint8');
converter = vision.ImageDataTypeConverter; 
opticalFlow = vision.OpticalFlow('ReferenceFrameDelay', 1);
opticalFlow.OutputValue = 'Horizontal and vertical components in complex form';
shapeInserter = vision.ShapeInserter('Shape','Lines','BorderColor','Custom', 'CustomBorderColor', 255);
videoPlayer = vision.VideoPlayer('Name','Motion Vector');
while ~isDone(videoReader)
    frame = step(videoReader);
    im = step(converter, frame);
    of = step(opticalFlow, im);
    lines = videooptflowlines(of, 20);
    if ~isempty(lines)
      out =  step(shapeInserter, im, lines); 
      step(videoPlayer, out);
    end
    
end
release(videoPlayer);
release(videoReader);