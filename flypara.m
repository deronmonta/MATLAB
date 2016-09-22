
% This is a program built for analyzing the fly position extracted from the
% mytracking 


function flypara()

prompt = 'Enter file name\n';


str = input(prompt, 's');

load(str);


disp(tracks.wholepos);

time = length(tracks.wholepos);
realtime = ((time/30)*4);
disp 'Total frames';
disp(time);

disp 'Total time';
disp(realtime);

tracks.zonet = 0;
tracks = correction(tracks);
tracks = zonetime(tracks);
tracks = findrunstop(tracks);
tracks = findaspeed(tracks);




reply = input('Do you want to print  Y/N [Y]:','s');
        
       if reply == 'y'||'Y'
                    
           displayresults(tracks);
           
       end


end

% Correct error value
function tracks = correction(tracks)


%Identify two kinds of error: distance error and speed error

diserror = find(tracks.dis2center > 380); % Get the indices of the error position

tracks.wholepos(diserror,:) = [];
tracks.dis2center(diserror,:) = [];
tracks.speed =[];


%recalculate speed using corrected data 
for i = 2:length(tracks.wholepos) 

    newspeed = sqrt(sum((tracks.wholepos(i,:)-tracks.wholepos((i-1),:)).^2));
    
    tracks.speed = [tracks.speed;newspeed];
    
    
end

speederror = find(tracks.speed > 40);% Find speed error using new speed


tracks.wholepos(speederror,:) = [];
tracks.dis2center(speederror,:) = [];
tracks.speed(speederror,:) = [];


fprintf('dis2center');
disp(length(tracks.dis2center));
fprintf('speed');
disp(length(tracks.speed));
fprintf('wholepos');
disp(size(tracks.wholepos));
end



% This function returns the time fly spent in the target zone and the speed
% in the target zone
function tracks = zonetime(tracks)

timeenter= 0;
timeexit = 0;
tracks.isin = [];
tracks.attraction = 0;
targetzone = 140;% Change this variable to reidentify the target zone. 

        for k = 1 : size(tracks.dis2center) 
   
    
    %Find the total zonetime and identify if fly is target zone and assign
    %bolean to all dis2center 
    if (tracks.dis2center(k) < targetzone)%The radius of the target zone 
        
        tracks.zonet = tracks.zonet + 1;
       
        
        tracks.isin= [tracks.isin;true]; 
    end
    
    if (tracks.dis2center(k) > targetzone)
        
       tracks.isin= [tracks.isin;false] ; 
        
    end
      
        end
    
        
        tracks.attraction = tracks.zonet / length(tracks.dis2center);
        
    inindex = find(tracks.isin == true);
    
     if ~isempty(inindex)
         
         inindex(end-1:end) = [];
     end
         
         
    outindex = find(tracks.isin == false);
    outindex(end-1:end) = [];
    
    
%     fprintf('in index\n');
%     
%      disp (length(outindex));
%      
%      disp outindex
%      disp (length(inindex));
%      
%      
%    
   
    tracks.speedin = mean(tracks.speed(inindex)); 
    tracks.speedout = mean (tracks.speed(outindex));
    
    
    
    fprintf('Attraction Index');
    disp(tracks.attraction);
   
    disp speedin;
    disp(tracks.speedin);
    
   disp speedout
    disp(tracks.speedout);
        


  %Find each indivdiual time entered
  for k = 2:size(tracks.dis2center)
     
     if (tracks.dis2center(k-1) < targetzone && tracks.dis2center(k) > targetzone)
        
       timeexit = cat(1,timeexit,k);
        
     end
     
     if (tracks.dis2center(k-1) > targetzone && tracks.dis2center(k) < targetzone)
        
       timeenter = cat(1,timeenter,k);
     
    end
    
  end
     
  
% fprintf('The time entered:\n');
% disp(timeenter);
% 
% fprintf('The time exit:\n');
% disp(timeexit);



%Make sure all the arrays are the same size and calculate each time span


tracks.outime = [];

for o = 2:length(timeenter)

   newout = timeenter (o)- timeenter(o-1);
    

   tracks.outime = [tracks.outime;newout];
end

disp(tracks.outime);

if tracks.dis2center(1) > targetzone
    
    if length(timeexit) == length(timeenter)
      tracks.timespan = timeexit - timeenter;

    end
    
    if length(timeexit) < length(timeenter)
       timeenter(end,:) = [];
       tracks.timespan = timeexit - timeenter;
       
    end
    
    if length(timeexit) > length(timeenter)
    
    timeexit(end,:) = [];
    tracks.timespan = timeexit - timeenter;

    end
end
    

  
    if tracks.dis2center(1) < targetzone
 
    if length(timeexit) == length(timeenter)
        
      tracks.outtimespan = timeexit - timeenter;
      tracks.timespan = timeenter - timeexit;
      
    end
    
    if length(timeexit) < length(timeenter)
        
       timeenter(end,:) = [];
       tracks.outtimespan = timeexit - timeenter;
        tracks.timespan = timeenter - timeexit;
    end
    
    if length(timeexit) > length(timeenter)
    
    timeexit(end,:) = [];
    tracks.outtimespan = timeexit - timeenter;
     tracks.timespan = timeenter - timeexit;
     
    end
    
    
    end



fprintf('Each time span:\n');
disp(tracks.timespan);

fprintf('Total Zone time\n');
disp(tracks.zonet);


end



%This function finds the runs and stops of the fly according to the speed

function tracks = findrunstop(tracks)


stopcount = 0;
stops = 0;
runcount = 0;
runs = 0;
for i = 1:length(tracks.speed)
    
    
    if tracks.speed(i) < 2 % Stop thereshold

         
        stopcount = stopcount + 1;
        
       if stopcount > 50
            
            stops = stops +1;
            
            stopcount = 0;
       
                   
       end
        
       
       
    end

end


tracks.stops = stops;
disp stopcount;
disp(stopcount);

disp stops;
disp(stops);

%Find total number of runs
for i = 1:length(tracks.speed)
    
    
    if tracks.speed(i) > 15 %run thereshold

         
        runcount = runcount + 1;
        
       if runcount > 50
            
            runs = runs +1;
            runcount = 0;     
           
       end
       
    end

end



tracks.runs = runs;
disp runs;
disp(runs);



end




%Find the angle of the fly
function tracks = findaspeed(tracks)


angspeed = [];

tenth = tracks.wholepos(1:10:length(tracks.wholepos),:);


for i = 2:length(tenth)
    
    center = [500 400];
    
      vec2 = center-tenth(i);
      vec = center-tenth(i-1);
      
       aspeed = dot(vec,vec2)/(norm(vec)*norm(vec2));% Angle between fly vector and the food 
        
      
      tracks.angspeed = [angspeed;aspeed];
    
        
end

fprintf('Angular speed');
disp(tracks.angspeed);

end




% Plot all the results 

function displayresults(tracks)

seconds = (length(tracks.wholepos)*4)/30;


figure;


title('Track results');
%set(gca,'YDir','Reverse');

hold on 
if length(tracks.wholepos) <400
    
plot(tracks.wholepos(1:end,1),tracks.wholepos(1:end,2),'color',[0 0 1]);

end

if length(tracks.wholepos) > 400 && length(tracks.wholepos) < 800
    
plot(tracks.wholepos(1:400,1),tracks.wholepos(1:400,2),'color',[0 0 1]);
plot(tracks.wholepos(401:end,1),tracks.wholepos(401:end,2),'color',[0 0.5 1]);

end

if length(tracks.wholepos) > 800 && length(tracks.wholepos) < 1200
    
plot(tracks.wholepos(1:400,1),tracks.wholepos(1:400,2),'color',[0 0 1]);
plot(tracks.wholepos(401:800,1),tracks.wholepos(401:800,2),'color',[0 0.5 1]);
plot(tracks.wholepos(801:end,1),tracks.wholepos(801:end,2),'color',[0 1 1]);

end

if length(tracks.wholepos) > 1200 && length(tracks.wholepos) < 1600

plot(tracks.wholepos(1:400,1),tracks.wholepos(1:400,2),'color',[0 0 1]);
plot(tracks.wholepos(401:800,1),tracks.wholepos(401:800,2),'color',[0 0.5 1]);
plot(tracks.wholepos(801:1200,1),tracks.wholepos(801:1200,2),'color',[0 1 1]);
plot(tracks.wholepos(1201:end,1),tracks.wholepos(1201:end,2),'color',[0 1 0.5]);

end

if length(tracks.wholepos) > 1600 && length(tracks.wholepos) < 2000
    plot(tracks.wholepos(1:400,1),tracks.wholepos(1:400,2),'color',[0 0 1]);
    plot(tracks.wholepos(401:800,1),tracks.wholepos(401:800,2),'color',[0 0.5 1]);
    plot(tracks.wholepos(801:1200,1),tracks.wholepos(801:1200,2),'color',[0 1 1]);
    plot(tracks.wholepos(1201:1600,1),tracks.wholepos(1201:1600,2),'color',[0 1 0.5]);
    plot(tracks.wholepos(1601:end,1),tracks.wholepos(1601:end,2),'color',[0 1 0.5]);

end


if length(tracks.wholepos) > 2000 && length(tracks.wholepos) < 2400
    
    plot(tracks.wholepos(1:400,1),tracks.wholepos(1:400,2),'color',[0 0 1]);
    plot(tracks.wholepos(401:800,1),tracks.wholepos(401:800,2),'color',[0 0.5 1]);
    plot(tracks.wholepos(801:1200,1),tracks.wholepos(801:1200,2),'color',[0 1 1]);
    plot(tracks.wholepos(1201:1600,1),tracks.wholepos(1201:1600,2),'color',[0 1 0.5]);
    plot(tracks.wholepos(1601:2000,1),tracks.wholepos(1601:2000,2),'color',[0 1 0.5]);
    plot(tracks.wholepos(2001:end,1),tracks.wholepos(2001:end,2),'color',[0.5 1 0]);

end

if length(tracks.wholepos) > 2400 && length(tracks.wholepos) < 2800
    plot(tracks.wholepos(1:400,1),tracks.wholepos(1:400,2),'color',[0 0 1]);
    plot(tracks.wholepos(401:800,1),tracks.wholepos(401:800,2),'color',[0 0.5 1]);
    plot(tracks.wholepos(801:1200,1),tracks.wholepos(801:1200,2),'color',[0 1 1]);
    plot(tracks.wholepos(1201:1600,1),tracks.wholepos(1201:1600,2),'color',[0 1 0.5]);
    plot(tracks.wholepos(1601:2000,1),tracks.wholepos(1601:2000,2),'color',[0 1 0.5]);
    plot(tracks.wholepos(2001:2400,1),tracks.wholepos(2001:2400,2),'color',[0.5 1 0]);
    plot(tracks.wholepos(2401:end,1),tracks.wholepos(2401:end,2),'color',[1 1 0]);

end

if length(tracks.wholepos) > 2800 && length(tracks.wholepos) < 3200
     plot(tracks.wholepos(1:400,1),tracks.wholepos(1:400,2),'color',[0 0 1]);
    plot(tracks.wholepos(401:800,1),tracks.wholepos(401:800,2),'color',[0 0.5 1]);
    plot(tracks.wholepos(801:1200,1),tracks.wholepos(801:1200,2),'color',[0 1 1]);
    plot(tracks.wholepos(1201:1600,1),tracks.wholepos(1201:1600,2),'color',[0 1 0.5]);
    plot(tracks.wholepos(1601:2000,1),tracks.wholepos(1601:2000,2),'color',[0 1 0.5]);
    plot(tracks.wholepos(2001:2400,1),tracks.wholepos(2001:2400,2),'color',[0.5 1 0]);
    plot(tracks.wholepos(2401:2800,1),tracks.wholepos(2401:2800,2),'color',[1 1 0]);
    plot(tracks.wholepos(2801:end,1),tracks.wholepos(2801:end,2),'color',[1 0.5 0]);

end

if length(tracks.wholepos) > 3200
    plot(tracks.wholepos(1:400,1),tracks.wholepos(1:400,2),'color',[0 0 1]);
    plot(tracks.wholepos(401:800,1),tracks.wholepos(401:800,2),'color',[0 0.5 1]);
    plot(tracks.wholepos(801:1200,1),tracks.wholepos(801:1200,2),'color',[0 1 1]);
    plot(tracks.wholepos(1201:1600,1),tracks.wholepos(1201:1600,2),'color',[0 1 0.5]);
    plot(tracks.wholepos(1601:2000,1),tracks.wholepos(1601:2000,2),'color',[0 1 0.5]);
    plot(tracks.wholepos(2001:2400,1),tracks.wholepos(2001:2400,2),'color',[0.5 1 0]);
    plot(tracks.wholepos(2401:2800,1),tracks.wholepos(2401:2800,2),'color',[1 1 0]);
    plot(tracks.wholepos(2801:3200,1),tracks.wholepos(2801:3200,2),'color',[1 0.5 0]);
    plot(tracks.wholepos(3201:end,1),tracks.wholepos(3201:end,2),'color',[1 0 0]);

end



hold off


                     figure;
                     
                     subplot(2,2,1);
                 
                    plot(tracks.angspeed);
                      set(gca,'YDir','Reverse');
                      title('Parameters');
                    
                   %Plot the distance to source
                    subplot(2,2,2)
                    title('Distance to Src');
                     
                     hold on;
                     t = length(tracks.dis2center);
                     
                     
                     targetzone = 140;
                     
                     plot([1 t],[targetzone,targetzone],'b');%Draw a line intersection the target zone
                    plot(tracks(1).dis2center,'r');
                     
                     hold off
                  
                      subplot(2,2,4);
                      
                     bar(tracks.timespan(2:end));
                       
                       title('In Zone time');
                     
                      subplot(2,2,3);
                      plot(tracks(1).speed);
                      title('Speed');
                      
                      
                      %Display all the data on the graph
                      
                      avgspeed = sum(tracks.speed(:))/length(tracks.speed);
                      str2= num2str(avgspeed);
                      str1 = 'Average speed:';
                      str = strcat(str1,str2);
                      dim = [.2 .4 .7 .325];
                      annotation('textbox',dim,'FitBoxToText','on','String',str);

                      
                      str2 = num2str(tracks.speedin);
                      str1 = 'Speed in Target Zone:';
                      str = strcat(str1,str2);
                      dim = [.2 .4 .7 .35];
                      annotation('textbox',dim,'FitBoxToText','on','String',str);
                      
                      
                      str2 = num2str(tracks.speedout);
                      str1 = 'Speed outside of Target zone:';
                      str = strcat(str1,str2);
                      dim = [.2 .4 .7 .375];
                      annotation('textbox',dim,'FitBoxToText','on','String',str);
                      
                       str2 = num2str(tracks.attraction);
                      str1 = 'Attraction index:';
                      str = strcat(str1,str2);
                      dim = [.2 .4 .7 .4];
                      annotation('textbox',dim,'FitBoxToText','on','String',str);
                      
                      str2 = num2str(tracks.zonet);
                      str1 = 'Total zone time:';
                      str = strcat(str1,str2);
                      dim = [.2 .4 .7 .425];
                      annotation('textbox',dim,'FitBoxToText','on','String',str);
                      
                      str2 = num2str(tracks.runs);
                      str1 = 'Number of runs:';
                      str = strcat(str1,str2);
                      dim = [.2 .4 .7 .45];
                      annotation('textbox',dim,'FitBoxToText','on','String',str);
                      
                       str2 = num2str(tracks.stops);
                      str1 = 'Number of stops:';
                      str = strcat(str1,str2);
                      dim = [.2 .4 .7 .475];
                      annotation('textbox',dim,'FitBoxToText','on','String',str);
                      
                      
                      
                      figure; %Next figure
                      
                      subplot(2,2,1);
                    
                      plot(tracks.isin);
                        title('isin');
                        
                        
                      subplot(2,2,2);
                    
                     
                      bar((tracks.outime(:)));
                       title('Out zone time');

end
