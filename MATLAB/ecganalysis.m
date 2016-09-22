function ecganalysis

close all
clear all
%load ecg data
ecgsig
 
%butterworth filter
N=2;
Wn = 0.5;
[b,a] = butter(N,Wn);
ECG_filtered = filter(b,a,ecg);

% find the QRS complex...i.e. peak of each beat
    
peaks = find(ECG_filtered > 200);

 %remove extraneous points
    for p = 1:length(peaks)-1
        if peaks(p+1)-peaks(p) == 1
            peaks(p)=0;
        else peaks(p)=peaks(p);
        end
        p=p+1;
    end

peaks(peaks==0)=[];    

%calculate the number of beats
number_beats_in_signal = length(peaks);
fs = 200;
time = 0:1/fs:(length(ecg)-1)*1/fs;
%calculate the heartrate
beat_frequency = (length(ecg)-1)*1/fs/number_beats_in_signal;
bpm = beat_frequency*60;

%% figures
%two plots on same axis   
disp('Finished...Press any key to see the answer!')  
pause

number_beats_in_signal
bpm
 
figure
hold on
plot(time,ecg)
plot(time,ECG_filtered, 'r')
    xlabel('Time (s)')
    ylabel('Amplitude')
    title('ECG Signal')
    legend('Raw signal', 'Filtered signal')
hold off

% two plots next to each other
figure
hold on
    subplot(2,1,1), plot(time,ecg) 
    title('Raw Signal')
    ylabel('Amplitude')
    xlabel('Time (s)')
    subplot(2,1,2), plot(time,ECG_filtered)
    title('Filtered Signal')
    ylabel('Amplitude')
    xlabel('Time (s)')
hold off
%verification plot    
figure
plot(ECG_filtered)
hold on
plot(peaks,ECG_filtered(peaks),'r*')
    xlabel('Samples')
    ylabel('Amplitude')
    title('Filtered ECG Signal with QRS complex identified')
hold off
%%



