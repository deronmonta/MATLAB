function varargout = Locomotion_v1_0(varargin)
% LOCOMOTION_V1_0 MATLAB code for Locomotion_v1_0.fig
%      LOCOMOTION_V1_0, by itself, creates a new LOCOMOTION_V1_0 or raises the existing
%      singleton*.
%
%      H = LOCOMOTION_V1_0 returns the handle to a new LOCOMOTION_V1_0 or the handle to
%      the existing singleton*.
%
%      LOCOMOTION_V1_0('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in LOCOMOTION_V1_0.M with the given input arguments.
%
%      LOCOMOTION_V1_0('Property','Value',...) creates a new LOCOMOTION_V1_0 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Locomotion_v1_0_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Locomotion_v1_0_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% r the above text to modify the response to help Locomotion_v1_0

% Last Modified by GUIDE v2.5 19-Aug-2014 11:24:20

% Begin initialization code - DO NOT R
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Locomotion_v1_0_OpeningFcn, ...
                   'gui_OutputFcn',  @Locomotion_v1_0_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT R


% --- Executes just before Locomotion_v1_0 is made visible.
function Locomotion_v1_0_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Locomotion_v1_0 (see VARARGIN)

% Choose default command line output for Locomotion_v1_0
handles.output = hObject;
set(handles.figure1,'Name','Locomotion');
handles.buffer=3;
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Locomotion_v1_0 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Locomotion_v1_0_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in load.
function load_Callback(hObject, eventdata, handles)
[fname1,fdir1]=uigetfile('*.mp4');
handles.movobj1=VideoReader([fdir1,'\',fname1]);
handles.numFrm=get(handles.movobj1,'numberOfFrames');
handles.currentFrm=1;
set(handles.from,'String',num2str(1));
set(handles.to,'String',num2str(handles.numFrm));
vid01=read(handles.movobj1,1);
handles.s1=size(vid01);
axes(handles.axes1)
imshow(vid01)
guidata(hObject, handles);
% hObject    handle to load (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


function from_Callback(hObject, eventdata, handles)
% hObject    handle to from (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of from as text
%        str2double(get(hObject,'String')) returns contents of from as a double


% --- Executes during object creation, after setting all properties.
function from_CreateFcn(hObject, eventdata, handles)
% hObject    handle to from (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: r controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function to_Callback(hObject, eventdata, handles)
% hObject    handle to to (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of to as text
%        str2double(get(hObject,'String')) returns contents of to as a double


% --- Executes during object creation, after setting all properties.
function to_CreateFcn(hObject, eventdata, handles)
% hObject    handle to to (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: r controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function distance_Callback(hObject, eventdata, handles)
% hObject    handle to distance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of distance as text
%        str2double(get(hObject,'String')) returns contents of distance as a double


% --- Executes during object creation, after setting all properties.
function distance_CreateFcn(hObject, eventdata, handles)
% hObject    handle to distance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in regulate.
function regulate_Callback(hObject, eventdata, handles)
startframe=str2num(get(handles.from,'String'));
vid01=read(handles.movobj1,startframe);
img01=vid01(:,:,:,1);
imshow(img01);
p1=round(getPosition(impoint));
p2=round(getPosition(impoint));
Rv=p2-p1;
R=sqrt(Rv(1)^2+Rv(2)^2);
set(handles.pixels,'String',num2str(R));

% hObject    handle to regulate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


function fps_Callback(hObject, eventdata, handles)
% hObject    handle to fps (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of fps as text
%        str2double(get(hObject,'String')) returns contents of fps as a double


% --- Executes during object creation, after setting all properties.
function fps_CreateFcn(hObject, eventdata, handles)
% hObject    handle to fps (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in start.
function start_Callback(hObject, eventdata, handles)
startframe=str2num(get(handles.from,'String'));
endframe=str2num(get(handles.to,'String'));
vid01=read(handles.movobj1,[startframe startframe+handles.buffer]);
img01=vid01(:,:,:,1);
imshow(img01);
p=round(getPosition(impoint));
x=p(:,1);
y=p(:,2);
r=img01(y,x,1);
g=img01(y,x,2);
b=img01(y,x,3);
track=0;
ct=1;
ct1=1;
temp=[];
temp2=[];
time=[];
timec=0;
speed=[];
vel=[];
mk=[];
t1=0;
t2=0;
f0=str2num(get(handles.fps,'String'));
f=round(30/f0);
for a=startframe:f:endframe
    img1=vid01(:,:,:,1);
    img2=img1-65;
    s=size(img2);
    img02=uint8(double(img2)*3);
    img3=zeros(s(1),s(2));
    bidx1=find(img1(:,:,1)>=r-5 & img1(:,:,2)<=g & img1(:,:,3)<=b);
    img3(bidx1)=255;
    img3(1,:)=0;
    img3(end,:)=0;
    img3(:,1)=0;
    img3(:,end)=0;
    imgb3=img3==255;
    a1=max(imgb3);
    a11=diff(a1);
    a2=max(imgb3');
    a21=diff(a2);
    ymax1=find(a11>0);
    ymin1=find(a11<0);
    ypt=round((ymax1+ymin1)/2);
    xmax1=find(a21>0);
    xmin1=find(a21<0);
    xpt=round((xmax1+xmin1)/2);
    for i=1:length(xpt)
        for j=1:length(ypt)
            if img3(xpt(i),ypt(j))==255
               mk(ct,:)=[xpt(i) ypt(j)];
               ct=ct+1;
            end
        end
    end
    axes(handles.axes1);
    imshow(img1);
    hold on
    if isempty(mk)==0 
       plot(mk(:,2),mk(:,1),'r');
    end
    hold off
    vid01(:,:,:,1)=[];
    if a+handles.buffer+1 < endframe
       vid01(:,:,:,1+handles.buffer)=read(handles.movobj1,a+handles.buffer+f);
    else break;
    end
end
s=size(mk(:,1));
s1=s(1);
p=str2num(get(handles.pixels,'String')); 
d=str2num(get(handles.distance,'String'));
for a=2:s1
    temp(a-1,:)=mk(a,:);
end
for a=2:s1
    temp2=mk(a,:)-mk(a-1,:);
    k=sqrt(temp2(1)^2+temp2(2)^2)/p*d;
    if k==0
       t1=t1+1;
    else t2=t2+1;
    end
    if a==2
       a=a+1;
    end
    track(a-1)=abs(track(a-2)+sqrt(temp2(1)^2+temp2(2)^2));
end
for a=2:s1-1
    vel(1)=0;
    vel(a)=track(a)-track(a-1);
    time(1)=0;
    timec=timec+1/f0;
    time(a)=timec;
    speed(1)=0;
    speed(a)=vel(a)*d/p*f0;
end
track=d/p*track;
vel=d/p*vel;
fr=endframe-startframe+1;
trackcm=track(s1-1);
speedcm=trackcm*30/fr;
test1=[time' temp track' vel' speed'];
mm={'時間(s)' 'X Axis' 'Y Axis' '位移(cm)' '順時位移(cm)' '順時速度(cm/s)'};
mm2={'總位移(cm)' '平均速度(cm/s)'};
xlswrite('output.xls',mm,1,'A1');
xlswrite('output.xls',mm2,1,'H1');
xlswrite('output.xls',test1,1,'A3');
xlswrite('output.xls',trackcm,1,'H3');
xlswrite('output.xls',speedcm,1,'I3');
    
    
    

% hObject    handle to start (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
