function varargout = projectgui(varargin)
% PROJECTGUI MATLAB code for projectgui.fig
%      PROJECTGUI, by itself, creates a new PROJECTGUI or raises the existing
%      singleton*.
%
%      H = PROJECTGUI returns the handle to a new PROJECTGUI or the handle to
%      the existing singleton*.
%
%      PROJECTGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PROJECTGUI.M with the given input arguments.
%
%      PROJECTGUI('Property','Value',...) creates a new PROJECTGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before projectgui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to projectgui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help projectgui

% Last Modified by GUIDE v2.5 09-Dec-2021 17:41:13

% Begin initialization code - DO NOT EDIT
clc;
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @projectgui_OpeningFcn, ...
    'gui_OutputFcn',  @projectgui_OutputFcn, ...
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
% End initialization code - DO NOT EDIT


% --- Executes just before projectgui is made visible.
function projectgui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to projectgui (see VARARGIN)

% Choose default command line output for projectgui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes projectgui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = projectgui_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global ini_positionX
global ini_positionY
global ini_positionZ
global ini_velocityX
global ini_velocityY
global ini_velocityZ
global time
global F H R Q

Runs = 500;
T = 0.1;
total_time = time;
t = 0: T: total_time;
TotalScans = length(t);

mean = 0;
global meas_noise_sd;


nstates = 6;
nmeas = 3;
init_pos = [ini_positionX; ini_positionY ; ini_positionZ];
V = [ini_velocityX; ini_velocityY; ini_velocityZ];

F=[eye(3) T*eye(3);zeros(3) eye(3)]; % Transition matrix for constant velocity model

H=[eye(nmeas) zeros(nstates-nmeas)];   %Output Coeffecient Matrix

R=(meas_noise_sd^2)*eye(nmeas); % measurement covariance matrix

for run = 1 : Runs
    
    targetX = ini_positionX;
    targetY = ini_positionY;
    targetZ = ini_positionZ;
    target_velocityX = ini_velocityX;
    target_velocityY = ini_velocityY;
    target_velocityZ = ini_velocityZ;
    
    for scan = 2 : TotalScans
        
        targetX(scan) = targetX(scan - 1) + T*target_velocityX;
        targetY(scan) = targetY(scan - 1) + T*target_velocityY;
        targetZ(scan) = targetZ(scan - 1) + T*target_velocityZ;
    end
    Ztrue = [targetX ; targetY; targetZ ];
    
    meas_noise = mean + R*randn(nmeas, TotalScans);
    Znoisy = Ztrue + meas_noise;
    X0 = [Znoisy(:,1) ; V+diag(R).*randn( size(V) )];
    P0 = [R*eye(nmeas)  zeros(nmeas) ; zeros(nmeas)  (R^2)*eye(nmeas)/T];
    
    for scan = 1 : TotalScans
        
        if scan > 1
            Xpred=F*Xest;
            xxx(:,scan)=Xpred;
            Ppred=F*Pest*F';
            S = H*Ppred*H' + R;
            K = Ppred*H'*inv(S);
            abc(:,scan)= K*( Znoisy(:,scan) - H*Xpred);
            Xest = Xpred + K*( Znoisy(:,scan) - H*Xpred );
            Pest = ( eye(nstates) - K*H )*Ppred;
            
        else
            
            Xest = X0;
            Pest = P0;
            
        end
        
        EstimatedX(run,scan) = Xest(1);
        EstimatedY(run,scan) = Xest(2);
        EstimatedZ(run,scan) = Xest(3);
        
        Estimated_Vx(run,scan) = Xest(4);
        Estimated_Vy(run,scan) = Xest(5);
        Estimated_Vz(run,scan) = Xest(6);
    end
    
    Zx(run,:)=Znoisy(1,:);
    Zy(run,:)=Znoisy(2,:);
    Zz(run,:)=Znoisy(3,:);
end

avg_estimateX = sum(EstimatedX)/Runs;
disp('Estimated X co-od')
avg_estimateX(end)
avg_estimateY = sum(EstimatedY)/Runs;
disp('Estimated Y co-od')
avg_estimateX(end)
avg_estimateZ = sum(EstimatedZ)/Runs;
disp('Estimated Z co-od')
avg_estimateX(end)
avg_estimateVx = sum(Estimated_Vx)/Runs;
avg_estimateVy = sum(Estimated_Vy)/Runs;
avg_estimateVz = sum(Estimated_Vz)/Runs;
avg_Znoisyx = sum(Zx)/Runs;
avg_Znoisyy = sum(Zy)/Runs;
avg_Znoisyz = sum(Zz)/Runs;

TruevelocityX = repmat(V(1),Runs, TotalScans);
TruevelocityY = repmat(V(2),Runs, TotalScans);
TruevelocityZ = repmat(V(3),Runs, TotalScans);
RMSEX = sqrt( sum( (repmat(Ztrue(1,:),Runs,1) - EstimatedX).^2 )/Runs );
RMSEY = sqrt( sum( (repmat(Ztrue(2,:),Runs,1) - EstimatedY).^2 )/Runs );
RMSEZ = sqrt( sum( (repmat(Ztrue(3,:),Runs,1) - EstimatedZ).^2 )/Runs );
RMSEVx = sqrt( sum( (TruevelocityX - Estimated_Vx).^2 )/Runs );
RMSEVy = sqrt( sum( (TruevelocityY - Estimated_Vy).^2 )/Runs );
RMSEVz = sqrt( sum( (TruevelocityZ - Estimated_Vz).^2 )/Runs );
PositionRMSE = sqrt( RMSEX.^2 + RMSEY.^2+ RMSEZ.^2);
VelocityRMSE = sqrt( RMSEVx.^2 + RMSEVy.^2 + RMSEVz.^2);

RMSE = [ PositionRMSE; VelocityRMSE];


Estimate=[avg_estimateX; avg_estimateY;avg_estimateZ; avg_estimateVx; avg_estimateVy ; avg_estimateVz];
Monte_Znoisy=[avg_Znoisyx;avg_Znoisyy;avg_Znoisyz];
figure
subplot(311)
plot(t,Ztrue(1,:),'--b','LineWidth',1.5)
hold on
plot(t,Znoisy(1,:),'r-*','LineWidth',1.5)
hold on
plot(t,Estimate(1,:),'g+','LineWidth',1.5)
subplot(312)
plot(t,Ztrue(2,:),'--b','LineWidth',1.5)
hold on
plot(t,Znoisy(2,:),'r-*','LineWidth',1.5)
hold on
plot(t,Estimate(2,:),'g+','LineWidth',1.5)
subplot(313)
plot(t,Ztrue(3,:),'--b','LineWidth',1.5)
hold on
plot(t,Znoisy(3,:),'r-*','LineWidth',1.5)
hold on
plot(t,Estimate(3,:),'g+','LineWidth',1.5)

figure
plot(t,V(1,:),'r','LineWidth',1.5)
hold on
plot(t,Estimate(4,:),'b','LineWidth',1.5)
subplot(3,1,1)
plot(t,V(1,:),'+b',t,Estimate(4,:),'r-*')
xlabel('Time in seconds')
ylabel('Velocity in m/s')
title('True Velocity on X axis')
grid on
hold off
subplot(3,1,2)
plot(t,V(2,:),'+b',t,Estimate(5,:),'r-*')
xlabel('Time in seconds')
ylabel('Velocity in m/s')
title('Target Velocity on Y axis')
grid on
hold off
subplot(3,1,3)
plot(t,V(3,:),'+b',t,Estimate(6,:),'r-*')
xlabel('Time in seconds')
ylabel('Velocity in m/s')
title('Target Velocity on Z axis')
grid on
hold off

figure
plot3(Ztrue(1,:),Ztrue(2,:),Ztrue(3,:),'+b',Znoisy(1,:),Znoisy(2,:),Znoisy(3,:),'r-*',Estimate(1,:),Estimate(2,:),Estimate(3,:),'g+','LineWidth',1.5)
legend('True Position ','Measured Position ','Estimated Position ')
title('Trajectries of position')
xlabel('X Axis');
ylabel('Y Axis');
zlabel('Z Axis')
grid on
hold off

figure
subplot(211)
plot(t,PositionRMSE, 'LineWidth',1.5)
xlabel('Time in seconds')
ylabel('MSE (m)')
title('Mean Square Error of Position')
grid on
subplot(212);
plot(t,VelocityRMSE,'LineWidth',1.5);
xlabel('Time in seconds')
ylabel('MSE (m/s)')
title('Mean Square Error of Velocity')
grid on





function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global ini_positionX
ini_positionX=str2num(get(hObject,'String'));
% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global ini_positionY
ini_positionY=str2num(get(hObject,'String'));

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global ini_positionZ
ini_positionZ=str2num(get(hObject,'String'));

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global ini_velocityX
ini_velocityX=str2num(get(hObject,'String'));

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global ini_velocityY
ini_velocityY=str2num(get(hObject,'String'));

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global ini_velocityZ
ini_velocityZ=str2num(get(hObject,'String'));

% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double


% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit7_Callback(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global time
time=str2num(get(hObject,'String'));
% Hints: get(hObject,'String') returns contents of edit7 as text
%        str2double(get(hObject,'String')) returns contents of edit7 as a double


% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
contents=cellstr(get(hObject,'String'));
global meas_noise_sd;
popchoice=contents(get(hObject,'Value'));


if(strcmp(popchoice,'sunny'))
    meas_noise_sd=1.2;
end
if(strcmp(popchoice,'rainy'))
    meas_noise_sd=1.5;
end
if(strcmp(popchoice,'winter'))
    meas_noise_sd=1.8;
end
