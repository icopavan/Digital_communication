function varargout = my_dig(varargin)
% MY_DIG MATLAB code for my_dig.fig
%      MY_DIG, by itself, creates a new MY_DIG or raises the existing
%      singleton*.
%
%      H = MY_DIG returns the handle to a new MY_DIG or the handle to
%      the existing singleton*.
%
%      MY_DIG('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MY_DIG.M with the given input arguments.
%
%      MY_DIG('Property','Value',...) creates a new MY_DIG or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before my_dig_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to my_dig_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help my_dig

% Last Modified by GUIDE v2.5 28-Feb-2015 18:24:11

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @my_dig_OpeningFcn, ...
    'gui_OutputFcn',  @my_dig_OutputFcn, ...
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


% --- Executes just before my_dig is made visible.
function my_dig_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to my_dig (see VARARGIN)
b=imread('awe.png');
set(handles.pushbutton4,'CData',b)
% Choose default command line output for my_dig
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes my_dig wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = my_dig_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
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

% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
N = str2double(get(handles.edit2,'string'));
noise_amp = str2double(get(handles.edit3,'string'));
% either 'bpsk1', 'bpsk2', 'ask', or 'fsk'
bits = ['1', '0', '1', '0', '0', '1'];
a=get(handles.checkbox1,'value');
b=get(handles.checkbox2,'value');
c=get(handles.checkbox3,'value');
d=get(handles.checkbox4,'value');
if a==1
    signal1 = ones(1,N);
    signal0 = -signal1;
elseif b==1
    signal1 = sqrt(2)*sin(2*pi*2*[0:N-1]/N);
    signal0 = -signal1;
elseif c==1
    signal1 = sqrt(2)*sin(2*pi*2*[0:N-1]/N);
    signal0 = sqrt(2)*sin(2*pi*3*[0:N-1]/N);
elseif d==1
    signal1 = ones(1,N);
    signal0 = zeros(1,N);
else
    h = msgbox('Select Digital Modulation technique', 'Error','error');
end
color0='r';color1='b';
x = []; xcolor = [];
for n=1:length(bits)
    x=[x eval(strcat('signal',bits(n)))];
    xcolor = [xcolor eval(strcat('color',bits(n)))];
end
% Send signal through white noise channel
r = x + noise_amp*randn(1,length(x));
% Run matched filters
y1=filter(signal1(N:-1:1),1,r);
y0=filter(signal0(N:-1:1),1,r);
% Graphics

axes(handles.axes1)
t=[0:length(r)-1];
plot(t,r,'k');
a = axis;
xp=x*(0.75*max(abs([a(3) a(4)])/max(x)));
for n=1:length(bits)
    plot(t((n-1)*N+1:n*N),xp((n-1)*N+1:n*N),[xcolor(n) '--']);
    h = text((n-1)*N+N/2,max(xp),bits(n));
    set(h,'color',xcolor(n));
end
for n=N*[1:length(bits)],h=line([n n],a(3:4));set(h,'linestyle','--');end
h=title('Received signal');
grid on

axes(handles.axes2)
plot(t,y0,color0,t,y1,color1)
a = axis;
for n=1:length(bits)
    if y1(n*N)>= y0(n*N)
        h = text(n*N-10,.75*a(4),'1');
        set(h,'color',color1);
        if bits(n) == '0'
            set(h,'fontweight','bold');
        end
    else
        h = text(n*N-10,.75*a(4),'0');
        set(h,'color',color0);
        if bits(n) == '1'
            set(h,'fontweight','bold');
        end
    end
end
for n=N*[1:length(bits)],h=line([n n],a(3:4));set(h,'linestyle','--');end
h=title('Matched Filter Output');grid on
%
% Compute Pr[e] curves
snrdb = [-10:.5:12];
snr = 10.^(snrdb/10);
p_bpsk = (sqrt(2*snr));
p_fsk = (sqrt(snr));
axes(handles.axes3)
h=semilogy(snrdb,p_bpsk,snrdb,p_fsk,'r--');grid;axis([-10 12 10^(-8) 1])
h=xlabel('Signal-to-Noise Ratio (dB)');
h=ylabel('Bit Error Probability');
legend('BPSK','FSK');
%
% Error Correction (Repetition Code)
%
axes(handles.axes4)
snrdb = [-10:.5:12];
snr = 10.^(snrdb/10);
p_bpsk = (sqrt(2*snr));
pb_bpsk = p_bpsk;
pb_rep = 1-(1-p_bpsk).^3-3*p_bpsk.*(1-p_bpsk).^2;
p_31 = (sqrt(2*3*snr)); % 3 times longer transmission time
pb_31 = p_31;
semilogy(snrdb,pb_bpsk,'b-',snrdb,pb_31,'r--',snrdb,pb_rep,'k-.');
grid;axis([-10 12 10^(-15) 1])
h=xlabel('Signal-to-Noise Ratio (dB)');
h=ylabel('Block Error Probability');
legend('No ECC','3 Times Longer Bit Interval','Length 3 Repetition Code');
% (7,4) Hamming Code Performance Curves
%
axes(handles.axes5)
snrdb = [-10:.5:12];
snr = 10.^(snrdb/10);
p_bpsk = (sqrt(2*snr));
pb_uc = 1-(1-p_bpsk).^4;
p_74 = (sqrt(2*7/4*snr)); % 7/4 longer transmission time
pb_74 = 1-(1-p_74).^4;
pb_hamming = 1-(1-p_bpsk).^7-7*p_bpsk.*(1-p_bpsk).^6;
semilogy(snrdb,pb_uc,'b-',snrdb,pb_74,'r--',snrdb,pb_hamming,'k-.');
grid;axis([-10 12 10^(-15) 1]);

legend('Uncoded','7/4 Longer Bit Interval','Hamming Code');
h=xlabel('Signal-to-Noise Ratio (dB)');
h=ylabel('Block Error Probability');
title('(7,4) Hamming Code Performance');
%
% Capacity Calculations
%
axes(handles.axes6)
p=[0:.01:.5];
C=1+p.*log2(p)+(1-p).*log2(1-p);
C(1) = 1;

plot(p,C)
grid on
h = xlabel('Error Probability');
h = ylabel('Capacity (bits)');
figure
C=1+p_bpsk.*log2(p_bpsk)+(1-p_bpsk).*log2(1-p_bpsk);
plot(snrdb,C);
grid;legend('Capacity using BPSK')
h = xlabel('Signal-to-Noise Ratio (dB)');
h = ylabel('Capacity (bits)');

% --- Executes on button press in checkbox1.
function checkbox1_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox1


% --- Executes on button press in checkbox3.
function checkbox3_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox3


% --- Executes on button press in checkbox4.
function checkbox4_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox4


% --- Executes on button press in checkbox2.
function checkbox2_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox2


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clear all
close all
clc


% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes1
cla


% --- Executes during object creation, after setting all properties.
function axes2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes2
cla


% --- Executes during object creation, after setting all properties.
function axes5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes5
cla


% --- Executes during object creation, after setting all properties.
function axes6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes6
cla


% --- Executes during object creation, after setting all properties.
function axes3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes3
cla


% --- Executes during object creation, after setting all properties.
function axes4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes4
cla

% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
helpdlg('Download my Standalone app (named contact) from my file exchange or mail me at qaziejazurrehman@gmail.com','Contact');
pause(5)
web('https://www.linkedin.com/pub/qazi-ejaz/87/813/967','-browser')
