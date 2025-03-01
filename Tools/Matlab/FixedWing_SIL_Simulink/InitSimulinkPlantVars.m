% Plant constants

%   Copyright 2022 The MathWorks, Inc.
InitFixedWingParam;
InitEnvironment;

% Initial states
init.posNED = [0, 0, 0]; % m
init.vb = [0 0 0]'; %m/s
init.euler = [0, 0, pi/3.5]'; %Roll Pitch Yaw Rads
init.angRates = [0, 0, 0]; %rad/s

%Reference location: Aeroclub RC Valencia (Cheste)
% This is the home position also

ref_lat = 39.49738807491014;
ref_lon =  -0.6268122488722822;
ref_height = 171;

%% UAV Dynamics Data Serialization Constants

%Gain to convert m to mm
m_to_mm = 1000;

%Gain to convert uT to Gauss
uT_to_gauss = 0.01;

%Gain to convert m/s^2 to mg
ms2_to_mg = (1/9.80665)*1000;

%Gain to convert m/s to cm/s
ms_to_cms = 100;

% Maximum Serial data read size from Pixhawk
MAVLink_Input_Read_Size = 1024;

% Sample Time of Plant and Controller (100 Hz)
SampleTime = 0.01;
