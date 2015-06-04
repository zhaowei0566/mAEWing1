%% setup.m
% UAV Software-in-the-Loop Simulation setup
%
% This script will setup the SIL simulation. Stored aircraft configuration
% and trim conditions are used.
%
% University of Minnesota
% Aerospace Engineering and Mechanics
% Copyright 2011 Regents of the University of Minnesota.
% All rights reserved.
%

%% Clean up
clear all
close all
bdclose all
clc

%% Add Libraries and controllers folder to MATLAB path
addpath ../Libraries
addpath ../Controllers
warning off Simulink:Engine:SaveWithParameterizedLinks_Warning
warning off Simulink:Commands:LoadMdlParameterizedLink 
warning off Simulink:ID:DuplicateSID

%% Load airframe configuration and trim condition
% To change these, use the functions "UAV_config.m" and "trim_UAV.m"
load UAV_modelconfig
load UAV_trimcondition

%% Simulation sample time
SampleTime = 0.02; % sec

%% Set controller variants
% Each variant corresponds to a different Simulink model that will be
% referenced in the "UAV_SIL/Control Software/Control Software" block.
flightcode_var = Simulink.Variant('controller_mode == 1 || controller_mode == 3 || controller_mode == 5 || controller_mode == 7 || controller_mode == 9');
baseline_control_var = Simulink.Variant('controller_mode == 2');

%% Set controller mode
% Use this variable to quickly change what controller is used in the
% simulation.
%
% 1 = baseline controller (C implementation)
controller_mode = 2;

% Load controller parameters or compile flight code
switch controller_mode
    case 1 % Baseline controller in C
        % Compile Flight Software:
        control_code_path = '../../Software/FlightCode/control/baseline_control.c';
        
    case 2 % Baseline controller in Simulink.
        baseline_gains;   % Declare baseline controller gains
        pitch_gains = [kp_PT, ki_PT, kp_PD];
        roll_gains = [kp_RT, ki_RT, kp_RD];
end

% Advanced Users: Include guidance, system ID, or fault injection codes.
% Set the path to the .c code in the following variables.
%
%%%%% GUIDANCE LAW %%%%%
% Point to the desired guidance code here. Use '-DSIMULINK_GUIDANCE' to
% input the reference commands from the simulink diagram.
% GUIDANCE = '../../Software/FlightCode/guidance/straight_level.c';
% GUIDANCE = '../../Software/FlightCode/guidance/doublet_phi_theta.c';
 GUIDANCE = '-DSIMULINK_GUIDANCE';

%%%%%% SYSTEM ID SELECTION %%%%%
% Point to the desired system ID code here
% SYSTEM_ID = '../../Software/FlightCode/system_id/chirp_sysid.c';
 SYSTEM_ID = '../../Software/FlightCode/system_id/systemid_none.c';

%%%%%% SURFACE FAULT MODE SELECTION %%%%%
% Point to the desired fault code here
% SURFACE_FAULT = '../../Software/FlightCode/faults/fault_onesurf.c';
% SURFACE_FAULT = '../../Software/FlightCode/faults/fault_onesurf_SingleStep.c';
 SURFACE_FAULT = '../../Software/FlightCode/faults/surffault_none.c';


%%%%%% SENS0R FAULT MODE SELECTION %%%%%
% Point to the desired fault code here
 SENSOR_FAULT = '../../Software/FlightCode/faults/sensfault_none.c';

% Compile control software
if exist('control_code_path','var')
    eval(['mex -I../../Software/FlightCode/ control_SIL.c  ' control_code_path...
                       ' ' GUIDANCE ' ' SYSTEM_ID ' ' SURFACE_FAULT ' ' SENSOR_FAULT ...
                       ' ../../Software/FlightCode/faults/fault_functions.c ' ...
                       ' ../../Software/FlightCode/system_id/systemid_functions.c ']);
end

%% Integer Time delay in flight software loop
IntegerTimeDelay = 2; % .04sec

%% Open sim diagram
UAV_SIL
