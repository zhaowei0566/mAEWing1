%% Outputs GVT parameters
% This script contains the parameters of the experiment and saves them
% as a structure Info
% 
% Info has following fields
%   - Fs:= Sampling frequency of the experiment
%   - Run2Accel:= run-by-2 matrix with Run2Accel(i,j) providing
%         the accelerometer number for measurement j of run i.
%   - Accel2Run: Naccel-by-2 matrix with Accel2Run(i,1:2) providing
%         the [Run #, Measurement #] for accelometer i.
%   - AccelPos:= Naccel-by-2 matrix where  AccelPos(i,1) and
%         AccelPos(i,2) are the (X,Y) positions of accel i measured
%         in inches.  The coordinates are located at the aircraft nose
%         with Y pointing aft and X pointing left (Hence Z is up).
%   - ShakerPos:= 1-by-2 matrix where ShakerPos(1) and ShakerPos(2) are
%       the (X,Y) positions of the shaker attachment point.  The 
%       coordinates are located at the aircraft nose with Y pointing aft
%       and X pointing left (Hence Z is up).
%   - Nrun := Number of runs in the experiment
%   - Nmeas := Number of measurements in each run
%   - Naccel := Number of total acceelerations measurements
%   - Force_V2lb := Conversion factor from measured voltage to force (Lbs)
%   - Accel_V2g := Conversion factor from measured voltage to acceleration
%       (g)
%   - FreqWin+Hz := Target frequency window of the experiment
%   - ForwardAccel := Order of the accelerometers on the leading edge from
%      left wing tip to the right wing tip
%   - AftAccel := Order of the accelerometers on the trailing edge from the
%       left wing tip to the right wing tip

%% Sample Info
Fs = 2000;                      % Sampling freq, Hz

%% Calibrations
% Force: 0.4684 V/lb , Accels: 1.028 V/g
Force_V2lb = 0.4684;
Accel_V2g = 1.028;

%% Accel Mapping
% The data consists of Nrun experiments with each experiment providing
% Nmeas accelerometer measurements.  The total number of accelerometer
% measruements is thus Naccel=Nrun*Nmeas.   The accels are numbered
% from 1 to Naccel and the position of each accelerometer is provided
% in the array AccelPos below.  The mapping between accel numbering
% and experiment is provided by two variables:
%   A) Run2Accel := Nrun-by-2 matrix with Run2Accel(i,j) providing
%         the accelerometer number for measurement j of run i.
%   B) Accel2Run := Naccel-by-2 matrix with Accel2Run(i,1:2) providing
%         the [Run #, Measurement #] for accelometer i.
Run2Accel = [1 2; 3 4; 5 6; 7 8; 9 10; 11 12; 13 14; 15 16; 17 18; 19 20];

[Nrun,Nmeas] = size(Run2Accel);
Naccel= Nrun*Nmeas;
Accel2Run = zeros(Naccel,2);

for i=1:Naccel;
    [ridx,cidx]=find(Run2Accel==i);
    Accel2Run(i,:) = [ridx cidx];
end


%% Accel Positions
% As noted above, the accelerometer measurements recorded in all
% experimental runs are numbered from 1 to Naccel.  The positions
% of these accelerometers is provided by:
%     AccelPos := Naccel-by-2 matrix where  AccelPos(i,1) and
%         AccelPos(i,2) are the (X,Y) positions of accel i measured
%         in inches.  The coordinates are located at the aircraft nose
%         with Y pointing aft and X pointing left (Hence Z is up).


AccelPos =  [-17.87 19.03; -16.00 23.66;-27.14 22.77; -25.27 27.41;
    -36.42 26.52; -34.54 31.16; -45.69 30.27; -43.81 34.90;
    -54.96 34.01; -53.09 38.65; 17.87 19.03; 16.00 23.66;
    27.14 22.77; 25.27 27.41; 36.42 26.52; 34.54 31.16;
    45.69 30.27; 43.81 34.90; 54.96 34.01; 53.09 38.65];

%% Shaker Position
% The shaker position remains same for all the runs for this particular
% experiment.
%       ShakerPos := 1-by-2 matrix where ShakerPos(1) and ShakerPos(2) are
%       the (X,Y) positions of the shaker attachment point.  The 
%       coordinates are located at the aircraft nose with Y pointing aft
%       and X pointing left (Hence Z is up).
ShakerPos = [0 71.12];

%% Frequency Range
% The target frequency range for which the experiment was conducted. This
% is usually different from the actual range of the input frequencies
% because the aircraft needs to settle down to a steady state before a
% proper reading can be taken. For example for a traget range of  [3, 35]
% Hz, the actual input range might be [1, 35] Hz.


FreqWin_Hz = [3 35];

%% Accel order
% Defines the order of the accelerometers on the aircarft
%       ForwardAccel := Accelerometer positions on the leading edge from
%          the left wing tip to the right wing tip
%       AftAccel := Accelerometer positions on the trailing edge from the
%          left wing tip to the right wing tip

ForwardAccel = [19:-2:11 1:2:9];
AftAccel = [20:-2:12 2:2:10];

%% Store Experiment Info

Info.Fs = Fs;
Info.Run2Accel = Run2Accel;
Info.Accel2Run = Accel2Run;
Info.AccelPos = AccelPos;
Info.ShakerPos = ShakerPos;
Info.Nrun = Nrun;
Info.Nmeas = Nmeas;
Info.Naccel = Naccel;
Info.Force_V2lb = Force_V2lb;
Info.Accel_V2g = Accel_V2g;
Info.FreqWin_Hz = FreqWin_Hz;
Info.ForwardAccel = ForwardAccel;
Info.AftAccel = AftAccel;
