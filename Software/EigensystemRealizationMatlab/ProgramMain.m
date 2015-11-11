% ====================================================================================================
% Start of program.
clear all ;
close all ;
fprintf(1,'====================================================================================================\n') ;
fprintf(1,'Start of program.\n') ;
% ====================================================================================================
% Set the files to read.
% Note that these files contain step response data.
ImpulseResponseDataFile = cellstr([ ...
    'ForcesMa0p100Alt0000BasisR6B2T2C8.txt' ; ...
    ]) ;
% Set the desired state-space system order.
StateSpaceSystemOrder = 2 ;
% Set the desired number of data points to skip.
NumberOfDataPointsToSkipAtStart = 0 ;
% Set the desired number of data points to skip.
NumberOfDataPointsToSkipAtEnd = 0 ;
% Set the desired number of data points to use.
NumberOfDataPointsToUse = 0 ;
% Run the analysis.
for CurrentCounter = 1 : 1 : length( ImpulseResponseDataFile( 1:1:end , 1 ) )
    % Get the current data file name.
    CurrentImpulseResponseDataFile = ImpulseResponseDataFile{CurrentCounter} ;
    % Import the data.
    %[CurrentData1,CurrentDataTime,CurrentData3,CurrentData4,CurrentData5,CurrentData6,CurrentDataLift] = importdata(CurrentImpulseResponseDataFile) ;
    CurrentData = importdata(CurrentImpulseResponseDataFile) ;
    CurrentDataTime = CurrentData.data(:,2) ;
    CurrentDataLift = CurrentData.data(:,7) ;
    % Remove the header line in the data.
    CurrentDataTime = CurrentDataTime( 2:1:end ) ;
    CurrentDataLift = CurrentDataLift( 2:1:end ) ;
    % Remove the first few lines in the data.
    CurrentDataTime = CurrentDataTime( (NumberOfDataPointsToSkipAtStart+1):1:(end-NumberOfDataPointsToSkipAtEnd) ) ;
    CurrentDataLift = CurrentDataLift( (NumberOfDataPointsToSkipAtStart+1):1:(end-NumberOfDataPointsToSkipAtEnd) ) ;
    % Compute the average lift.
    CurrentAverageLift = 0.0 ;
    for i = 1 : 1 : length(CurrentDataLift)
        CurrentAverageLift = CurrentAverageLift+CurrentDataLift(i) ;
    end
    CurrentAverageLift = CurrentAverageLift/length(CurrentDataLift) ;
    % Compute the adjusted lift.
    CurrentAdjustedLift = zeros(length(CurrentDataLift),1) ;
    for i = 1 : 1 : length(CurrentDataLift)
        CurrentAdjustedLift(i) = CurrentDataLift(i)-CurrentAverageLift ;
    end
    % Compute the current time step.
    CurrentTimeStep = (CurrentDataTime(end)-CurrentDataTime(2))/(length(CurrentDataTime)-2) ;
    % Compute the lift derivatives.
    % This converts the step response data into impulse response data.
    CurrentDataLiftDerivative = zeros(length(CurrentDataLift),1) ;
    CurrentDataLiftDerivative(1) = (CurrentDataLift(2)-CurrentDataLift(1))/CurrentTimeStep ;
    for i = 2 : 1 : (length(CurrentDataLiftDerivative)-1)
        CurrentDataLiftDerivative(i) = (CurrentDataLift(i+1)-CurrentDataLift(i-1))/(2*CurrentTimeStep) ;
    end
    CurrentDataLiftDerivative(end) = (CurrentDataLift(end)-CurrentDataLift(end-1))/CurrentTimeStep ;
    for i = 1 : 1 : length(CurrentDataLiftDerivative)
        CurrentDataLiftDerivative(i) = CurrentDataLiftDerivative(i) ; %*exp(-10.0*CurrentDataTime(i)) ;
    end
    % Call the eigensystem realization function.
    [ CurrentStateMatrixA , CurrentInputMatrixB , CurrentOutputMatrixC , CurrentFeedthroughMatrixD ] = computeEigensystemRealization(CurrentAdjustedLift,StateSpaceSystemOrder,NumberOfDataPointsToUse,CurrentTimeStep,1) ;
    % Assemble the state-space system.
    CurrentSystem = ss( CurrentStateMatrixA , CurrentInputMatrixB , CurrentOutputMatrixC , CurrentFeedthroughMatrixD , CurrentTimeStep ) ;
    [y,t] = impulse(CurrentSystem) ;
    % Call the damp function.
    fprintf(1,'====================================================================================================\n') ;
    fprintf(1,'%s\n',CurrentImpulseResponseDataFile) ;
    damp2( CurrentStateMatrixA , CurrentTimeStep ) ;
    % Plot.
    figure() ;
    plot(CurrentDataTime,CurrentAdjustedLift,'-k') ;
    title(CurrentImpulseResponseDataFile) ;
    xlabel('Time (seconds)') ;
    ylabel('Adjusted Lift Force (lb)') ;
    xlim([min(CurrentDataTime) max(CurrentDataTime)]) ;
    ylim([1.2*min(CurrentAdjustedLift) 1.2*max(CurrentAdjustedLift)]) ;
    grid on ;
    break ;
end
%clear Current* ;
% ====================================================================================================
fprintf(1,'====================================================================================================\n') ;
fprintf(1,'End of program.\n') ;
fprintf(1,'====================================================================================================\n') ;
% End of program.
% ====================================================================================================
