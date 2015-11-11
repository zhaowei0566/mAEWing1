function [ StateMatrixA , InputMatrixB , OutputMatrixC , FeedthroughMatrixD ] = computeEigensystemRealization( ImpulseResponseData , StateSpaceSystemOrder , NumberOfDataPointsToUse , TimeStep , DiscreteOrContinuous )
% ====================================================================================================
% This is an Eigensystem Realization Algorithm (ERA).
% Works for single-input single-output systems.
% Needs the control toolbox.
% Reference: Juang, J. N. and Phan, M. Q. "Identification and Control of Mechanical Systems", Cambridge University Press, 2001.
% Inputs:
%   ImpulseResponseData:
%       Discrete-time impulse response.
%   StateSpaceSystemOrder:
%       Desired order of the state-space system. Must be a positive integer.
%   NumberOfDataPointsToUse:
%       Number of data points to use. Set to 0 to use all the data.
%   TimeStep:
%       Time step of the discrete-time impulse response.
%   DiscreteOrContinuous:
%        If = 1, the output is the discrete-time state-space model.
%        If = 2, the output is the continuous-time state-space model. This does not work in Octave.
% Outputs:
%   [ StateMatrixA , InputMatrixB , OutputMatrixC , FeedthroughMatrixD ]: State-space model matrices.
% ====================================================================================================
% Check the input.
StateSpaceSystemOrder = ceil( StateSpaceSystemOrder ) ;
if ( StateSpaceSystemOrder < 1 )
    StateSpaceSystemOrder = 1 ;
end
if ( NumberOfDataPointsToUse <= 0 )
    NumberOfDataPointsToUse = length( ImpulseResponseData )-2 ;
end
if ( TimeStep <= 0.0 )
    error('The inputted time step must be a positive and non-zero number.') ;
end
if ( ( DiscreteOrContinuous ~= 1 ) && ( DiscreteOrContinuous ~= 2 ) )
    error('The inputted discrete or continuous flag must have a value of either 1 or 2.') ;
end
% Create the Hankel matrices.
HankelMatrix0 = hankel( ImpulseResponseData( 2:1:(NumberOfDataPointsToUse+1) ) ) ;
HankelMatrix1 = hankel( ImpulseResponseData( 3:1:(NumberOfDataPointsToUse+2) ) ) ;
% Factor of the Hankel matrix using a singular value decomposition.
% MatrixR and MatrixS are orthonormal and SingularValuesMatrix is a rectangular matrix.
[ MatrixR , SingularValuesMatrix , MatrixS ] = svd( HankelMatrix0 ) ;   
% Truncate the singular values to the desired order.
SingularValuesMatrixTruncated = SingularValuesMatrix( 1:1:StateSpaceSystemOrder , 1:1:StateSpaceSystemOrder ) ;            
% Create the observability matrix.
ObservabilityMatrix = MatrixR( 1:1:end , 1:1:StateSpaceSystemOrder )*SingularValuesMatrixTruncated^(0.5) ;
% Create the controllability matrix.
ControllabilityMatrix = SingularValuesMatrixTruncated^(0.5)*MatrixS( 1:1:end , 1:1:StateSpaceSystemOrder )' ;
% Create the state matrix.
StateMatrixA = SingularValuesMatrixTruncated^(-0.5)*(MatrixR( 1:1:end , 1:1:StateSpaceSystemOrder )')*HankelMatrix1*MatrixS( 1:1:end , 1:1:StateSpaceSystemOrder )*SingularValuesMatrixTruncated^(-0.5) ;
% Create the input matrix.
InputMatrixB = ControllabilityMatrix( 1:1:end , 1 ) ;
% Create the output matrix.
OutputMatrixC = ObservabilityMatrix( 1 , 1:1:end ) ;
% Create the feedthrough matrix.
FeedthroughMatrixD = ImpulseResponseData(1) ;
% Check if the continuous system is asked-for.
if ( DiscreteOrContinuous == 2 )
    % Create the discrete-time system.
    SystemDiscrete = ss( StateMatrixA , InputMatrixB , OutputMatrixC , FeedthroughMatrixD , TimeStep ) ;
    % Convert the discrete LTI model to continuous time.
    SystemContinuous = d2c( SystemDiscrete , 'zoh' ) ;
    % Create the continuous-time system.
    [ StateMatrixA , InputMatrixB , OutputMatrixC , FeedthroughMatrixD ] = ssdata( SystemContinuous ) ;
end
% End of function.
% ====================================================================================================
end
