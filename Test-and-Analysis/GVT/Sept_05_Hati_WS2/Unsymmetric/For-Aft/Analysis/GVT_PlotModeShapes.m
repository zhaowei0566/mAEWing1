function [] = GVT_PlotModeShapes(Info,Modal,ModeShapes)

Nmode = Modal.Nmode;
Nrun = Info.Nrun;
Naccel = Info.Naccel;
ForwardAccel = Info.ForwardAccel;
AccelPos = Info.AccelPos;
xcoord = AccelPos(:,1);             % X-coordinates of accelerometers
ycoord = AccelPos(:,2);             % Y-coordiantes of accelerometers

PlotOrder = ForwardAccel;


%% Plot mode shapes

% 2-D plots
heave = zeros(Nmode,Naccel);

for i = 1:Nmode
    
    heave(i,:) = ModeShapes(ForwardAccel,i);
    figure
    hold on
    plot(AccelPos(ForwardAccel,1),heave(i,:)*4,'b')
    legend('Heave')
    title(['Mode shape for mode:',num2str(i),...
        ', Freq:',num2str(Modal.w_mode(i)),'rad/s']);
    grid on
    garyfyFigure
end