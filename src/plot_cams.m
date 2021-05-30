% Real time data collection example
%
% This script is implemented as a function so that it can
%   include sub-functions
%
% This script can be modified to be used on any platform by changing the
% serialPort variable.
% Example:-
% On Linux:     serialPort = '/dev/ttyS0';
% On MacOS:     serialPort = '/dev/tty.KeySerial1';
% On Windows:   serialPort = 'COM1';
%
%To run: 
%plot_cams()
%To reset ports: (if MATLAB still thinks they're busy)
%delete(instrfindall)
%

function plot_cams 
    %Send over bluetooth or serial
    serialPort = 'COM4';
    serialObject = serial(serialPort);
    %configure serial connection
    %serialObject.BaudRate = 115200; %(Default)
    serialObject.BaudRate = 9600;
    %serialObject.FlowControl = 'software';
    %serialObject.ReadAsyncMode = 'manual';

    %Initiate serial connection
    fopen(serialObject);

    % This gets called on cleanup to ensure the stream gets closed
    finishup = onCleanup(@() myCleanupFun(serialObject));

    % Instantiate variables
    count = 1;
    bytes = 0;
    trace = zeros(1, 128); %Stored Values for Raw Input

    while (1)
        % Check for data in the stream
        %readasync(serialObject);
        if serialObject.BytesAvailable
            val = fscanf(serialObject,'%i');
            if ((val == -1) || (val == -3) || (val == -5) || (val == -7)) % start keywords
                count = 1;
            elseif (val == -2) % End camera1 tx
                plotdata(trace, 0);
                trace = zeros(1,128); %Stored Values for Raw Input
                count = 1;
            elseif (val == -4) % End camera2 tx
                count = 1;
                plotdata(trace, 1);
                trace = zeros(1, 128); %Stored Values for Raw Input
            elseif (val == -6) % End camera2 tx
                count = 1;
                plotdata(trace, 2);
                trace = zeros(1, 128); %Stored Values for Raw Input
            elseif (val == -8) % End camera2 tx
                count = 1;
                plotdata(trace, 3);
                trace = zeros(1, 128); %Stored Values for Raw Input
            else
                trace(count) = val;
                count = count + 1;
            end % if
        end %bytes available    
    end % while(1)

    % Clean up the serial object
    fclose(serialObject);
    delete(serialObject);
    clear serialObject;

end %plot_cams

%*****************************************************************************************************************
%*****************************************************************************************************************

function plotdata(trace, cam)
    drawnow;
    subplot(4,1,cam+1);

    plot(trace);
    xl = xline(64);
    xl.LineWidth = 1;
    xl.Color = 'r';
    xl.Label = 'Center';
    xl.LabelVerticalAlignment = 'bottom';

    [m,i] = max(trace(1:64));
    y1 = yline(m-m/5);
    y1.LineWidth = 1;
    y1.Color = 'b';
    y1.Label = m;
    y1.LabelHorizontalAlignment = 'left';
    
    [m,i] = max(trace(65:128));
    y2 = yline(m-m/5);
    y2.LineWidth = 1;
    y2.Color = 'r';
    y2.Label = m;
    y2.LabelHorizontalAlignment = 'right';
        
    if cam == 0
        title("Raw Camera Trace");
    elseif cam == 1
        title("Smoothed Trace");
    elseif cam == 2
        title("Derivative Trace");
    else
        title("Trace");
    end
    
end %function

function myCleanupFun(serialObject)
    % Clean up the serial object
    fclose(serialObject);
    delete(serialObject);
    clear serialObject;
    delete(instrfindall);
end