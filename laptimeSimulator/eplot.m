function  eplot(signals,Time ,varargin)
%EPLOT Summary of this function goes here
%   Detailed explanation goes here\
% xD lol all my homies hate detailed explanation
clf

if length(varargin) ==0
   start = 1;
   stop = length(signals(1,1).data);
else
    start = varargin(1);
    stop = varargin(2);
end

for signals_index = 1:size(signals,1)
    maxTiles = 10;
    myAxes(signals_index)=nexttile([1 maxTiles]);

    for singals_index2 = 1:size(signals,2)
        if ~isempty(signals(signals_index,singals_index2).name )
            if  isempty(signals(signals_index,singals_index2).time)
                signalTime = Time;
            else
                signalTime = signals(signals_index,singals_index2).time;
            end
            
             if length(varargin) == 0
                plot(signalTime,signals(signals_index,singals_index2).data(start:length(signals(signals_index,singals_index2).data)),'LineWidth',1.5)
            else
                    plot(signalTime,signals(signals_index,singals_index2).data(start:stop),'LineWidth',1.5)
            end
            grid on
            hold on
            legend(signals(signals_index,singals_index2).name,'Interpreter', 'none','Location','NE')
            %if singals_index2 == size(signals,2)
            legend(signals(signals_index,1:singals_index2).name,'Interpreter', 'none','Location','NE')
            ylabel(signals(signals_index,singals_index2).name,'Interpreter', 'none');
            %end
        end
    end

end
linkaxes(myAxes,'x')

grid on
end

