close all; clear; clc

%%

x=1:0.2:10;
y=sin(x);

gui1();

for i=1:length(x)
    handles = guidata(gui1);
    
    index = num2str(i);
    
    set(handles.Xdata, 'String', index);
    
    Current.x=x(1:i);
    Current.y=y(1:i);
    
    handles.hPlot1 = plot(handles.axes1, NaN, NaN);
    set(handles.hPlot1, 'XData', [get(handles.hPlot1, 'XData') Current.x], ...
                        'YData', [get(handles.hPlot1, 'YData') Current.y], ...
                        'MarkerEdgeColor', 'r');
             
    drawnow;
end