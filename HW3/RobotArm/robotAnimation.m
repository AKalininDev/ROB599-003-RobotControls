%% Manipulator Animation
% Author: Jiefu Zhang
% Date: Feb 1 24
%
% Input: 
% t: time vector from simulation (ode45)
% x: state vector from simulation (ode45), should be N-4 vector
%
%%
function robotAnimation(t, x)

q1 = x(:,1); q2 = x(:,3);

robot = loadrobot("abbIrb120", "DataFormat","row");
config = homeConfiguration(robot);

q1_offset = -q1 + pi/2;
q2_offset = -q2 - pi/2;

t_animation = linspace(0, 4, 30*8);
q1_int = interp1(t, q1_offset, t_animation);
q2_int = interp1(t, q2_offset, t_animation);

rateObj = rateControl(30);

figure();

timeText = annotation('textbox', [0.7, 0.8, 0.1, 0.1]);
timeText.String = 'Time: 0s';

for i = 1 : length(t_animation)
    config = [0 q1_int(i) q2_int(i) 0 0 0];
    ax = show(robot, config, 'PreservePlot',false,'Frames','off','Collisions','off','Visuals','on','FastUpdate',true);

    ax.XLim = [-1 1];
    ax.YLim = [-0.5 0.5];
    ax.ZLim = [-0.25 1];
    timeText.String = ['Time: ' num2str(t_animation(i)) 's'];
    drawnow
     
    waitfor(rateObj);
end
