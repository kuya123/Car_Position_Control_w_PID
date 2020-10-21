## draw a simple car with X position as parameter only
function proj_PID_draw_simple_car(x)
# Car picture dimension ratio
M=10;  

% dimensions
% L = 2;  % pendulum length
W = 1*sqrt(M/5);  % cart width
H = .5*sqrt(M/5); % cart height
wr = .2; % wheel radius

% positions
% y = wr/2; % cart vertical position
y = wr/2+H/2; % cart vertical position
w1x = x-.9*W/2;
w1y = 0;
w2x = x+.9*W/2-wr;
w2y = 0;



h1=plot([-20 20],[0 0],'r','LineWidth',2);
hold on;
h2=rectangle('Position',[x-W/2,y-H/2,W,H],'Curvature',.1,'FaceColor',[1 0.1 0.1]);
h3=rectangle('Position',[w1x,w1y,wr,wr],'Curvature',1,'FaceColor',[0 0 0]);
h4=rectangle('Position',[w2x,w2y,wr,wr],'Curvature',1,'FaceColor',[0 0 0]);

flag=1;


% set(gca,'YTick',[])
% set(gca,'XTick',[])
xlim([-10 10]);
ylim([-2.5 2.5]);
set(gcf,'Position',[100 550 1000 400])
% box off
drawnow
hold off;
