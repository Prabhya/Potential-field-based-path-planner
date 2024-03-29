%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Vector field based path planner 2d
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc
clear
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This planner was written based on potential field based path planning (ppbfp)
% This program does not follow to theory faithfully and is a different outlook on the same path planning
% The ppbfp will be later implemented using this program style as the basis
% A similar program will be written in python version for learning and understanding
% -Prabhjeet Singh Arora
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%% Workspace %%%%%%%%

dx = 0.5; % move step

%%%%%% Outer area %%%%%%%%%
H = [1 0;0 1;-1 0;0 -1];
h = [50;50;50;50];    % outer area defined using poytope
V = con2vert(H,h);
k = convhull(V);

%%%%% Outer Area plot %%%%%%%
plot(V(k,1),V(k,2),'k','LineWidth',1.5)
%%%%% Plot Styling %%%%%%%
hold on 
grid on
axis([-51 51 -51 51])
xlabel('x-position','FontSize',18)
ylabel('y-position','FontSize',18)
title('Path Planning','FontSize',20)
%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%% Starting position %%%%%%%%
start = [-40;-40];    % start position can be changed
plot(start(1),start(2),'ob')
text(start(1),start(2),'Start','VerticalAlignment','bottom','HorizontalAlignment','right')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%% Goal/ target/ Final position %%%%%%%%%%
final = [40;40];      % final position can be changed
plot(final(1),final(2),'ok')
text(final(1),final(2),'goal','VerticalAlignment','bottom','HorizontalAlignment','right')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%% Obstacles %%%%%%%%%%%%%%%%%%%%%%%%%%%
th = 5;
numobs = 15;                                                % Number of point obstacles
obstacle = [-40 + (30+40)*rand(2,numobs)];                  % random point obstacles
%obstacle1 = [0:0.1:10;10*ones(1,length(0:0.1:10))];        % case where it fails
%obstacle2 = [10*ones(1,length(20:-0.1:10));10:-0.1:0];
%obstacle = [obstacle1 obstacle2];
plot(obstacle(1,:),obstacle(2,:),'*r')
viscircles(obstacle',th*ones(length(obstacle),1))
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%% Basic potential field based planner %%%%%%%

pos = start;                                     % setting up position
del = 100;
while(norm(final - pos) > dx)                    % condition to end planning
    repnet = 0;                                  % repnet is net repulsive vector
    att = norm(final - pos);                     % magnitude of attraction based on distance
    rep = vecnorm(obstacle - pos);               % distance between obstacle and position
    for i = 1:length(rep)                        % checking for obstacle within threshold distance
        if(rep(i)>th)
            rep(i) = 0;
        else
            repnet = repnet + (obstacle(:,i)-pos)/rep(i);
        end
    end
    
    if(norm(repnet)>0)                           % normalizing the repulsive vector
        repnet = repnet/norm(repnet);
    end
    
    pos = pos + dx*(final-pos)/att- dx*(repnet) + (dx/10)*[-1+(1+1)*rand(2,1)]; % new position
    plot(pos(1,:),pos(2,:),'.b')
    del = norm(dx*(final-pos)/att- dx*(repnet));
    pause(0.05)
    if del <10^(-10)
        break;
    end
end    