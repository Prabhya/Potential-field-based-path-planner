%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Vector field based path planner 2d
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc
clear

%%%%%% Workspace %%%%%%%%

%%%%%% Outer area %%%%%%%%%
H = [1 0;0 1;-1 0;0 -1];
h = [50;50;50;50];    % outer area defined using poytope
V = con2vert(H,h);
k = convhull(V);
plot(V(k,1),V(k,2),'k','LineWidth',1.5)
hold on 
grid on
axis([-51 51 -51 51])
xlabel('x-position')
ylabel('y-position')
%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%% Starting position %%%%%%%%
start = [-40;-40];    % start position can be changed
plot(start(1),start(2),'*b')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%% Goal/ target/ Final position %%%%%%%%%%
final = [40;40];      % final position can be changed
plot(final(1),final(2),'*k')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%% Obstacles %%%%%%%%%%%%%%%%%%%%%%%%%%%
%obstacle = [-50 + (50+50)*rand(1,15);-50 + (50+50)*rand(1,15)]; % 10 random point obstacles
obstacle1 = [0:0.1:10;10*ones(1,length(0:0.1:10))];     % case where it fails
obstacle2 = [10*ones(1,length(20:-0.1:10));10:-0.1:0];
obstacle = [obstacle1 obstacle2];
plot(obstacle(1,:),obstacle(2,:),'*r')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%% Basic potential field based planner %%%%%%%
dx = 0.5;  % move step

pos = start;  % setting up position
del = 100;
while(norm(final - pos) > dx) % condition to end planning
    repnet = 0; % repnet is net repulsive vector
    att = norm(final - pos); % magnitude of attraction based on distance
    rep = vecnorm(obstacle - pos); % distance between obstacle and position
    for i = 1:length(rep) % checking for obstacle within threshold distance
        if(rep(i)>5)
            rep(i) = 0;
        %else
         %   rep(i) = obstacle(:,i)-pos;
        end
        repnet = repnet + (obstacle(:,i)-pos)*rep(i);
    end
    
    if(norm(repnet)>0)  % normalizing the repulsive vector
        repnet = repnet/norm(repnet);
    end
    
    pos = pos + dx*(final-pos)/att- dx*(repnet); % + dx*[-1+(1+1)*rand;-1+(1+1)*rand]; % new position
    plot(pos(1,:),pos(2,:),'.b')
    del = norm(dx*(final-pos)/att- dx*(repnet));
    pause(0.05)
    if del <10^(-10)
        break;
    end
end    