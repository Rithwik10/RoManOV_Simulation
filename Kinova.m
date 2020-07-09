robot = loadrobot('kinovaGen3','DataFormat','column','Gravity',[0 0 -9.81]);
currentRobotJConfig = homeConfiguration(robot);
endEffector = "EndEffector_Link";
points=0.1*(([repmat(-3,140,1) repmat(5,140,1) repmat(4,140,1)])+([repmat(1,11,1) (2:-0.2:0)' repmat(0,11,1);repmat(1,6,1) (0:0.2:1)' repmat(0,6,1);(1:0.2:2)' repmat(1,6,1)  repmat(0,6,1);repmat(2,6,1) (1:-0.2:0)' repmat(0,6,1);(2:0.2:2.5)' repmat(0,3,1)  repmat(0,3,1);repmat(2.5,6,1) (0:0.2:1)' repmat(0,6,1);(2.5:0.2:3.5)' repmat(1,6,1)  repmat(0,6,1);repmat(3.5,3,1) (1:-0.2:0.5)' repmat(0,3,1);(3.5:-0.2:2.5)' repmat(0.5,6,1)  repmat(0,6,1);repmat(2.5,3,1) (0.5:-0.2:0)' repmat(0,3,1);(2.5:0.2:4)' repmat(0,8,1)  repmat(0,8,1);repmat(4,11,1) (0:0.2:2)' repmat(0,11,1);repmat(4,11,1) (2:-0.2:0)' repmat(0,11,1);(4:0.2:5)' repmat(0,6,1)  repmat(0,6,1);repmat(5,11,1) (0:0.2:2)' repmat(0,11,1);repmat(5,11,1) (2:-0.2:0)' repmat(0,11,1);(5:0.2:6.5)' repmat(0,8,1)  repmat(0,8,1);repmat(6.5,6,1) (0:0.2:1)' repmat(0,6,1);(6.5:-0.2:5.5)' repmat(1,6,1)  repmat(0,6,1);repmat(5.5,6,1) (1:-0.2:0)' repmat(0,6,1)]));
count = length(points);
q0 = homeConfiguration(robot);
ndof = length(q0);
qs = zeros(count, ndof);
ik = inverseKinematics('RigidBodyTree', robot);
weights = [0, 0, 0, 1, 1, 1];
qInitial = q0; % Use home configuration as the initial guess
for i = 1:count
    % Solve for the configuration satisfying the desired end effector
    % position
    point=points(i,:);
    qSol = ik(endEffector,trvec2tform(point),weights,qInitial);
    % Store the configuration
    qs(i,:) = qSol;
    % Start from prior solution
    qInitial = qSol;
end
figure
show(robot,qs(1,:)');
view(2)
ax = gca;

hold on
plot3(points(:,1),points(:,2),points(:,3),'k')
axis([-1 1 -1 1])
framesPerSecond = 15;
r = rateControl(framesPerSecond);
for i = 1:count
    show(robot,qs(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
end

