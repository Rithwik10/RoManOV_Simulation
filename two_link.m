robot = rigidBodyTree('DataFormat','column','MaxNumBodies',3);
L1 = 4;
L2 = 4;
body = rigidBody('link1');
joint = rigidBodyJoint('joint1', 'revolute');
setFixedTransform(joint,trvec2tform([0 0 0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'base');
body = rigidBody('link2');
joint = rigidBodyJoint('joint2','revolute');
setFixedTransform(joint, trvec2tform([L1,0,0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'link1');
body = rigidBody('tool');
joint = rigidBodyJoint('fix1','fixed');
setFixedTransform(joint, trvec2tform([L2, 0, 0]));
body.Joint = joint;
addBody(robot, body, 'link2');
points=[repmat(1,11,1) (2:-0.2:0)' repmat(0,11,1);repmat(1,6,1) (0:0.2:1)' repmat(0,6,1);(1:0.2:2)' repmat(1,6,1)  repmat(0,6,1);repmat(2,6,1) (1:-0.2:0)' repmat(0,6,1);(2:0.2:2.5)' repmat(0,3,1)  repmat(0,3,1);repmat(2.5,6,1) (0:0.2:1)' repmat(0,6,1);(2.5:0.2:3.5)' repmat(1,6,1)  repmat(0,6,1);repmat(3.5,3,1) (1:-0.2:0.5)' repmat(0,3,1);(3.5:-0.2:2.5)' repmat(0.5,6,1)  repmat(0,6,1);repmat(2.5,3,1) (0.5:-0.2:0)' repmat(0,3,1);(2.5:0.2:4)' repmat(0,8,1)  repmat(0,8,1);repmat(4,11,1) (0:0.2:2)' repmat(0,11,1);repmat(4,11,1) (2:-0.2:0)' repmat(0,11,1);(4:0.2:5)' repmat(0,6,1)  repmat(0,6,1);repmat(5,11,1) (0:0.2:2)' repmat(0,11,1);repmat(5,11,1) (2:-0.2:0)' repmat(0,11,1);(5:0.2:6.5)' repmat(0,8,1)  repmat(0,8,1);repmat(6.5,6,1) (0:0.2:1)' repmat(0,6,1);(6.5:-0.2:5.5)' repmat(1,6,1)  repmat(0,6,1);repmat(5.5,6,1) (1:-0.2:0)' repmat(0,6,1)];
count = length(points);
q0 = homeConfiguration(robot);
ndof = length(q0);
qs = zeros(count, ndof);
ik = inverseKinematics('RigidBodyTree', robot);
weights = [0, 0, 0, 1, 1, 0];
endEffector = 'tool';
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
ax.Projection = 'orthographic';
hold on
plot(points(:,1),points(:,2),'k')
axis([-1 10 -1 10])
framesPerSecond = 15;
r = rateControl(framesPerSecond);
for i = 1:count
    show(robot,qs(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
end
