% University of Washington
% Mechanical Engineering
% Robotics, Vision, and Mechatronics for Manufacturing, 2021 Sp
% 
% required toolbox:
%   Robotics toolbox from Peter Corke.
%   Computer Vision toolbox from Mathworks (to use pose estimation function).
%   Statistics and Machine Learning Toolbox from Mathworks (to generate
%       Gaussian noise)

%% set camera model
camera.K = [1349.056772980567, 0, 774.9519749992618;
            0, 1351.558896220857, 557.5626838601945;
            0, 0, 1];
camera.img_size = [1600, 1200];
camera_param = cameraIntrinsics([camera.K(1,1), camera.K(2,2)],...
                                [camera.K(1,3), camera.K(2,3)], ...
                                camera.img_size);

%% environment setup
% featuer points' coordinates w.r.t the board frame
featurePositions = [-0.1, -0.1, 0.1, 0.1; 
                    -0.1, 0.1,  0.1, -0.1; 
                    0,    0,    0,   0];
         
% the desired pose of board w.r.t camera (4x4 matrix)
goal_camera2board_pose = transl(0,0,0.3)*trotx(pi);

% camera's initial pose w.r.t the board frame (4x4 matrix)
% don't use this vairable in your algorithm
% it is assumed unknown
init_board2camera_pose = transl(-0.5,-0.4,2.5)*trotz(1.6)*trotx(3.1)*troty(0.5);

%% simulation
step_size = 0.005;          % simulation step size in seconds
sim_time = 5;               % simulation duration in seconds
N_steps = sim_time/step_size;

% initialize the state object
% the state object describes all state variables at a time
state_obj = state(init_board2camera_pose, camera, featurePositions);
state_obj2 = state_obj;
state_obj3 = state_obj;
state_obj4 = state_obj;
state_obj5 = state_obj;
state_obj6 = state_obj;
state_obj7 = state_obj;
state_obj8 = state_obj;
state_obj9 = state_obj;

% we create a cell array to save state object at each time step.
data = cell([N_steps,1]);
data2 = data;
data3 = data;
data4 = data;
data5 = data;
data6 = data;
data7 = data;
data8 = data;
data9 = data;

% simulation start.
k = 0;
for t = step_size:step_size:sim_time
    k = k + 1;
    % Step 1: compute (by simulation) the image coordinates of projected 
    % feature points
    [state_obj, imgPoints] = state_obj.project();
    [state_obj2, imgPoints2] = state_obj2.project();
    [state_obj3, imgPoints3] = state_obj3.project();
    [state_obj4, imgPoints4] = state_obj4.project();
    [state_obj5, imgPoints5] = state_obj5.project();
    [state_obj6, imgPoints6] = state_obj6.project();
    [state_obj7, imgPoints7] = state_obj7.project();
    [state_obj8, imgPoints8] = state_obj8.project();
    [state_obj9, imgPoints9] = state_obj9.project();
    
    % Step 2: visual servo step to compute desired camera velocity.
    
    %===========================
    % Your code starts from here
    % You should obtain desired camera velocity (w.r.t to camera frame)
    % cam_vel (6x1 vector) based on the obtained feature points in the image.

    % ---- PBVS without pose estimation noises -----
    % A possible PBVS_step function is written at the end of this file to compute the camera velocity vector under a PBVS algorithm.
    lambda = 1;
    cam_vel = PBVS_step(imgPoints, camera,...
                        featurePositions, goal_camera2board_pose, lambda, 1);

    % ---- IBVS (using the goal depth) -----
    % This is an example code structure. You will need to write your own IBVS_step function.
    %cam_vel = IBVS_step(...)
    % test the case with different Z values
    cam_vel2 = IBVS_step(imgPoints2, camera, featurePositions,...
                        goal_camera2board_pose,state_obj2,lambda,1,1,camera_param);
    % ---- IBVS (using the true depth) -----
    %Z = state_obj.getFeatureDepth();
    %cam_vel = IBVS_step(...)
    cam_vel3 = IBVS_step(imgPoints3, camera, featurePositions,...
                        goal_camera2board_pose,state_obj3,lambda,0,1,camera_param);
    cam_vel4 = IBVS_step(imgPoints4, camera, featurePositions,...
                        goal_camera2board_pose,state_obj4,lambda,0,1.1,camera_param);
    cam_vel5 = IBVS_step(imgPoints5, camera, featurePositions,...
                        goal_camera2board_pose,state_obj5,lambda,0,1.3,camera_param);
    cam_vel6 = IBVS_step(imgPoints6, camera, featurePositions,...
                        goal_camera2board_pose,state_obj6,lambda,0,1.5,camera_param);
    cam_vel7 = IBVS_step(imgPoints7, camera, featurePositions,...
                        goal_camera2board_pose,state_obj7,lambda,0,2,camera_param);                
    % ---- PBVS and IBVS with noises in pose estimation -----
    %
    imgPoints_noisy = imgPoints8 + normrnd(0, 5, size(imgPoints));
    imgPoints_noisy2 = imgPoints9 + normrnd(0, 5, size(imgPoints));
    cam_vel8 = PBVS_step(imgPoints_noisy, camera, featurePositions, goal_camera2board_pose, lambda, 1);
    cam_vel9 = IBVS_step(imgPoints_noisy2, camera, featurePositions, goal_camera2board_pose,state_obj9,lambda,1,2,camera_param);
    % your code ends here.
    %====================
    
    % Step 3: update camera position. We assume a perfect motion
    % controller. That is, the actual camera velocity equals to the desired
    % camera velocity given by the visual servo step. We assume that the 
    % camera is moving with a constant speed during the step_size time.
    state_obj = state_obj.step(cam_vel, step_size);
    state_obj2 = state_obj2.step(cam_vel2, step_size);
    state_obj3 = state_obj3.step(cam_vel3, step_size);
    state_obj4 = state_obj4.step(cam_vel4, step_size);
    state_obj5 = state_obj5.step(cam_vel5, step_size);
    state_obj6 = state_obj6.step(cam_vel6, step_size);
    state_obj7 = state_obj7.step(cam_vel7, step_size);
    state_obj8 = state_obj8.step(cam_vel8, step_size);
    state_obj9 = state_obj9.step(cam_vel9, step_size);
    
    % save current state. 
    data{k} = state_obj;
    data2{k} = state_obj2;
    data3{k} = state_obj3;
    data4{k} = state_obj4;
    data5{k} = state_obj5;
    data6{k} = state_obj6;
    data7{k} = state_obj7;
    data8{k} = state_obj8;
    data9{k} = state_obj9;
end
% simulation end.

section = 'Part A'
imgplot(data, section)
ccplot(data, featurePositions,section)
peplot(N_steps, data, goal_camera2board_pose, section)
velplot(N_steps, data, section)

section = 'Part B.i'
imgplot(data2, section)
ccplot(data2, featurePositions,section)
peplot(N_steps, data2, goal_camera2board_pose, section)
velplot(N_steps, data2, section)

section = 'Part B.ii'
imgplot(data3, section)
ccplot(data3, featurePositions,section)
peplot(N_steps, data3, goal_camera2board_pose, section)
velplot(N_steps, data3, section)

section = 'Part B.ii @ Zo = 1.1'
imgplot(data4, section)
ccplot(data4, featurePositions,section)
peplot(N_steps, data4, goal_camera2board_pose, section)
velplot(N_steps, data4, section)

section = 'Part B.ii @ Zo = 1.3'
imgplot(data5, section)
ccplot(data5, featurePositions,section)
peplot(N_steps, data5, goal_camera2board_pose, section)
velplot(N_steps, data5, section)

section = 'Part B.ii @ Zo = 1.5'
imgplot(data6, section)
ccplot(data6, featurePositions,section)
peplot(N_steps, data6, goal_camera2board_pose, section)
velplot(N_steps, data6, section)

section = 'Part B.ii @ Zo = 2.0'
imgplot(data7, section)
ccplot(data7, featurePositions,section)
peplot(N_steps, data7, goal_camera2board_pose, section)
velplot(N_steps, data7, section)

section = 'Part C - PBVS'
imgplot(data8, section)
ccplot(data8, featurePositions,section)
peplot(N_steps, data8, goal_camera2board_pose, section)
velplot(N_steps, data8, section)

section = 'Part C - IBVS'
imgplot(data9, section)
ccplot(data9, featurePositions,section)
peplot(N_steps, data9, goal_camera2board_pose, section)
velplot(N_steps, data9, section)

%% helper function
% Feel free to construct helper functions and put them here.

function cam_vel = PBVS_step(imgPoints, camera, featurePositions,...
                        goal_camera2board_pose, lambda, method)
    % pose estimation
    camera_param = cameraIntrinsics([camera.K(1,1), camera.K(2,2)],...
                                    [camera.K(1,3), camera.K(2,3)], ...
                                     camera.img_size);
    [board2camera_R, board2camera_t] = estimateWorldCameraPose(imgPoints',...
                                         featurePositions', camera_param,...
                                         'Confidence', 99.99,...
                                         'MaxReprojectionError', 1000);
    camera2board_pose = [board2camera_R, -board2camera_R*board2camera_t'; ...
                         0 0 0 1];
    % pbvs
    cam_vel = lambda * pbvs(camera2board_pose, goal_camera2board_pose, method);
end

function command_vel = pbvs(c2t, desired_c2t, method)
    % c2t is the pose from camera to target object
    % desired_c2t is the desired pose from camera to target object

    cstar2c = desired_c2t/c2t;
    R = cstar2c(1:3,1:3);   % rotation matrix
    t = cstar2c(1:3,4);     % translation matrix
    t_c2t = c2t(1:3,4);
    t_desired_c2t = desired_c2t(1:3,4);
    axang = rotm2axang(R);
    v = axang(1:3)';
    theta = axang(4);
    if method == 1
        v_c = -R'*t;
        omega_c = -theta*v;
    else
        % code here for other velocity control laws
    end

    command_vel = [v_c;omega_c];

        function S = skew(V)
            S = [0, -V(3), V(2); V(3), 0, -V(1); -V(2), V(1) 0];
        end
end
function cam_vel = IBVS_step(imgPoints, camera, featurePositions,...
                        goal_camera2board_pose,state_obj,lambda,TD,n,camera_param)
% Determine True or Fixed Depth
    if TD == 1
        Z = state_obj.getFeatureDepth(); % True Depth
    else
        Z = n*ones(4)*0.3; % Fixed Depth
    end

Ps = cameraProjection(camera.K,goal_camera2board_pose(1:3,1:3),...
    goal_camera2board_pose(1:3,4),featurePositions);

    P_t = Ps-imgPoints;

    P = [P_t(1,1);P_t(2,1);P_t(1,2);P_t(2,2);P_t(1,3);P_t(2,3);P_t(1,4);P_t(2,4)];
    J = visjac_p(camera_param, imgPoints, Z);
    cam_vel = lambda*pinv(J)*P;
    
end
function J = visjac_p(camera_param, uv, Z)

    cam_u0 = camera_param.PrincipalPoint(1); % prinicipal points
    cam_v0 = camera_param.PrincipalPoint(2); % prinicipal points
    cam_rhou_f = camera_param.FocalLength(1);
    cam_rhov_f = camera_param.FocalLength(2);
    if numcols(uv) > 1
        J = [];
        if length(Z) == 1
            % if depth is a scalar, assume same for all points
            Z = repmat(Z, 1, numcols(uv));
        end
        % recurse for each point
        for i=1:numcols(uv)
            J = [J; visjac_p(camera_param, uv(:,i), Z(i))];
        end
        return;
    end
    
    % convert to normalized image-plane coordinates
    x = (uv(1) - cam_u0) /cam_rhou_f;
    y = (uv(2) - cam_v0) /cam_rhov_f;

    J = [
        1/Z, 0, -x/Z, -x*y, (1+x^2), -y
        0, 1/Z, -y/Z, -(1+y^2), x*y, x
        ];

    J = -1 * diag(camera_param.FocalLength) * J;
end

function imgplot(data, section)
% Plot image plane trajectory
    figure, hold on
    num_data = size(data,1);    
    for i = 1:num_data
        imgPoints = data{i}.imagePoints;
        plot(imgPoints(1,:),imgPoints(2,:),'.b');
        plot(mean(imgPoints(1,:)), mean(imgPoints(2,:)), '.r');
        % Red line is the trajectory of the feature center.
        axis([0 1600 0 1200]);
    end
    title('Feature Trajectory in the Image Plane',section)
end 

function ccplot(data, featurePositions,section)
%% % Plot camera center trajectory % %
num_data = size(data,1);
camera_central.x = zeros(num_data,1);
camera_central.y = zeros(num_data,1);
camera_central.z = zeros(num_data,1);
camera_central.time = zeros(num_data,1);
for i = 1:num_data
    cameraPose = data{i}.cameraPose;
    camera_central.x(i) = cameraPose(1,4);
    camera_central.y(i) = cameraPose(2,4);
    camera_central.z(i) = cameraPose(3,4);
    camera_central.time(i) = data{i}.time;
end
figure,
hold on
plot3(camera_central.x, camera_central.y, camera_central.z);
hold on
plot3(featurePositions(1,:), featurePositions(2,:), featurePositions(3,:), 'or');
xlabel('x');
ylabel('y');
zlabel('z');
axis([-1 1 -1 1 0 4])
title('camera central trajectory', section);
view(27,37)
axis equal
xlim([-1 1])
ylim([-1 1])
zlim([-0.5 4])
        
end

function peplot(N_steps, data, goal_camera2board_pose, section)
%% plot pose error vs time
num_data = size(data,1);
rotation_error = zeros(N_steps, 1);
translation_error = zeros(N_steps,1);
time = zeros(N_steps,1);
for i = 1:num_data
    cam_pose = data{i}.cameraPose;
    error_pose = goal_camera2board_pose * cam_pose;
    
    translation_error(i) = norm(tform2trvec(error_pose));
    angaxis = tform2axang(error_pose);
    rotation_error(i) = abs(angaxis(4));
    time(i) = data{i}.time;
end
figure,
hold on
yyaxis left
plot(time, rotation_error*180/pi);
set(gca, 'YScale', 'log')
ylabel('Rotational error (degree)')
yyaxis right
plot(time, translation_error);
set(gca, 'YScale', 'log')
ylabel('Translational error (meter)');
xlabel('Time (seconds)')
title('pose error vs. time', section);
end

function velplot(N_steps, data, section)
%% plot camera velocity
figure,
cam_velocity.time = zeros(N_steps, 1);
cam_velocity.l.x = zeros(N_steps, 1);
cam_velocity.l.y = zeros(N_steps, 1);
cam_velocity.l.z = zeros(N_steps, 1);
cam_velocity.a.x = zeros(N_steps, 1);
cam_velocity.a.y = zeros(N_steps, 1);
cam_velocity.a.z = zeros(N_steps, 1);
for i = 1:N_steps
    cam_vel = data{i}.cam_vel;
    cam_velocity.time(i) = data{i}.time;
    cam_velocity.l.x(i) = cam_vel(1);
    cam_velocity.l.y(i) = cam_vel(2);
    cam_velocity.l.z(i) = cam_vel(3);
    cam_velocity.a.x(i) = cam_vel(4);
    cam_velocity.a.y(i) = cam_vel(5);
    cam_velocity.a.z(i) = cam_vel(6);
end
subplot(1,2,1)
plot(cam_velocity.time, cam_velocity.l.x);
hold on
plot(cam_velocity.time, cam_velocity.l.y);
plot(cam_velocity.time, cam_velocity.l.z);
xlabel('time(sec)')
ylabel('velocity(m/s)');
title('camera linear velocity', section);
legend('x','y','z','Location','best');
subplot(1,2,2)
plot(cam_velocity.time, cam_velocity.a.x);
hold on
plot(cam_velocity.time, cam_velocity.a.y);
plot(cam_velocity.time, cam_velocity.a.z);
xlabel('time(sec)')
ylabel('velocity(rad/s)');
title('camera angular velocity', section);
legend('x','y','z','Location','best');
end
