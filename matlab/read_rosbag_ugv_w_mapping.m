clear all;
close all;
clc;
 
bag = rosbag('D:\User\Videos\2024.3.Drone\2024-03-15-17-03-23.bag');
 
%%
ground_truth_uav1_select = select(bag, 'Time', [bag.StartTime bag.EndTime], 'Topic', '/vrpn_client_node/droneyee1/pose');

ground_truth_uav1 = readMessages(ground_truth_uav1_select);

t1_start = 0;
for i = 1 : size(ground_truth_uav1, 1)
    uav1_pose_index = ground_truth_uav1(i, 1);
    uav1_pose_index = uav1_pose_index{1, 1};
    
    if i == 1
        t1_start = uav1_pose_index.Header.Stamp.Sec + uav1_pose_index.Header.Stamp.Nsec / 1.e9;
    end
    uav1_pos_t(i, :) = uav1_pose_index.Header.Stamp.Sec + uav1_pose_index.Header.Stamp.Nsec / 1.e9;
    
    uav1_x(i, :) = uav1_pose_index.Pose.Position.X;
    uav1_y(i, :) = uav1_pose_index.Pose.Position.Y;
    uav1_z(i, :) = uav1_pose_index.Pose.Position.Z;

    [uav1_roll(i, :), uav1_pitch(i, :), uav1_yaw(i, :)] = quat2angle([uav1_pose_index.Pose.Orientation.X, uav1_pose_index.Pose.Orientation.Y, uav1_pose_index.Pose.Orientation.Z, uav1_pose_index.Pose.Orientation.W]);

end

%%
ground_truth_uav2_select = select(bag, 'Time', [bag.StartTime bag.EndTime], 'Topic', '/vrpn_client_node/droneyee2/pose');

ground_truth_uav2 = readMessages(ground_truth_uav2_select);

t2_start = 0;
for i = 1 : size(ground_truth_uav2, 1)
    uav2_pose_index = ground_truth_uav2(i, 1);
    uav2_pose_index = uav2_pose_index{1, 1};
    
    if i == 1
        t2_start = uav2_pose_index.Header.Stamp.Sec + uav2_pose_index.Header.Stamp.Nsec / 1.e9;
    end
    uav2_pos_t(i, :) = uav2_pose_index.Header.Stamp.Sec + uav2_pose_index.Header.Stamp.Nsec / 1.e9;
    
    uav2_x(i, :) = uav2_pose_index.Pose.Position.X;
    uav2_y(i, :) = uav2_pose_index.Pose.Position.Y;
    uav2_z(i, :) = uav2_pose_index.Pose.Position.Z;

    [uav2_roll(i, :), uav2_pitch(i, :), uav2_yaw(i, :)] = quat2angle([uav2_pose_index.Pose.Orientation.X, uav2_pose_index.Pose.Orientation.Y, uav2_pose_index.Pose.Orientation.Z, uav2_pose_index.Pose.Orientation.W]);

end

%%
ground_truth_uav3_select = select(bag, 'Time', [bag.StartTime bag.EndTime], 'Topic', '/vrpn_client_node/droneyee6/pose');

ground_truth_uav3 = readMessages(ground_truth_uav3_select);

t3_start = 0;
for i = 1 : size(ground_truth_uav3, 1)
    uav3_pose_index = ground_truth_uav3(i, 1);
    uav3_pose_index = uav3_pose_index{1, 1};
    
    if i == 1
        t3_start = uav3_pose_index.Header.Stamp.Sec + uav3_pose_index.Header.Stamp.Nsec / 1.e9;
    end
    uav3_pos_t(i, :) = uav3_pose_index.Header.Stamp.Sec + uav3_pose_index.Header.Stamp.Nsec / 1.e9;
    
    uav3_x(i, :) = uav3_pose_index.Pose.Position.X;
    uav3_y(i, :) = uav3_pose_index.Pose.Position.Y;
    uav3_z(i, :) = uav3_pose_index.Pose.Position.Z;

    [uav3_roll(i, :), uav3_pitch(i, :), uav3_yaw(i, :)] = quat2angle([uav3_pose_index.Pose.Orientation.X, uav3_pose_index.Pose.Orientation.Y, uav3_pose_index.Pose.Orientation.Z, uav3_pose_index.Pose.Orientation.W]);

end

%%
ground_truth_target_select = select(bag, 'Time', [bag.StartTime bag.EndTime], 'Topic', '/vrpn_client_node/rigidbody_8/pose');

ground_truth_target = readMessages(ground_truth_target_select);

t_tgt_start = 0;
for i = 1 : size(ground_truth_target, 1)
    target_pose_index = ground_truth_target(i, 1);
    target_pose_index = target_pose_index{1, 1};
    
    if i == 1
        t_tgt_start = target_pose_index.Header.Stamp.Sec + target_pose_index.Header.Stamp.Nsec / 1.e9;
    end
    target_pos_t(i, :) = target_pose_index.Header.Stamp.Sec + target_pose_index.Header.Stamp.Nsec / 1.e9;
    
    target_x(i, :) = target_pose_index.Pose.Position.X;
    target_y(i, :) = target_pose_index.Pose.Position.Y;
    target_z(i, :) = target_pose_index.Pose.Position.Z;

    [target_roll(i, :), target_pitch(i, :), target_yaw(i, :)] = quat2angle([target_pose_index.Pose.Orientation.X, target_pose_index.Pose.Orientation.Y, target_pose_index.Pose.Orientation.Z, target_pose_index.Pose.Orientation.W]);

end

%%
plot3(uav1_x, uav1_y, uav1_z)
hold on
plot3(uav2_x, uav2_y, uav2_z)
hold on
plot3(uav3_x, uav3_y, uav3_z)
hold on
plot3(target_x, target_y, target_z)

%%
save("../data/uav_data" + datestr(now, 30) + ".mat")