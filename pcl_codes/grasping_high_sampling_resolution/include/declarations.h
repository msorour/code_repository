
#ifndef __DECLARATIONS_H__
#define __DECLARATIONS_H__

clock_t begin, begin2, begin3, begin4, begin5, end;
double time_elapsed_checking_gripper_collision_with_table        = 0.0;
double average_time_to_check_gripper_collision_with_table        = 0.0;
double time_elapsed_checking_gripper_collision_with_object       = 0.0;
double average_time_to_check_gripper_collision_with_object       = 0.0;
double time_elapsed_checking_object_contact_with_gripper_support = 0.0;


clock_t time_to_load_clouds_begin;
clock_t initial_overhead_begin;
clock_t time_to_run_algorithm_begin;
double time_spent;
double time_spent_for_initial_overhead;
std::string gripper_model;

long int sampling_iteration = 0;
double metric_1_score = 0.0, metric_2_score = 0.0, metric_3_score = 0.0, metric_4_score = 0.0, metric_5_score = 0.0, metric_6_score = 0.0, metric_7_score = 0.0, metric_8_score = 0.0, total_score = 0.0, total_score_best = 0.0;
double metric_1_score_best = 0.0, metric_2_score_best = 0.0, metric_3_score_best = 0.0, metric_4_score_best = 0.0, metric_5_score_best = 0.0, metric_6_score_best = 0.0, metric_7_score_best = 0.0, metric_8_score_best = 0.0;

// point clouds declarations
pcl::PointCloud<pcl::PointXYZ>::Ptr     object_cloud_in_camera_depth_optical_frame_xyz                       (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     object_cloud_in_arm_hand_frame_xyz                                   (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     object_cloud_transformed_in_gripper_frame_xyz                        (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     object_cloud_transformed_in_arm_hand_frame_xyz                       (new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr     object_sampling_in_object_frame_xyz                                  (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     object_sampling_in_arm_hand_frame_xyz                                (new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr     object_cloud_downsampled_in_camera_depth_optical_frame_xyz           (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     object_cloud_downsampled_in_gripper_frame_xyz                        (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     object_cloud_downsampled_in_arm_hand_frame_xyz                       (new pcl::PointCloud<pcl::PointXYZ>);


pcl::PointCloud<pcl::PointXYZ>::Ptr     object_plane_cloud_in_camera_depth_optical_frame_xyz                 (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     object_plane_cloud_downsampled_in_camera_depth_optical_frame_xyz     (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     object_plane_cloud_downsampled_in_gripper_frame_xyz                  (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     object_plane_cloud_downsampled_in_arm_hand_frame_xyz                 (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     object_plane_cloud_downsampled_in_object_plane_frame_xyz             (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     object_plane_special_ellipsoid_point_cloud_in_object_plane_frame     (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     object_plane_special_ellipsoid_point_cloud_in_arm_hand_frame         (new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr     gripper_cloud_transformed_in_arm_hand_frame_xyz                      (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     gripper_cloud_transformed_in_object_plane_frame_xyz                  (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     gripper_cloud_downsampled_in_arm_hand_frame_xyz                      (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     gripper_cloud_downsampled_in_gripper_frame_xyz                       (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     gripper_as_set_of_special_ellipsoids_in_gripper_frame                (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     gripper_as_set_of_special_ellipsoids_in_arm_hand_frame               (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     gripper_as_set_of_special_ellipsoids_transformed_in_arm_hand_frame   (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     gripper_support_point_cloud_in_gripper_frame                         (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     gripper_support_point_cloud_in_arm_hand_frame                        (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     gripper_support_point_cloud_in_gripper_transformed_frame             (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     gripper_support_point_cloud_transformed_in_arm_hand_frame            (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     gripper_support_point_cloud_in_gripper_frame_1                       (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     gripper_support_point_cloud_in_arm_hand_frame_1                      (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     gripper_support_point_cloud_in_gripper_transformed_frame_1           (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     gripper_support_point_cloud_transformed_in_arm_hand_frame_1          (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     gripper_support_point_cloud_in_gripper_frame_2                       (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     gripper_support_point_cloud_in_arm_hand_frame_2                      (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     gripper_support_point_cloud_in_gripper_transformed_frame_2           (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     gripper_support_point_cloud_transformed_in_arm_hand_frame_2          (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     gripper_support_point_cloud_in_gripper_frame_3                       (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     gripper_support_point_cloud_in_arm_hand_frame_3                      (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     gripper_support_point_cloud_in_gripper_transformed_frame_3           (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     gripper_support_point_cloud_transformed_in_arm_hand_frame_3          (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     gripper_support_point_cloud_in_gripper_frame_4                       (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     gripper_support_point_cloud_in_arm_hand_frame_4                      (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     gripper_support_point_cloud_in_gripper_transformed_frame_4           (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     gripper_support_point_cloud_transformed_in_arm_hand_frame_4          (new pcl::PointCloud<pcl::PointXYZ>);


pcl::PointCloud<pcl::PointXYZ>::Ptr     thumb_workspace_spheres_best                                         (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     index_workspace_spheres_best                                         (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     middle_workspace_spheres_best                                        (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     pinky_workspace_spheres_best                                         (new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr     right_finger_workspace_spheres_best                                  (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     left_finger_workspace_spheres_best                                   (new pcl::PointCloud<pcl::PointXYZ>);

// declarations for allegro right hand
pcl::PointCloud<pcl::PointXYZ>::Ptr     object_points_in_thumb_workspace                                     (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     object_points_in_index_workspace                                     (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     object_points_in_middle_workspace                                    (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     object_points_in_pinky_workspace                                     (new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr     object_points_in_thumb_workspace_best                                (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     object_points_in_index_workspace_best                                (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     object_points_in_middle_workspace_best                               (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     object_points_in_pinky_workspace_best                                (new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr     thumb_workspace_active_spheres_offset                                (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     thumb_workspace_active_spheres_parameter                             (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     thumb_workspace_active_spheres_offset_best                           (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     thumb_workspace_active_spheres_parameter_best                        (new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr     index_workspace_active_spheres_offset                                (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     index_workspace_active_spheres_parameter                             (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     index_workspace_active_spheres_offset_best                           (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     index_workspace_active_spheres_parameter_best                        (new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr     middle_workspace_active_spheres_offset                               (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     middle_workspace_active_spheres_parameter                            (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     middle_workspace_active_spheres_offset_best                          (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     middle_workspace_active_spheres_parameter_best                       (new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr     pinky_workspace_active_spheres_offset                                (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     pinky_workspace_active_spheres_parameter                             (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     pinky_workspace_active_spheres_offset_best                           (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     pinky_workspace_active_spheres_parameter_best                        (new pcl::PointCloud<pcl::PointXYZ>);

// declarations for panda gripper
pcl::PointCloud<pcl::PointXYZ>::Ptr     object_points_in_right_finger_workspace                              (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     object_points_in_left_finger_workspace                               (new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr     object_points_in_right_finger_workspace_best                         (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     object_points_in_left_finger_workspace_best                          (new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr     right_finger_workspace_active_spheres_offset                         (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     right_finger_workspace_active_spheres_parameter                      (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     right_finger_workspace_active_spheres_offset_best                    (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     right_finger_workspace_active_spheres_parameter_best                 (new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr     left_finger_workspace_active_spheres_offset                          (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     left_finger_workspace_active_spheres_parameter                       (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     left_finger_workspace_active_spheres_offset_best                     (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     left_finger_workspace_active_spheres_parameter_best                  (new pcl::PointCloud<pcl::PointXYZ>);

// scene captured clouds
pcl::PointCloud<pcl::PointXYZ>::Ptr     scene_cloud_xyz_1                                                    (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     scene_cloud_xyz_2                                                    (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     scene_cloud_xyz_3                                                    (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     scene_cloud_xyz_1_transformed                                        (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     scene_cloud_xyz_2_transformed                                        (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     scene_cloud_xyz_3_transformed                                        (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     scene_cloud_xyz_1_transformed_downsampled                            (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     scene_cloud_xyz_2_transformed_downsampled                            (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     scene_cloud_xyz_3_transformed_downsampled                            (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     scene_cloud_xyz_transformed_downsampled                              (new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr     object_cloud_xyz_downsampled                                         (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     table_cloud_xyz_downsampled                                          (new pcl::PointCloud<pcl::PointXYZ>);




pcl::PCDReader reader;

std::string mode;
std::string gripper_file_name;
std::string file_name1, file_name2, file_name3;
std::string transformation_matrix_file_name;
std::string object_file_name;
Eigen::MatrixXd tm;
Eigen::Matrix4f tm1, tm2, tm3;



// for point cloud visualization
boost::shared_ptr<pcl::visualization::PCLVisualizer> scene_cloud_viewer  (new pcl::visualization::PCLVisualizer ("scene cloud viewer"));

pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> magenta_color       (object_cloud_downsampled_in_arm_hand_frame_xyz, 255, 0, 255);

pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> brown_color         (object_plane_cloud_downsampled_in_arm_hand_frame_xyz, 165, 42, 42);
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> orange_color        (object_plane_special_ellipsoid_point_cloud_in_arm_hand_frame, 255, 165, 0);

pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color           (thumb_workspace_spheres_best, 255, 0, 0);
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green_color         (index_workspace_spheres_best, 0, 255, 0);
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue_color          (middle_workspace_spheres_best, 0, 0, 255);
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> grey_color          (pinky_workspace_spheres_best, 100, 100, 100);

pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color_again     (right_finger_workspace_spheres_best, 255, 0, 0);
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green_color_again   (left_finger_workspace_spheres_best, 0, 255, 0);
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue_color_again    (object_sampling_in_arm_hand_frame_xyz, 0, 0, 255);

pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> black_color         (gripper_as_set_of_special_ellipsoids_transformed_in_arm_hand_frame, 0, 0, 0);
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> black_color_again   (gripper_as_set_of_special_ellipsoids_in_arm_hand_frame, 0, 0, 0);
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cyan_color          (gripper_cloud_transformed_in_arm_hand_frame_xyz, 0, 255, 255);
//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cyan_color_2        (gripper_cloud_transformed_in_arm_hand_frame_xyz, 255, 0, 0);
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cyan_color_again    (gripper_cloud_downsampled_in_arm_hand_frame_xyz, 0, 255, 255);

pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> orange_color_again  (gripper_support_point_cloud_in_arm_hand_frame, 255, 165, 0);
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> orange_color_again2 (gripper_support_point_cloud_transformed_in_arm_hand_frame, 255, 165, 0);

pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> orange_color_again_1  (gripper_support_point_cloud_in_arm_hand_frame_1, 255, 165, 0);
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> orange_color_again2_1 (gripper_support_point_cloud_transformed_in_arm_hand_frame_1, 255, 165, 0);

pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> orange_color_again_2  (gripper_support_point_cloud_in_arm_hand_frame_2, 255, 165, 0);
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> orange_color_again2_2 (gripper_support_point_cloud_transformed_in_arm_hand_frame_2, 255, 165, 0);

pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> orange_color_again_3  (gripper_support_point_cloud_in_arm_hand_frame_3, 255, 165, 0);
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> orange_color_again2_3 (gripper_support_point_cloud_transformed_in_arm_hand_frame_3, 255, 165, 0);

pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> orange_color_again_4  (gripper_support_point_cloud_in_arm_hand_frame_4, 255, 165, 0);
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> orange_color_again2_4 (gripper_support_point_cloud_transformed_in_arm_hand_frame_4, 255, 165, 0);







// other declarations
Eigen::Vector3f dummy_translation;  Eigen::Matrix3f dummy_rotation;  Eigen::Matrix4f dummy_transform;
Eigen::Vector3f parameter_vector;  Eigen::Vector3f offset_vector;
pcl::PointXYZ point_xyz;
Eigen::Affine3f transform = Eigen::Affine3f::Identity();
int point_cloud_samples = 250;
double value_x, value_y, value_z;
double special_ellipsoid_value;
std::string save_file_name, save_file_name_old;



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Transformations
// camera optical frame wrt arm hand frame
Eigen::Vector3f camera_depth_optical_frame_wrt_arm_hand_frame_translation;
Eigen::Matrix4f camera_depth_optical_frame_wrt_arm_hand_frame_transform;

// gripper frame wrt arm hand frame
Eigen::Vector3f gripper_wrt_arm_hand_frame_translation;
Eigen::Matrix3f gripper_wrt_arm_hand_frame_rotation;
Eigen::Matrix4f gripper_wrt_arm_hand_frame_transform;
Eigen::Matrix4f gripper_wrt_arm_hand_frame_inverse_transform;

// camera optical frame wrt gripper frame
Eigen::Vector3f camera_depth_optical_frame_wrt_gripper_frame_translation;
Eigen::Matrix3f camera_depth_optical_frame_wrt_gripper_frame_rotation;
Eigen::Matrix4f camera_depth_optical_frame_wrt_gripper_frame_transform;




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Object pose approximation and sampling
// compute object transformation matrix
Eigen::Matrix3f object_rotation_wrt_arm_hand_frame;
Eigen::Vector3f object_translation_wrt_arm_hand_frame;
Eigen::Matrix4f object_transform_wrt_arm_hand_frame;
Eigen::Vector4f object_far_point_in_pos_direction_in_global_frame;
Eigen::Vector4f object_far_point_in_neg_direction_in_global_frame;
Eigen::Vector3f object_major_dimensions;

// sampling the object around its z-axis for scanning
int object_sampling_in_x_axis = 5;   int object_sampling_in_y_axis = 5;   int object_sampling_in_z_axis = 5;
int orientation_samples = 18;
int orientation_samples_in_x = 4, orientation_samples_in_y = 4, orientation_samples_in_z = 4;
double initial_orientation = 0.0;
double orientation_range = 2*M_PI;
//double initial_orientation = -M_PI/2;
//double orientation_range = M_PI/2;
int desired_number_of_object_cloud_points = 300;
double desired_distance_to_table = 0.07;

// object centroid location in arm hand frame
pcl::CentroidPoint<pcl::PointXYZ> object_centroid_in_arm_hand_frame;
pcl::PointXYZ object_centroid_point_in_arm_hand_frame;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Object plane pose approximation
// compute object plane transformation matrix
Eigen::Matrix3f object_plane_rotation_wrt_arm_hand_frame;
Eigen::Vector3f object_plane_translation_wrt_arm_hand_frame;
Eigen::Matrix4f object_plane_transform_wrt_arm_hand_frame;
Eigen::Vector4f object_plane_far_point_in_pos_direction_in_global_frame;
Eigen::Vector4f object_plane_far_point_in_neg_direction_in_global_frame;
Eigen::Vector3f object_plane_major_dimensions;
Eigen::Vector4f gripper_support_offset_in_arm_hand_frame;
Eigen::Vector4f gripper_support_offset_in_gripper_frame;
Eigen::Vector4f gripper_support_offset_in_gripper_transformed_frame;
Eigen::Vector4f gripper_support_in_arm_hand_frame;
Eigen::Vector4f gripper_support_in_gripper_frame;
Eigen::Vector4f gripper_support_in_gripper_transformed_frame;
Eigen::Vector4f gripper_support_in_object_plane_frame;

Eigen::Vector4f gripper_support_offset_in_arm_hand_frame_1;
Eigen::Vector4f gripper_support_offset_in_gripper_frame_1;
Eigen::Vector4f gripper_support_offset_in_gripper_transformed_frame_1;
Eigen::Vector4f gripper_support_in_arm_hand_frame_1;
Eigen::Vector4f gripper_support_in_gripper_frame_1;
Eigen::Vector4f gripper_support_in_gripper_transformed_frame_1;
Eigen::Vector4f gripper_support_in_object_plane_frame_1;

Eigen::Vector4f gripper_support_offset_in_arm_hand_frame_2;
Eigen::Vector4f gripper_support_offset_in_gripper_frame_2;
Eigen::Vector4f gripper_support_offset_in_gripper_transformed_frame_2;
Eigen::Vector4f gripper_support_in_arm_hand_frame_2;
Eigen::Vector4f gripper_support_in_gripper_frame_2;
Eigen::Vector4f gripper_support_in_gripper_transformed_frame_2;
Eigen::Vector4f gripper_support_in_object_plane_frame_2;

Eigen::Vector4f gripper_support_offset_in_arm_hand_frame_3;
Eigen::Vector4f gripper_support_offset_in_gripper_frame_3;
Eigen::Vector4f gripper_support_offset_in_gripper_transformed_frame_3;
Eigen::Vector4f gripper_support_in_arm_hand_frame_3;
Eigen::Vector4f gripper_support_in_gripper_frame_3;
Eigen::Vector4f gripper_support_in_gripper_transformed_frame_3;
Eigen::Vector4f gripper_support_in_object_plane_frame_3;

Eigen::Vector4f gripper_support_offset_in_arm_hand_frame_4;
Eigen::Vector4f gripper_support_offset_in_gripper_frame_4;
Eigen::Vector4f gripper_support_offset_in_gripper_transformed_frame_4;
Eigen::Vector4f gripper_support_in_arm_hand_frame_4;
Eigen::Vector4f gripper_support_in_gripper_frame_4;
Eigen::Vector4f gripper_support_in_gripper_transformed_frame_4;
Eigen::Vector4f gripper_support_in_object_plane_frame_4;


// object plane cloud in its own frame
Eigen::Matrix4f object_plane_transform_wrt_arm_hand_frame_inverse;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Defining special ellipsoids
// gripper support region special ellipsoid
double gripper_support_x; double gripper_support_y; double gripper_support_z;
double gripper_support_offset_x; double gripper_support_offset_y; double gripper_support_offset_z;

double gripper_support_x_1; double gripper_support_y_1; double gripper_support_z_1;
double gripper_support_offset_x_1; double gripper_support_offset_y_1; double gripper_support_offset_z_1;

double gripper_support_x_2; double gripper_support_y_2; double gripper_support_z_2;
double gripper_support_offset_x_2; double gripper_support_offset_y_2; double gripper_support_offset_z_2;

double gripper_support_x_3; double gripper_support_y_3; double gripper_support_z_3;
double gripper_support_offset_x_3; double gripper_support_offset_y_3; double gripper_support_offset_z_3;

double gripper_support_x_4; double gripper_support_y_4; double gripper_support_z_4;
double gripper_support_offset_x_4; double gripper_support_offset_y_4; double gripper_support_offset_z_4;

// object plane special ellipsoid
double object_plane_x; double object_plane_y; double object_plane_z;
double object_plane_offset_x; double object_plane_offset_y; double object_plane_offset_z;

// gripper approximation as a set of special ellipsoids
std::vector<double> gripper_x, gripper_y, gripper_z;
std::vector<double> gripper_offset_x, gripper_offset_y, gripper_offset_z;

pcl::PointCloud<pcl::PointXYZ>::Ptr  dummy_cloud_xyz                                       (new pcl::PointCloud<pcl::PointXYZ>);



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Fixing object plane transformation problem !
// check if object point cloud collides with object plane special ellipsoid
pcl::PointCloud<pcl::PointXYZ>::Ptr  object_cloud_temp    (new pcl::PointCloud<pcl::PointXYZ>);
int collision_counter = 0;




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// load gripper workspace spheres and compute the centroid in the gripper frame
// declarations for allegro hand
pcl::PointCloud<pcl::PointXYZ>::Ptr     thumb_workspace_convex_offset                   (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     index_workspace_convex_offset                   (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     middle_workspace_convex_offset                  (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     pinky_workspace_convex_offset                   (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     thumb_workspace_convex_parameter                (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     index_workspace_convex_parameter                (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     middle_workspace_convex_parameter               (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     pinky_workspace_convex_parameter                (new pcl::PointCloud<pcl::PointXYZ>);

// declarations for panda gripper
pcl::PointCloud<pcl::PointXYZ>::Ptr     right_finger_workspace_convex_offset            (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     left_finger_workspace_convex_offset             (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     right_finger_workspace_convex_parameter         (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     left_finger_workspace_convex_parameter          (new pcl::PointCloud<pcl::PointXYZ>);

// declarations for both
pcl::PointCloud<pcl::PointXYZ>::Ptr     gripper_augmented_workspace_xyz                    (new pcl::PointCloud<pcl::PointXYZ>);
Eigen::Vector3f gripper_workspace_centroid_point_in_gripper_frame;




// grasp candidates selection
double sphere_value;

bool gripper_collides_with_object_plane = false;
bool gripper_collides_with_object       = false;
bool object_touches_gripper_support     = false;

int counter;

int gripper_collide_with_table   = 0;
int gripper_collide_with_object  = 0;
int gripper_contacts_with_object = 0;

double distance_between_gripper_support_and_object_centroid;
double distance_between_gripper_support_and_object_centroid_best = 1000.0;
double distance_between_gripper_fingers;
double distance_between_gripper_fingers_best = 0;

Eigen::Vector3f workspace_centroid_wrt_gripper_frame_translation;
Eigen::Matrix4f workspace_centroid_wrt_gripper_frame_transform;
Eigen::Matrix4f workspace_centroid_wrt_gripper_frame_transform_inverse;

Eigen::Matrix4f object_frame_wrt_gripper_centroid_frame_transform;

Eigen::Vector3f gripper_centroid_translation_in_gripper_centroid_frame;
Eigen::Matrix3f gripper_centroid_rotation_in_gripper_centroid_frame;
Eigen::Matrix4f gripper_centroid_transform_in_gripper_centroid_frame;

Eigen::Matrix4f gripper_centroid_transform_before_orientation_loop;

Eigen::Vector3f gripper_translation;
Eigen::Matrix3f gripper_rotation;
Eigen::Matrix4f gripper_transform;
Eigen::Matrix4f inverse_gripper_transform;

Eigen::Vector4f object_centroid_point_transformed;

Eigen::Matrix4f best_gripper_transform;
Eigen::Matrix3f best_gripper_rotation;
Eigen::Vector3f best_gripper_translation;




#endif

