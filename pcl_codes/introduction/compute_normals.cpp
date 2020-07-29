void downsample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points, float leaf_size, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &downsampled_out){
  pcl::VoxelGrid<pcl::PointXYZRGB> vox_grid;
  vox_grid.setLeafSize(leaf_size,leaf_size,leaf_size);
  vox_grid.setInputCloud(points);
  vox_grid.filter(*downsampledout);
}

void compute_surface_normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points, float normal_radius, pcl::PointCloud<pcl::Normal>::Ptr &normals_out){
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
  
  //Use a FLANN−based KdTree to perform neighborhood searches
  norm_est.setSearchMethod(pcl::search::KdTree<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
  //Specify the local neighborhood size for computing the surface normals
  norm_est.setRadiusSearch(normal_radius);
  //Set the input points
  norm_est.setInputCloud(points);
  //Estimate the surface normals and store the result in "normals_out"
  norm_est.compute(*normals_out);
}
void visualize_normals(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr points, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr normal_points, const pcl::PointCloud<pcl::Normal>::Ptr normals){
  pcl::visualization::PCLVisualizer viz;
  viz.addPointCloud(points,"points");
  viz.addPointCloud(normal_points,"normal_points");
  viz.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(normal_points,normals,1,0.01,"normals");
  viz.spin();
}

int main(int argc, char** argv){
  //Load data from pcd...
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  
  //Downsample the cloud
  const float voxel_grid_leaf_size=0.01;
  downsample(cloud,voxel_grid_leaf_size,ds);
  
  //Compute surface normals
  const float normal_radius=0.03;
  compute_surface_normals(ds, normal_radius, normals);
  
  //Visualize the normals
  visualize_normals(cloud, ds, normals);
  
  return(0);
}
