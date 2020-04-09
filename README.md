# mesh_drive_proc


## Subscribed topics
input mesh ``pcl_msgs::PolygonMesh``  
    Mesh topic with surface mesh from reconstruction pipeline

## Published topics
traversability point cloud ``pcl::PointCloud<pcl::PointXYZRGBNormal>``  
    A point cloud of color coded surface points, green if traversable, red if not. Method determined with ``local_neighborhood`` parameter.

## Parameters
``max_surface_angle_degree``  
    Maximum angle between surface normal and (0, 0, 1)  
``local_neighborhood``  
    If true, traversable points are mesh vertices that are marked depending on their adjacent faces.
``path_ply``
    Path to export a mesh as .ply file
