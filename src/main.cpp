//
// Created by jonasgerstner on 19.02.20.
//
#include <ros/ros.h>
#include <cmath>
#include <std_srvs/Empty.h>

// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/geometry/mesh_conversion.h>
#include <pcl/common/common.h>

#include <mesh_drive_proc/normals.h>
#include <mesh_drive_proc/mesh.h>
#include <pcl/io/ply_io.h>

ros::Publisher pub;

class MeshAnalyzer{
public:
    MeshAnalyzer(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    void mesh_cb(const pcl_msgs::PolygonMeshPtr &input_mesh);
    void mesh_cb_neighborhood(const pcl_msgs::PolygonMeshPtr &input_mesh);
    bool savePLY(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber mesh_sub_;
    ros::Publisher pcl_pub_;

    ros::ServiceServer save_ply_srv_;
    float cos_max_surface_angle_;
    float max_surf_angle_deg_;

    Mesh half_edge_mesh_;

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr last_pcl_traversable_;

    std::string path_ply_;

    bool check_local_neighborhood_;
};

MeshAnalyzer::MeshAnalyzer(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) :
    nh_(nh), nh_private_(nh_private) {

    nh_private_.param("local_neighborhood", check_local_neighborhood_, false);

    if (check_local_neighborhood_)
        mesh_sub_ = nh_.subscribe("input", 1, &MeshAnalyzer::mesh_cb_neighborhood, this);
    else
        mesh_sub_ = nh_.subscribe("input", 1, &MeshAnalyzer::mesh_cb, this);

    ROS_INFO("Subscribed to %s", mesh_sub_.getTopic().c_str());

    // Create a ROS publisher for the output point cloud
    pcl_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGBNormal>>("output", 1, true);

    // services
    save_ply_srv_ = nh_private_.advertiseService("save_ply", &MeshAnalyzer::savePLY, this);

    nh_private_.param("max_surface_angle", max_surf_angle_deg_, 30.0f);
    nh_private_.param<std::string>("path_ply", path_ply_, "");

    cos_max_surface_angle_ = cos(max_surf_angle_deg_ * EIGEN_PI / 180.0);
}

void MeshAnalyzer::mesh_cb(const pcl_msgs::PolygonMeshPtr &input_mesh){
    ROS_INFO("Received mesh_pcl message.");

    auto *mesh = new pcl::PolygonMesh;
    pcl::PolygonMeshConstPtr meshPtr(mesh);
    pcl_conversions::toPCL(*input_mesh, *mesh);

    pcl::PointCloud<pcl::PointXYZ>::Ptr vertices( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::fromPCLPointCloud2(meshPtr->cloud, *vertices);

    last_pcl_traversable_.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    const auto& faces = meshPtr->polygons;

    for (const auto& face : faces) {

        Eigen::Matrix3f mat;

        if (face.vertices.size() > 3)
            continue;

        for (int i = 0; i < 3; ++i){
            mat.col(i) = vertices->operator[](face.vertices[i]).getVector3fMap();
        }

        Eigen::Vector3f normal = (mat.col(0) - mat.col(1)).cross(mat.col(0) - mat.col(2));
        normal.normalize();

        Eigen::Vector3f center = mat.rowwise().sum() / 3.0;

        pcl::PointXYZRGBNormal p_temp;
        p_temp.x = center.x();
        p_temp.y = center.y();
        p_temp.z = center.z();
        p_temp.normal_x = normal.x();
        p_temp.normal_y = normal.y();
        p_temp.normal_z = normal.z();

        float surface_angle = normal.dot(Eigen::Vector3f::UnitZ());

        if (surface_angle < cos_max_surface_angle_)
            p_temp.r = 255;
        else
            p_temp.g = 255;

        last_pcl_traversable_->push_back(p_temp);

    }

    last_pcl_traversable_->header.frame_id = "world";
    pcl_pub_.publish(last_pcl_traversable_);
    ROS_INFO("Finished processing pointcloud.");

    /*
    int not_added = pcl::geometry::toHalfEdgeMesh(*meshPtr, half_edge_mesh_);
    ROS_INFO("%d faces could not be added.", not_added);
    ROS_INFO("Size Edges: %ld, size faces: %ld", half_edge_mesh_.sizeEdges(), half_edge_mesh_.sizeFaces());

    half_edge_mesh_.cleanUp();

    last_pcl_traversable_.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    const auto& faces = half_edge_mesh_.getFaceDataCloud();
    const auto& vertices = half_edge_mesh_.getVertexDataCloud();

    for (unsigned int i = 0; i < half_edge_mesh_.sizeFaces(); ++i) {
        VAFC circ_vaf = half_edge_mesh_.getVertexAroundFaceCirculator(FaceIndex(i));

        Eigen::Matrix3f mat;
        int j = 0;
        const VAFC circ_vaf_end = circ_vaf;
        do {
            mat.col(j) = vertices[circ_vaf.getTargetIndex().get()].getVector3fMap();
            ++j;
        } while (++circ_vaf != circ_vaf_end);

        Eigen::Vector3f normal = (mat.col(0) - mat.col(1)).cross(mat.col(0) - mat.col(2));
        normal.normalize();

        Eigen::Vector3f center = mat.rowwise().sum() / 3.0;

        pcl::PointXYZRGBNormal p_temp;
        p_temp.x = center.x();
        p_temp.y = center.y();
        p_temp.z = center.z();
        p_temp.normal_x = normal.x();
        p_temp.normal_y = normal.y();
        p_temp.normal_z = normal.z();

        float surface_angle = normal.dot(Eigen::Vector3f::UnitZ());

        if (surface_angle < cos_max_surface_angle_)
            p_temp.r = 255;
        else
            p_temp.g = 255;

        last_pcl_traversable_->push_back(p_temp);
    }

    last_pcl_traversable_->header.frame_id = "world";
    pcl_pub_.publish(last_pcl_traversable_);
    ROS_INFO("Finished processing pointcloud.");
    */
}

void MeshAnalyzer::mesh_cb_neighborhood(const pcl_msgs::PolygonMeshPtr &input_mesh){
    ROS_INFO("Received mesh_pcl message.");

    half_edge_mesh_.clear();

    bool has_normals = false;
    for (auto &field : input_mesh->cloud.fields) {
        if (field.name == "normal_x")
            has_normals = true;
    }
    ROS_INFO_COND(!has_normals, "Incoming point cloud message has no vertex normals.");
    auto *mesh = new pcl::PolygonMesh;
    pcl::PolygonMeshConstPtr meshPtr(mesh);
    pcl_conversions::toPCL(*input_mesh, *mesh);


    int not_added = pcl::geometry::toHalfEdgeMesh(*meshPtr, half_edge_mesh_);
    ROS_INFO("%d faces could not be added.", not_added);
    ROS_INFO("Size Edges: %ld, size faces: %ld", half_edge_mesh_.sizeEdges(), half_edge_mesh_.sizeFaces());

    half_edge_mesh_.cleanUp();

    // 1. Iterate over all faces and calculate face normal
    ROS_INFO("Iterating over faces.");
    const pcl::PointCloud<MyLabeledPointType> &vertex_cloud = half_edge_mesh_.getVertexDataCloud();

    for (unsigned int i = 0; i < half_edge_mesh_.sizeFaces(); ++i) {
        calcNormalAndFaceAngle(half_edge_mesh_, i);
    }
    ROS_INFO("Surface normal calculation finished.");
    // Removes all mesh elements and data that are marked as deleted.
    half_edge_mesh_.cleanUp();
    // 2. iterate over all vertices (i<mesh.sizeVertices() and mesh.getVertexDataCloud()[i])
    bool traversable;
    for (unsigned int i = 0; i < half_edge_mesh_.sizeVertices(); ++i) {
        traversable = true;
        // only used if incoming point cloud didn't have normals
        Eigen::Vector3f normal;
        // 2.2 iterate over all neighbouring faces: mesh.getFaceAroundVertexCalculator()
        FAVC circ_fav = half_edge_mesh_.getFaceAroundVertexCirculator(VertexIndex(i));
        const FAVC circ_fav_end = circ_fav;
        do {
            if (!half_edge_mesh_.isBoundary(circ_fav.getCurrentHalfEdgeIndex())) {
                const auto &face = half_edge_mesh_.getFaceDataCloud()[circ_fav.getTargetIndex().get()];
                if (face.surface_angle < cos_max_surface_angle_) {
                    traversable = false;
                    break;
                }
                if (!(has_normals))
                    normal += face.getNormalVector3fMap();
            } else {
                traversable = false;
                break;
            }
        } while (++circ_fav != circ_fav_end);
        // 2.3 if all drivable, set drivable, if one doesn't exist (border) undrivable
        auto &vertex = half_edge_mesh_.getVertexDataCloud()[i];
        vertex.label = static_cast<std::uint16_t>(traversable);
        if (!(has_normals))
            vertex.getNormalVector3fMap() = normal.normalized();
    }

    last_pcl_traversable_.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    for (unsigned int j = 0; j < half_edge_mesh_.sizeVertices(); ++j) {
        const MyLabeledPointType &pt_in = half_edge_mesh_.getVertexDataCloud()[j];
        pcl::PointXYZRGBNormal p_temp;
        p_temp.x = pt_in.x;
        p_temp.y = pt_in.y;
        p_temp.z = pt_in.z;
        p_temp.normal_x = pt_in.normal_x;
        p_temp.normal_y = pt_in.normal_y;
        p_temp.normal_z = pt_in.normal_z;
        if (pt_in.label == 0)
            p_temp.r = 255;
        else
            p_temp.g = 255;
        last_pcl_traversable_->push_back(p_temp);
    }

    last_pcl_traversable_->header.frame_id = "world";
    pcl_pub_.publish(last_pcl_traversable_);
    ROS_INFO("Finished processing pointcloud.");
}

bool MeshAnalyzer::savePLY(std_srvs::Empty::Request& /*request*/,
                           std_srvs::Empty::Response& /*response*/){
    pcl::PolygonMesh output;
    pcl::geometry::toFaceVertexMesh(half_edge_mesh_, output);
    return pcl::io::savePLYFile(path_ply_, output);
}

int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "ovpc_mesh_analysis");

    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    MeshAnalyzer ma(nh, nh_private);

    // Spin
    ros::spin();
}
