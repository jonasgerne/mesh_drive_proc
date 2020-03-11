//
// Created by jonasgerstner on 20.02.20.
//

#ifndef MESH_DRIVE_PROC_NORMALS_H
#define MESH_DRIVE_PROC_NORMALS_H

#include "mesh_drive_proc/mesh.h"

//static Eigen::Vector3f vp = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
//                             std::numeric_limits<float>::max()};
//
//
//inline void flipNormalTowardsViewpoint(const Eigen::Vector3f &point, Eigen::Vector3f viewpoint, Eigen::Vector3f &normal) {
//    // check if normal needs to be flipped
//    viewpoint -= point;
//    float cos_theta = viewpoint.dot(normal);
//    std::cout << "before flipping\n" << normal << std::endl;
//    if (cos_theta < 0) {
//        std::cout << "flip" << std::endl;
//        normal *= -1;
//    }
//}
//
///**
// * TODO: add delta_h_max calculation
// * @param mesh
// * @param vertices
// * @return surface angle of the face defined by the given vertices
// */
//inline float calcFaceAngle(
//        // const pcl::Vector3fMap& v1, const pcl::Vector3fMap& v2, const pcl::Vector3fMap& v3,
//        const pcl::PointCloud<MyLabeledPointType>& vertex_data_cloud, const std::array<VertexIndex, 3>& vertices) {
//    // calculate normal from vertices
//    Timer get_vertices("get_vertices");
//    auto v1 = vertex_data_cloud[vertices[0].get()].getVector3fMap();
//    auto v2 = vertex_data_cloud[vertices[1].get()].getVector3fMap();
//    auto v3 = vertex_data_cloud[vertices[2].get()].getVector3fMap();
//    get_vertices.Stop();
//    Timer calc_normal("calc_normal");
//    Eigen::Vector3f normal = (v1 - v2).cross(v1 - v3);
//    calc_normal.Stop();
//    Timer calc_center("center");
//    Eigen::Vector3f center = (v1 + v2 + v3) / 3;
//    calc_center.Stop();
//    Timer normalize("normalize");
//    normal.normalize();
//
//    normalize.Stop();
//
//    Timer flip_normal("flip_normal");
//    //flipNormalTowardsViewpoint(center, vp, normal);
//    flip_normal.Stop();
//    // calculate the surface angle
//    return normal.dot(Eigen::Vector3f::UnitZ());
//}

inline void calcNormalAndFaceAngle(Mesh& half_edge_mesh, unsigned int idx){
    VAFC circ_vaf = half_edge_mesh.getVertexAroundFaceCirculator(FaceIndex(idx));
    const auto& vertices = half_edge_mesh.getVertexDataCloud();
    auto& faces = half_edge_mesh.getFaceDataCloud();
    Eigen::Matrix3f mat;
    int j = 0;
    const VAFC circ_vaf_end = circ_vaf;
    do {
        mat.col(j) = vertices[circ_vaf.getTargetIndex().get()].getVector3fMap();
        ++j;
    } while (++circ_vaf != circ_vaf_end);

    Eigen::Vector3f normal = (mat.col(0) - mat.col(1)).cross(mat.col(0) - mat.col(2));
    normal.normalize();

    faces[FaceIndex(idx).get()].getNormalVector3fMap() = normal;

    faces[FaceIndex(idx).get()].surface_angle = normal.dot(Eigen::Vector3f::UnitZ());
}


#endif //MESH_DRIVE_PROC_NORMALS_H
