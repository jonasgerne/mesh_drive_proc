//
// Created by jonasgerstner on 20.02.20.
//
#ifndef MESH_DRIVE_PROC_MESH_H
#define MESH_DRIVE_PROC_MESH_H
#define PCL_NO_PRECOMPILE

#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/geometry/polygon_mesh.h>


struct EIGEN_ALIGN16 MyLabeledPointType {
    PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
    PCL_ADD_NORMAL4D;
    uint16_t label;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
};

POINT_CLOUD_REGISTER_POINT_STRUCT (MyLabeledPointType,
                                   (float, x, x)
                                           (float, y, y)
                                           (float, z, z)
                                           (float, normal_x, normal_x)
                                           (float, normal_y, normal_y)
                                           (float, normal_z, normal_z)
                                           (uint16_t, label, label)
)


struct EIGEN_ALIGN16 MyFaceAngleData {
//    union {
//        struct {
//            float surface_angle;
//        };
//        float data_c[4];
//    };
    PCL_ADD_NORMAL4D;
    float surface_angle;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// TODO: definition causes multiple definition error despite guards, fix later
//std::ostream &operator<<(std::ostream &os, const MyFaceAngleData &fa) {
//    return (os << fa.surface_angle);
//}

template<bool IsManifoldT>
struct MeshTraits {
    typedef MyLabeledPointType VertexData;
    typedef pcl::geometry::NoData HalfEdgeData;
    typedef pcl::geometry::NoData EdgeData;
    typedef MyFaceAngleData FaceData;
    typedef boost::integral_constant<bool, IsManifoldT> IsManifold;
};

// Declare the mesh.
using Mesh = pcl::geometry::PolygonMesh<MeshTraits<false>>;

using VertexIndex = Mesh::VertexIndex;
using HalfEdgeIndex = Mesh::HalfEdgeIndex;
using FaceIndex = Mesh::FaceIndex;

using VAVC = Mesh::VertexAroundVertexCirculator;
using OHEAVC = Mesh::OutgoingHalfEdgeAroundVertexCirculator;
using IHEAVC = Mesh::IncomingHalfEdgeAroundVertexCirculator;
using FAVC = Mesh::FaceAroundVertexCirculator;
using VAFC = Mesh::VertexAroundFaceCirculator;
using IHEAFC = Mesh::InnerHalfEdgeAroundFaceCirculator;
using OHEAFC = Mesh::OuterHalfEdgeAroundFaceCirculator;
#endif //MESH_DRIVE_PROC_MESH_H
