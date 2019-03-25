#pragma once

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_representation.h>
#include <pcl/point_cloud.h>
#include <pcl/impl/pcl_base.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <pcl/search/impl/search.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/impl/organized.hpp>

#include <pcl/features/impl/pfh.hpp>
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/features/impl/normal_3d_omp.hpp>

#include <pcl/filters/impl/extract_indices.hpp>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/impl/pcl_visualizer.hpp>
#include <pcl/visualization/impl/point_cloud_geometry_handlers.hpp>
#include <pcl/visualization/impl/point_cloud_color_handlers.hpp>

namespace pcl
{
    struct _PointNormalK
    {
        PCL_ADD_POINT4D
        PCL_ADD_NORMAL4D
        PCL_ADD_RGB
        float curvature;
        int k;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;


    struct PointNormalK : public _PointNormalK
    {
        inline PointNormalK(const _PointNormalK &p)
        {
            x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
            normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z; data_n[3] = 0.0f;
            rgba = p.rgba;
            curvature = p.curvature;
            k = p.k;
        }

        inline PointNormalK()
        {
            x = y = z = 0.0f;
            data[3] = 1.0f;
            normal_x = normal_y = normal_z = data_n[3] = 0.0f;
            r = g = b = 255;
            a = 255;
            curvature = 0;
            k = 0;
        }
    };

    /*
    class PNKPointRepresentation : public PointRepresentation <PointNormalK>
    {
        using PointRepresentation<PointNormalK>::nr_dimensions_;

    public:
        PNKPointRepresentation()
        {
            nr_dimensions_ = 3;
        }

        // Override the copyToFloatArray method to define our feature vector
        virtual void copyToFloatArray (const PointNormalK &p, float * out) const
        {
            // < x, y, z >
            out[0] = p.x;
            out[1] = p.y;
            out[2] = p.z;
        }
    };
    */
}

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::_PointNormalK,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, normal_x, normal_x)
                                   (float, normal_y, normal_y)
                                   (float, normal_z, normal_z)
                                   (uint32_t, rgb, rgb)
                                   (uint8_t, r, r)
                                   (uint8_t, g, g)
                                   (uint8_t, b, b)
                                   (float, curvature, curvature)
                                   (int, k, k)
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::PointNormalK, pcl::_PointNormalK)

typedef pcl::PointNormalK PointNormalK;
typedef pcl::PointCloud<PointNormalK> PointNormalKCloud;
typedef pcl::KdTreeFLANN<PointNormalK, flann::L2_Simple<float>> KdTreeFlannK;
