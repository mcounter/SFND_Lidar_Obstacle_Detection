#ifndef CLOUD_SEGMENTS_H_
#define CLOUD_SEGMENTS_H_

#include <cmath>
#include <vector>

#include <pcl/point_cloud.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "render/box.h"

template<typename PointT>
class CloudSegment
{
private:
    void transformPlaneCoeffs(const Eigen::Affine3d& transform, const int dimension)
    {
        Eigen::Vector4d planeNorm{planeCoeffs[0], planeCoeffs[1], planeCoeffs[2], 1.0};
        
        std::cout << "Plane normal:" << std::endl;
        std::cout << planeNorm << std::endl;

        planeNorm = transform * planeNorm;

        std::cout << "Plane normal transformed:" << std::endl;
        std::cout << planeNorm << std::endl;

        Eigen::Vector4d planePoint{
            dimension == 0 ? -planeCoeffs[3] / planeCoeffs[0] : 0.0,
            dimension == 1 ? -planeCoeffs[3] / planeCoeffs[1] : 0.0,
            dimension == 2 ? -planeCoeffs[3] / planeCoeffs[2] : 0.0,
            1.0};

        std::cout << "Plane point:" << std::endl;
        std::cout << planePoint << std::endl;        

        planePoint = transform * planePoint;

        std::cout << "Transformed plane point:" << std::endl;
        std::cout << planePoint << std::endl;

        planeCoeffs[0] = planeNorm[0];
        planeCoeffs[1] = planeNorm[1];
        planeCoeffs[2] = planeNorm[2];
        planeCoeffs[3] = -(planeNorm[0] * planePoint[0] + planeNorm[1] * planePoint[1] + planeNorm[2] * planePoint[2]);
    }

    void transformPointCloud(const Eigen::Affine3d& transform, typename pcl::PointCloud<PointT>::Ptr cloud)
    {
        if (cloud == nullptr)
        {
            return;
        }

        int n = cloud->size();
        if (n <= 0)
        {
            return;
        }

        Eigen::Matrix<double, 4, Eigen::Dynamic> points{4, n};

        for (int i = 0; i < n; ++i)
        {
            points(0, i) = cloud->points[i].x;
            points(1, i) = cloud->points[i].y;
            points(2, i) = cloud->points[i].z;
            points(3, i) = 1.0;
        }

        points = transform * points;
        
        for (int i = 0; i < n; ++i)
        {
            cloud->points[i].x = points(0, i);
            cloud->points[i].y = points(1, i);
            cloud->points[i].z = points(2, i);
        }
    }   
public:
    typename pcl::PointCloud<PointT>::Ptr plane;
    typename pcl::PointCloud<PointT>::Ptr obstacles;

    std::vector<double> planeCoeffs;

    std::vector<typename pcl::PointCloud<PointT>::Ptr> obstacleClusters;
    typename pcl::PointCloud<PointT>::Ptr noiseCluster;

    int planeDimension;

    void normalizeByDim(const int dimension)
    {
        if (dimension < 0 || dimension > 2 || planeCoeffs.size() < 4)
        {
            return;
        }

        if (planeCoeffs[dimension] == 0.0)
        {
            return;
        }

        double scale = std::sqrt(planeCoeffs[0] * planeCoeffs[0] + planeCoeffs[1] * planeCoeffs[1] + planeCoeffs[2] * planeCoeffs[2]);
        if (scale == 0.0)
        {
            return;
        }

        if (planeCoeffs[dimension] < 0)
        {
            for (auto& v: planeCoeffs)
            {
                v = -v;
            }
        }

        planeDimension = dimension;

        Eigen::Vector3d planeNorm{planeCoeffs[0], planeCoeffs[1], planeCoeffs[2]};
        planeNorm.normalize();

        std::cout << "Original plane normal:" << std::endl;
        std::cout << planeNorm << std::endl;

        Eigen::Vector3d dimAxis{
            dimension == 0 ? 1.0 : 0.0,
            dimension == 1 ? 1.0 : 0.0,
            dimension == 2 ? 1.0 : 0.0};

        Eigen::Vector3d rotVector = planeNorm.cross(dimAxis);
        rotVector.normalize();

        std::cout << "Rotation vector:" << std::endl;
        std::cout << rotVector << std::endl;
        
        double rotAngle = std::acos(planeNorm.dot(dimAxis));
        std::cout << "Rotation angle:" << std::endl;
        std::cout << rotAngle << std::endl;

        Eigen::Affine3d transform = Eigen::Affine3d::Identity();
        transform.rotate(Eigen::AngleAxisd(rotAngle, rotVector));

        std::cout << "Plane transformation matrix:" << std::endl;
        std::cout << transform.matrix() << std::endl;

        transformPlaneCoeffs(transform, dimension);
        transformPointCloud(transform, plane);
        transformPointCloud(transform, obstacles);

        for (auto& cluster: obstacleClusters)
        {
            transformPointCloud(transform, cluster);
        }

        transformPointCloud(transform, noiseCluster);
    }

    BoxQ clusterBox(typename pcl::PointCloud<PointT>::Ptr cluster)
    {
        int n = cluster->size();

        Eigen::Matrix<double, 4, Eigen::Dynamic> points{4, n};
        for (int i = 0; i < n; ++i)
        {
            points(0, i) = cluster->points[i].x;
            points(1, i) = cluster->points[i].y;
            points(2, i) = cluster->points[i].z;
            points(3, i) = 1.0;
        }

        Eigen::Vector3d centroid{0, 0, 0};
        Eigen::Vector3d pointMin{0, 0, 0};
        Eigen::Vector3d pointMax{0, 0, 0};

        for (int i = 0; i < n; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                double& pointDimVal = points(j, i);

                centroid[j] += pointDimVal;

                if (i == 0 || pointDimVal < pointMin[j])
                {
                    pointMin[j] = pointDimVal;
                }

                if (i == 0 || pointDimVal > pointMax[j])
                {
                    pointMax[j] = pointDimVal;
                }
            }
        }

        if (n > 0)
        {
            for (int j = 0; j < 3; ++j)
            {
                centroid[j] /= n;
            }
        }

        BoxQ box;
        box.bboxTransform = Eigen::Vector3f{float(centroid[0]), float(centroid[1]), float(centroid[2])};
        box.cube_length = pointMax[0] - pointMin[0];
        box.cube_width = pointMax[1] - pointMin[1];
        box.cube_height = pointMax[2] - pointMin[2];

        Eigen::Affine3d clusterTransform = Eigen::Affine3d::Identity();
        clusterTransform.translate(centroid);
        //clusterTransform.rotate(Eigen::AngleAxisd(-planeRotationAngle, planeRotationVector));
        
        Eigen::Matrix3f rotMatrixf = clusterTransform.matrix().cast<float>().block(0, 0, 3, 3);
        box.bboxQuaternion = Eigen::Quaternionf(rotMatrixf);

        return box;
    }
};

#endif /* CLOUD_SEGMENTS_H_ */