#ifndef CLOUD_SEGMENTS_H_
#define CLOUD_SEGMENTS_H_

#include <cmath>
#include <vector>
#include <algorithm>

#include <pcl/point_cloud.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "trackingObject.h"

struct SortSegmentPoint
{
    int sortDim;

    SortSegmentPoint(int sortDimIdx) : sortDim{sortDimIdx} {};

    bool operator()(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2) const
    {
        return vec1[sortDim] < vec2[sortDim];
    }
};

template<typename PointT>
class CloudSegment
{
private:
    const bool debug = false;

    int planeDimension = 2;
    int nestedSplitDim = 0;
    double minNestedSegmentSize = 1;
    int minNestedSegmentPoints = 10;
    double maxNestedFactor = 0.8;
    
public:
    typename pcl::PointCloud<PointT>::Ptr plane;
    typename pcl::PointCloud<PointT>::Ptr obstacles;

    std::vector<double> planeCoeffs;

    std::vector<typename pcl::PointCloud<PointT>::Ptr> obstacleClusters;
    typename pcl::PointCloud<PointT>::Ptr noiseCluster;

    Eigen::Affine3d planeTransform;

    void calcPlaneTransform()
    {
        if (planeCoeffs[planeDimension] == 0.0)
        {
            return;
        }

        double scale = std::sqrt(planeCoeffs[0] * planeCoeffs[0] + planeCoeffs[1] * planeCoeffs[1] + planeCoeffs[2] * planeCoeffs[2]);
        if (scale == 0.0)
        {
            return;
        }

        if (planeCoeffs[planeDimension] < 0)
        {
            for (auto& v: planeCoeffs)
            {
                v = -v;
            }
        }

        Eigen::Vector3d planeNorm{planeCoeffs[0], planeCoeffs[1], planeCoeffs[2]};
        planeNorm.normalize();

        if (debug)
        {
            std::cout << "Original plane normal:" << std::endl;
            std::cout << planeNorm << std::endl;
        }

        Eigen::Vector3d dimAxis{
            planeDimension == 0 ? 1.0 : 0.0,
            planeDimension == 1 ? 1.0 : 0.0,
            planeDimension == 2 ? 1.0 : 0.0};

        Eigen::Vector3d rotVector = planeNorm.cross(dimAxis);
        rotVector.normalize();

        if (debug)
        {
            std::cout << "Rotation vector:" << std::endl;
            std::cout << rotVector << std::endl;
        }
        
        double rotAngle = std::acos(planeNorm.dot(dimAxis));

        if (debug)
        {
            std::cout << "Rotation angle:" << std::endl;
            std::cout << rotAngle << std::endl;
        }

        planeTransform = Eigen::Affine3d::Identity();
        planeTransform.rotate(Eigen::AngleAxisd(rotAngle, rotVector));

        if (debug)
        {
            std::cout << "Plane transformation matrix:" << std::endl;
            std::cout << planeTransform.matrix() << std::endl;
        }
    }

    bool buildTrackingObjectCollisionNested(
        TrackingObject& trackingObject,
        const std::vector<Eigen::Vector3d>& points,
        const int pointFrom,
        const int pointTo,
        const CollisionBox& parentShape)
    {
        if ((pointTo - pointFrom) < minNestedSegmentPoints ||
            ((parentShape.maxVert[nestedSplitDim] - parentShape.minVert[nestedSplitDim]) / 2.0) < minNestedSegmentSize)
        {
            return false;
        }

        CollisionBox leftShape{Eigen::Vector3d{0, 0, 0}, Eigen::Vector3d{0, 0, 0}};
        CollisionBox rightShape{Eigen::Vector3d{0, 0, 0}, Eigen::Vector3d{0, 0, 0}};

        double nestedSegmentMid = (parentShape.maxVert[nestedSplitDim] + parentShape.minVert[nestedSplitDim]) / 2.0;
        
        leftShape.minVert[nestedSplitDim] = parentShape.minVert[nestedSplitDim];
        leftShape.maxVert[nestedSplitDim] = nestedSegmentMid;
        rightShape.minVert[nestedSplitDim] = nestedSegmentMid;
        rightShape.maxVert[nestedSplitDim] = parentShape.maxVert[nestedSplitDim];

        leftShape.minVert[planeDimension] = parentShape.minVert[planeDimension];
        leftShape.maxVert[planeDimension] = parentShape.maxVert[planeDimension];
        rightShape.minVert[planeDimension] = parentShape.minVert[planeDimension];
        rightShape.maxVert[planeDimension] = parentShape.maxVert[planeDimension];

        int rightFrom = pointFrom;
        for (int i = pointFrom; i < pointTo; ++i)
        {
            bool isLeft = points[i][nestedSplitDim] <= nestedSegmentMid;
            if (isLeft)
            {
                rightFrom = i + 1;
            }

            bool isFirst = isLeft ? i == pointFrom : i == rightFrom;
            for (int j = 0; j < 3; ++j)
            {
                if (j != nestedSplitDim && j != planeDimension)
                {
                    const double& pointDimVal = points[i][j];
                    double& minVert = isLeft ? leftShape.minVert[j] : rightShape.minVert[j];
                    double& maxVert = isLeft ? leftShape.maxVert[j] : rightShape.maxVert[j];

                    if (isFirst || pointDimVal < minVert)
                    {
                        minVert = pointDimVal;
                    }

                    if (isFirst || pointDimVal > maxVert)
                    {
                        maxVert = pointDimVal;
                    }
                }
            }
        }

        if ((rightFrom - pointFrom) < minNestedSegmentPoints ||
            (pointTo - rightFrom) < minNestedSegmentPoints)
        {
            return false;
        }

        double parenSize =
            (parentShape.maxVert[0] - parentShape.minVert[0]) *
            (parentShape.maxVert[1] - parentShape.minVert[1]) *
            (parentShape.maxVert[2] - parentShape.minVert[2]);

        double leftSize =
            (leftShape.maxVert[0] - leftShape.minVert[0]) *
            (leftShape.maxVert[1] - leftShape.minVert[1]) *
            (leftShape.maxVert[2] - leftShape.minVert[2]);

        double rightSize =
            (rightShape.maxVert[0] - rightShape.minVert[0]) *
            (rightShape.maxVert[1] - rightShape.minVert[1]) *
            (rightShape.maxVert[2] - rightShape.minVert[2]);

        if ((leftSize + rightSize) > maxNestedFactor * parenSize)
        {
            return false;
        }
        
        if (!buildTrackingObjectCollisionNested(trackingObject, points, pointFrom, rightFrom, leftShape))
        {
            trackingObject.collisionNested.push_back(leftShape);
        }

        if (!buildTrackingObjectCollisionNested(trackingObject, points, rightFrom, pointTo, rightShape))
        {
            trackingObject.collisionNested.push_back(rightShape);
        }

        return true;
    }

    void buildTrackingObjectCollisionShape(TrackingObject& trackingObject, const std::vector<Eigen::Vector3d>& points, const bool& usePCA)
    {
        int n = points.size();

        trackingObject.collisionShape.minVert = Eigen::Vector3d{0, 0, 0};
        trackingObject.collisionShape.maxVert = Eigen::Vector3d{0, 0, 0};

        for (int i = 0; i < n; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                const double& pointDimVal = points[i][j];

                if (i == 0 || pointDimVal < trackingObject.collisionShape.minVert[j])
                {
                    trackingObject.collisionShape.minVert[j] = pointDimVal;
                }

                if (i == 0 || pointDimVal > trackingObject.collisionShape.maxVert[j])
                {
                    trackingObject.collisionShape.maxVert[j] = pointDimVal;
                }
            }
        }

        trackingObject.trackingCenter = Eigen::Vector3d{
            (trackingObject.collisionShape.maxVert[0] + trackingObject.collisionShape.minVert[0]) / 2.0,
            (trackingObject.collisionShape.maxVert[1] + trackingObject.collisionShape.minVert[1]) / 2.0,
            (trackingObject.collisionShape.maxVert[2] + trackingObject.collisionShape.minVert[2]) / 2.0
        };

        if (!usePCA)
        {
            buildTrackingObjectCollisionNested(trackingObject, points, 0, n, trackingObject.collisionShape);
        }
    }

    void buildPointsSortedVector(const Eigen::Matrix<double, 4, Eigen::Dynamic>& matr, std::vector<Eigen::Vector3d>& vec)
    {
        int n = matr.cols();
        for (int i = 0; i < n; ++i)
        {
            vec.push_back(Eigen::Vector3d{matr(0, i), matr(1, i), matr(2, i)});
        }

        std::sort(vec.begin(), vec.end(), SortSegmentPoint(nestedSplitDim));
    }

    TrackingObject trackingObject(typename pcl::PointCloud<PointT>::Ptr cluster, const int idx, const bool& usePCA)
    {
        TrackingObject tracking;
        tracking.clusterIdx = idx;

        int n = cluster->size();
        if (n <= 0)
        {
            return tracking;
        }

        Eigen::Matrix<double, 4, Eigen::Dynamic> points{4, n};
        for (int i = 0; i < n; ++i)
        {
            points(0, i) = cluster->points[i].x;
            points(1, i) = cluster->points[i].y;
            points(2, i) = cluster->points[i].z;
            points(3, i) = 1.0;
        }

        points = planeTransform * points;

        double rotAngleXY = 0;

        if (usePCA)
        {
            Eigen::Matrix<double, 2, Eigen::Dynamic> points2D{2, n};
            Eigen::Vector2d centroid{0, 0};

            for (int i = 0; i < n; ++i)
            {
                if (planeDimension == 2)
                {
                    points2D(0, i) = points(0, i);
                    points2D(1, i) = points(1, i);
                }
                else if (planeDimension == 1)
                {
                    points2D(0, i) = points(0, i);
                    points2D(1, i) = points(2, i);
                }
                else
                {
                    points2D(0, i) = points(1, i);
                    points2D(1, i) = points(2, i);
                }

                centroid[0] += points2D(0, i);
                centroid[1] += points2D(1, i);
            }

            centroid /= n;

            if (debug)
            {
                std::cout << "Centroid:" << std::endl;
                std::cout << centroid << std::endl;
            }

            points2D.colwise() -= centroid;
            Eigen::Matrix2d corr = points2D * points2D.transpose();
            corr /= n;

            if (debug)
            {
                std::cout << "Correlation:" << std::endl;
                std::cout << corr << std::endl;
            }

            double det = (corr(0, 0) - corr(1, 1)) * (corr(0, 0) - corr(1, 1)) + 4 * corr(0, 1) * corr(0, 1);
            if (det >= 0)
            {
                det = std::sqrt(det);
                double lambdaL = (corr(0, 0) + corr(1, 1) - det) * 0.5;
                Eigen::Vector2d mainVector{
                    corr(0, 0) + corr(0, 1) - lambdaL,
                    corr(1, 1) + corr(0, 1) - lambdaL
                };

                mainVector.normalize();

                rotAngleXY = std::acos(mainVector[0]);

                if (debug)
                {
                    std::cout << "XY rotation angle:" << std::endl;
                    std::cout << rotAngleXY << std::endl;
                }
            }
        }

        Eigen::Vector3d rotVectorXY{
            planeDimension == 0 ? 1.0 : 0.0,
            planeDimension == 1 ? 1.0 : 0.0,
            planeDimension == 2 ? 1.0 : 0.0};

        Eigen::Affine3d transformXY = Eigen::Affine3d::Identity();
        transformXY.rotate(Eigen::AngleAxisd(-rotAngleXY, rotVectorXY));

        if (debug)
        {
            std::cout << "XY rotation matrix:" << std::endl;
            std::cout << transformXY.matrix() << std::endl;
        }

        points = transformXY * points;
        
        std::vector<Eigen::Vector3d> pointsVector;
        buildPointsSortedVector(points, pointsVector);
        buildTrackingObjectCollisionShape(tracking, pointsVector, usePCA);

        tracking.renderTransform = (transformXY * planeTransform).inverse();

        return tracking;
    }

    void mergeTrackingObjects(const std::vector<TrackingObject>& origState, std::vector<TrackingObject>& newState, const double& distTolerance)
    {
        newState.clear();

        int n = origState.size();
        std::vector<bool> touched(n, false);
        for (int i = 0; i < n; ++i)
        {
            if (!touched[i])
            {
                touched[i] = true;
                newState.push_back(origState[i]);
                TrackingObject& obj = newState[newState.size() - 1];

                std::vector<int> queue;
                queue.push_back(i);
                int queueIdx = 0;

                while (queueIdx < queue.size())
                {
                    int idx = queue[queueIdx];
                    if (!touched[idx])
                    {
                        obj.mergeWithObject(origState[idx]);
                    }

                    if (queueIdx == 0 || !touched[idx])
                    {
                        touched[idx] = true;

                        for (int j = 0; j < n; ++j)
                        {
                            if (!touched[j] &&
                                origState[idx].isTrackingObjectClose(origState[j], distTolerance))
                            {
                                queue.push_back(j);
                            }
                        }
                    }

                    ++queueIdx;
                }
            }
        }
    }
};

#endif /* CLOUD_SEGMENTS_H_ */