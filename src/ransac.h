#ifndef RANSAC_H_
#define RANSAC_H_

#include <cmath>
#include <chrono>
#include <algorithm>
#include <string>
#include <unordered_set>
#include <chrono>

#include <pcl/point_cloud.h>

#include "tools.h"

template <typename PointT>
class RansacPlane3D
{
private:
    double distanceTolerance = 0.3;
    double successProb = 0.99999;
    double inliersProb = 0.3;
    double optFactor = 0.9;

    int calcMaxIterations(const int& subsetSize, const double& successProb, const double& inliersProb)
    {
        double w = std::pow(inliersProb, subsetSize);

        return std::log(1 - successProb) / std::log(1 - w) + std::sqrt(1 - w) / w + 0.5;
    }

public:
    RansacPlane3D() {}
    ~RansacPlane3D() {}

    double getDistanceTolerance()
    {
        return distanceTolerance;
    }

    void setDistanceTolerance(double distanceTolerance)
    {
        this-> distanceTolerance = distanceTolerance;
    }

    double getSuccessProb()
    {
        return successProb;
    }

    void setSuccessProb(double successProb)
    {
        this-> successProb = successProb;
    }

    double getInliersProb()
    {
        return inliersProb;
    }

    void setInliersProb(double inliersProb)
    {
        this-> inliersProb = inliersProb;
    }

    double getOptFactor()
    {
        return optFactor;
    }

    void setOptFactor(double optFactor)
    {
        this-> optFactor = optFactor;
    }

    bool setParm(const std::string& parm, const std::string& uParm, const std::string& val, const std::string& uVal)
    {
        if (uParm == strUpr("RansacDistanceTolerance"))
        {
            distanceTolerance = std::stod(uVal);
        }
        else if (uParm == strUpr("RansacSuccessProb"))
        {
            successProb = std::stod(uVal);
        }
        else if (uParm == strUpr("RansacInliersProb"))
        {
            inliersProb = std::stod(uVal);
        }
        else if (uParm == strUpr("RansacOptFactor"))
        {
            optFactor = std::stod(uVal);
        }
        else
        {
            return false;
        }

        return true;
    }

    void segment(const typename pcl::PointCloud<PointT>::Ptr& cloud, std::unordered_set<int>& inliers, std::vector<double>& coeffs)
    {
        int maxIterations = calcMaxIterations(3, successProb, inliersProb);
        std::cout << "Ransac: Maximum iterations calculated " << maxIterations << std::endl;

        int cloudSize = cloud->size();
        if (cloudSize >= 3)
        {
            int inliersSize = inliers.size();
            
            for (int i = 0; i < maxIterations; ++i)
            {
                std::unordered_set<int> inliersIteration;
                std::vector<int> pidx;
                while (inliersIteration.size() < 3)
                {
                    int idx = std::rand() % cloudSize;
                    if (inliersIteration.find(idx) == inliersIteration.end())
                    {
                        inliersIteration.insert(idx);
                        pidx.push_back(idx);
                    }
                }

                PointT& point1 = cloud->points[pidx[0]];
                PointT& point2 = cloud->points[pidx[1]];
                PointT& point3 = cloud->points[pidx[2]];

                double a = (point2.y - point1.y) * (point3.z - point1.z) - (point2.z - point1.z) * (point3.y - point1.y);
                double b = (point2.z - point1.z) * (point3.x - point1.x) - (point2.x - point1.x) * (point3.z - point1.z);
                double c = (point2.x - point1.x) * (point3.y - point1.y) - (point2.y - point1.y) * (point3.x - point1.x);
                double d = -(a * point1.x + b * point1.y + c * point1.z);
                
                double distanceTolNorm = distanceTolerance * std::sqrt(a*a + b*b + c*c);

                if (distanceTolNorm > 0)
                {
                    for (int idx = 0; idx < cloudSize; ++idx)
                    {
                        PointT& pointX = cloud->points[idx];
                        double dist = std::fabs(a * pointX.x + b * pointX.y + c * pointX.z + d);
                        if (dist < distanceTolNorm)
                        {
                            inliersIteration.insert(idx);
                        }
                    }
                }

                int inliersIterSize = inliersIteration.size();
                if (inliersIterSize > inliersSize)
                {
                    inliers = inliersIteration;
                    inliersSize = inliersIterSize;
                    coeffs.resize(4);
                    coeffs[0] = a;
                    coeffs[1] = b;
                    coeffs[2] = c;
                    coeffs[3] = d;

                    if (optFactor > 0)
                    {
                        maxIterations = std::min(maxIterations, i + 1 + calcMaxIterations(3, successProb, optFactor * (double)inliersSize / (double)cloudSize));
                        std::cout << "Ransac: Maximum iterations updated " << maxIterations << std::endl;
                    }
                }
            }
        }
    }

    void splitCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud, const std::unordered_set<int>& inliers, CloudSegment<PointT>& cloudSegment)
    {
        cloudSegment.plane = typename pcl::PointCloud<PointT>::Ptr{new pcl::PointCloud<PointT>};
        cloudSegment.obstacles = typename pcl::PointCloud<PointT>::Ptr{new pcl::PointCloud<PointT>};

        int cloudSize = cloud->size();
        for (int i = 0; i < cloudSize; ++i)
        {
            if (inliers.find(i) == inliers.end())
            {
                cloudSegment.obstacles->points.push_back(cloud->points[i]);
            }
            else
            {
                cloudSegment.plane->points.push_back(cloud->points[i]);
            }
        }
    }

    void segmentAndSplit(const typename pcl::PointCloud<PointT>::Ptr& cloud, CloudSegment<PointT>& cloudSegment)
    {
        auto startTime = std::chrono::steady_clock::now();

        std::unordered_set<int> inliers;
        segment(cloud, inliers, cloudSegment.planeCoeffs);
        splitCloud(cloud, inliers, cloudSegment);

        auto endTime = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
        std::cout << "RANSAC 3D plane segmentation took " << duration.count() << " mcs" << std::endl;
    }
};

#endif /* RANSAC_H_ */