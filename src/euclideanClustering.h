#ifndef EUCLIDEANCLUSTERING_H_
#define EUCLIDEANCLUSTERING_H_

#include <vector>
#include <chrono>
#include <pcl/point_cloud.h>

#include "kdtree.h"

template <typename PointT>
class EuclideanClustering
{
private:
    KdTree* tree;
    float clusterTolerance = 1;
    int minClusterSize = -1;
    int maxClusterSize = -1;

    std::vector<double> cloudPointToVector(const PointT& point)
    {
        return std::vector<double>{point.x, point.y, point.z};
    }

    void createKdTree(const typename pcl::PointCloud<PointT>::Ptr cloud)
    {
        if (tree != nullptr)
        {
            delete tree;
        }

        tree = new KdTree;

        int n = cloud->points.size();
        for (int i = 0; i < n; ++i)
        {
            tree->insert(cloudPointToVector(cloud->points[i]), i);
        }
    }

public:
    EuclideanClustering() : tree(nullptr) {}
    ~EuclideanClustering()
    {
        if (tree != nullptr)
        {
            delete tree;
        }
    }

    float getClusterTolerance()
    {
        return clusterTolerance;
    }

    void setClusterTolerance(float clusterTolerance)
    {
        this->clusterTolerance = clusterTolerance;
    }

    int getMinClusterSize()
    {
        return minClusterSize;
    }

    void setMinClusterSize(int minClusterSize)
    {
        this->minClusterSize = minClusterSize;
    }

    int getMaxClusterSize()
    {
        return maxClusterSize;
    }

    void setMaxClusterSize(int maxClusterSize)
    {
        this->maxClusterSize = maxClusterSize;
    }

    void clustering(const typename pcl::PointCloud<PointT>::Ptr cloud, CloudSegment<PointT>& cloudSegment)
    {
        auto startTime = std::chrono::steady_clock::now();

        cloudSegment.obstacleClusters.clear();
        cloudSegment.noiseCluster = typename pcl::PointCloud<PointT>::Ptr{new typename pcl::PointCloud<PointT>};

        int n = cloud->points.size();
        if (n > 0)
        {
            createKdTree(cloud);

            std::vector<bool> processedPoints(n, false);

            for (int i = 0; i < n; ++i)
            {
                if (!processedPoints[i])
                {
                    typename pcl::PointCloud<PointT>::Ptr cluster{new typename pcl::PointCloud<PointT>};
                    
                    std::vector<int> queue;
                    queue.push_back(i);
                    int qPos = 0;

                    while (qPos < queue.size())
                    {
                        int idx = queue[qPos];

                        if (!processedPoints[idx])
                        {
                            processedPoints[idx] = true;
                            cluster->points.push_back(cloud->points[idx]);
                            
                            std::vector<int> nearbyPoints;
                            tree->search(cloudPointToVector(cloud->points[idx]), clusterTolerance, nearbyPoints);

                            for (int nextIdx : nearbyPoints)
                            {
                                if (!processedPoints[nextIdx])
                                {
                                    queue.push_back(nextIdx);
                                }
                            }
                        }

                        ++qPos;
                    }

                    int clusterSize = cluster->points.size();
                    if (clusterSize > 0)
                    {
                        if ((minClusterSize < 0 || clusterSize >= minClusterSize) &&
                            (maxClusterSize < 0 || clusterSize <= maxClusterSize))
                        {
                            cloudSegment.obstacleClusters.push_back(cluster);
                        }
                        else
                        {
                            for (auto& point: cluster->points)
                            {
                                cloudSegment.noiseCluster->points.push_back(point);
                            }
                        }
                    }
                }
            }
        }

        auto endTime = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
        std::cout << "Custom Euclidian clustering took " << duration.count() << " mcs and found " << cloudSegment.obstacleClusters.size() << " clusters" << std::endl;
    }
};

#endif /* EUCLIDEANCLUSTERING_H_ */