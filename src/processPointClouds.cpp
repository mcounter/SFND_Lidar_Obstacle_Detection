// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud, float& filterRes,
        Eigen::Vector4f& minPoint, Eigen::Vector4f& maxPoint,
        Eigen::Vector4f& egoMinPoint, Eigen::Vector4f& egoMaxPoint)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    typename pcl::PointCloud<PointT>::Ptr filtered{new pcl::PointCloud<PointT>};

    pcl::VoxelGrid<PointT> gridFilter;
    gridFilter.setInputCloud(cloud);
    gridFilter.setLeafSize(filterRes, filterRes, filterRes);
    gridFilter.filter(*filtered);

    typename pcl::PointCloud<PointT>::Ptr cropped{new pcl::PointCloud<PointT>};
    pcl::CropBox<PointT> cropBox;
    cropBox.setMin(minPoint);
    cropBox.setMax(maxPoint);
    cropBox.setInputCloud(filtered);
    cropBox.filter(*cropped);

    pcl::PointIndices::Ptr egoIndices{new pcl::PointIndices};
    pcl::CropBox<PointT> egoCropBox;
    egoCropBox.setMin(egoMinPoint);
    egoCropBox.setMax(egoMaxPoint);
    egoCropBox.setInputCloud(cropped);
    egoCropBox.filter(egoIndices->indices);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cropped);
    extract.setIndices(egoIndices);

    typename pcl::PointCloud<PointT>::Ptr croppedNoEgo{new pcl::PointCloud<PointT>};
    extract.setNegative(true);
    extract.filter(*croppedNoEgo);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
    std::cout << "Filtering took " << elapsedTime.count() << " mcs" << std::endl;

    return croppedNoEgo;

}


template<typename PointT>
void ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud, CloudSegment<PointT>& cloudSegment) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);

    typename pcl::PointCloud<PointT>::Ptr plane{new pcl::PointCloud<PointT>};
    extract.setNegative(false);
    extract.filter(*plane);

    typename pcl::PointCloud<PointT>::Ptr obstacles{new pcl::PointCloud<PointT>};
    extract.setNegative(true);
    extract.filter(*obstacles);

    cloudSegment.obstacles = obstacles;
    cloudSegment.plane = plane;
}


template<typename PointT>
void ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold, CloudSegment<PointT>& cloudSegment)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::SACSegmentation<PointT> segm;
    segm.setOptimizeCoefficients(true);
    segm.setModelType(pcl::SACMODEL_PLANE);
    segm.setMethodType(pcl::SAC_RANSAC);

    segm.setDistanceThreshold(distanceThreshold);
    segm.setMaxIterations(maxIterations);

    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coeff{new pcl::ModelCoefficients};
    
    segm.setInputCloud(cloud);
    segm.segment(*inliers, *coeff);

    cloudSegment.planeCoeffs.clear();
    for (auto v: coeff->values)
    {
        cloudSegment.planeCoeffs.push_back(v);
    }

    SeparateClouds(inliers, cloud, cloudSegment);

    auto endTime = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
    std::cout << "PCL plane segmentation took " << duration.count() << " mcs" << std::endl;
}


template<typename PointT>
void ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize, CloudSegment<PointT>& cloudSegment)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::unordered_set<int> clustered_indices;

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> clustering;
    clustering.setClusterTolerance(clusterTolerance);
    clustering.setMinClusterSize(minSize);
    clustering.setMaxClusterSize(maxSize);
    clustering.setSearchMethod(tree);
    clustering.setInputCloud(cloud);
    clustering.extract(clusterIndices);

    cloudSegment.obstacleClusters.clear();
    for(auto& indices: clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cluster(new typename pcl::PointCloud<PointT>);

        for (int index: indices.indices)
        {
            clustered_indices.insert(index);
            cluster->push_back((*cloud)[index]);
        }

        cluster->width = cluster->size();
        cluster->height = 1;
        cluster->is_dense = true;

        cloudSegment.obstacleClusters.push_back(cluster);
    }

    cloudSegment.noiseCluster = typename pcl::PointCloud<PointT>::Ptr{new typename pcl::PointCloud<PointT>};

    int cloudSize = cloud->size();
    for (int index = 0; index < cloudSize; ++index)
    {
        if (clustered_indices.find(index) == clustered_indices.end())
        {
            cloudSegment.noiseCluster->push_back((*cloud)[index]);
        }
    }

    cloudSegment.noiseCluster->width = cloudSegment.noiseCluster->size();
    cloudSegment.noiseCluster->height = 1;
    cloudSegment.noiseCluster->is_dense = true;

    auto endTime = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
    std::cout << "Euclidian clustering took " << duration.count() << " mcs and found " << cloudSegment.obstacleClusters.size() << " clusters" << std::endl;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(const std::string& file)
{
    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(const std::string& dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}