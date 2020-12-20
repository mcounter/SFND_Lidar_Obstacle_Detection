/* \author Victor Mauze, Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include <cmath>
#include <chrono>
#include <algorithm>

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

int RansacMaxIterations(const int& subsetSize, const double& successProb, const double& inliersProb)
{
	double w = std::pow(inliersProb, subsetSize);

	return std::log(1 - successProb) / std::log(1 - w) + std::sqrt(1 - w) / w + 0.5;
}

std::unordered_set<int> Ransac(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
	const double& distanceTol, const double& successProb, const double& inliersProb, const double& optFactor)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	
	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers

	int cloudSize = cloud->size();
	std::cout << "Cloud size " << cloudSize << std::endl;

	int maxIterations = RansacMaxIterations(3, successProb, inliersProb);
	std::cout << "Maximum iterations calculated " << maxIterations << std::endl;

	if (cloudSize >= 3)
	{
		int inliersResultSize = inliersResult.size();
		
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

			pcl::PointXYZ& point1 = cloud->points[pidx[0]];
			pcl::PointXYZ& point2 = cloud->points[pidx[1]];
			pcl::PointXYZ& point3 = cloud->points[pidx[2]];

			double a = (point2.y - point1.y) * (point3.z - point1.z) - (point2.z - point1.z) * (point3.y - point1.y);
			double b = (point2.z - point1.z) * (point3.x - point1.x) - (point2.x - point1.x) * (point3.z - point1.z);
			double c = (point2.x - point1.x) * (point3.y - point1.y) - (point2.y - point1.y) * (point3.x - point1.x);
			double d = -(a * point1.x + b * point1.y + c * point1.z);
			
			double distanceTolNorm = distanceTol * std::sqrt(a*a + b*b + c*c);

			if (distanceTolNorm > 0)
			{
				for (int idx = 0; idx < cloudSize; ++idx)
				{
					pcl::PointXYZ& pointX = cloud->points[idx];
					double dist = std::fabs(a * pointX.x + b * pointX.y + c * pointX.z + d);
					if (dist < distanceTolNorm)
					{
						inliersIteration.insert(idx);
					}
				}
			}

			int inliersIterSize = inliersIteration.size();
			if (inliersIterSize > inliersResultSize)
			{
				inliersResult = inliersIteration;
				inliersResultSize = inliersIterSize;

				if (optFactor > 0)
				{
					maxIterations = std::min(maxIterations, i + 1 + RansacMaxIterations(3, successProb, optFactor * (double)inliersResultSize / (double)cloudSize));
					std::cout << "Maximum iterations updated " << maxIterations << std::endl;
				}
			}
		}
	}
	
	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	
	// Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 0.3, 0.99999, 0.3, 0.9);

	auto endTime = std::chrono::steady_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
	std::cout << "RANSAC took " << duration.count() << " mcs" << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
