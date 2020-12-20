/* \author Victor Mauze, Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include <string>
#include <map>
#include <ctype.h>
#include <algorithm>

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"

// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

#include "tools.h"
#include "ransac.h"
#include "euclideanClustering.h"
#include "cloudSegment.h"
#include "trackingEng.h"

template<typename PointT>
class Environment
{
private:
    // RENDER OPTIONS
    // Render scene object
    bool renderScene = false;
    bool renderSceneRays = false;

    // CAMERA OPTIONS
    // Distance away in meters
    int cameraDistance = 16;

    // LIDAR OPTIONS 
    // Ground slope
    double groundSlope = 0;

    // POINT PROCESSOR OPTIONS
    int planeDetectorMaxIterations = 1000;
    double planeDetectorDistanceThreshold = 0.3;

    // CLUSTERING OBSTACLES
    double clusterTolerance = 0.3;
    int minClusterSize = 10;
    int maxClusterSize = 1000;

    // CUSTOM METHODS
    bool useCustomRansac = true;
    bool useCustomClustering = true;
    bool usePCABoxes = false;

    // FILTERING
    float gridCellSize = 0.25;
    float boxDistAhead = 30;
    float boxDistBack = 15;
    float boxDistLeft = 7;
    float boxDistRight = 6;
    float boxDistDown = 3;
    float boxDistUp = 1;

    float egoBoxLength = 5.5;
    float egoBoxWidth = 3.1;
    float egoBoxHeight = 1.5;

    // STREAMING
    std::string rootFolderName = "../src/sensors/data/pcd/data_1/";
    bool streamFolder = true;
    bool streamRepeat = true;
    bool stepByStep = false;
    int streamDelay = 1;

private:
    pcl::visualization::PCLVisualizer::Ptr viewer;
    std::vector<Car> cars;
    Lidar<PointT>* lidar = nullptr;

    ProcessPointClouds<PointT> processor;
    RansacPlane3D<PointT> ransacProcessor;
    EuclideanClustering<PointT> euclideanClusteringProcessor;
    typename pcl::PointCloud<PointT>::Ptr scannedPoints;
    CloudSegment<PointT> cloudSegment;
    TrackingEng* trackingEng = nullptr;

    void createLidar()
    {
        deleteLidar();

        lidar = new Lidar<PointT>{cars, groundSlope};
    }

    void deleteLidar()
    {
        if (lidar != nullptr)
        {
            delete lidar;
            lidar = nullptr;
        }
    }

    void createTrackingEng()
    {
        deleteTrackingEng();

        trackingEng = new TrackingEng;
    }

    void deleteTrackingEng()
    {
        if (trackingEng != nullptr)
        {
            delete trackingEng;
            trackingEng = nullptr;
        }
    }

    void initHighway()
    {
        Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
        Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
        Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
        Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");

        cars.clear();
        cars.push_back(egoCar);
        cars.push_back(car1);
        cars.push_back(car2);
        cars.push_back(car3);

        if (renderScene)
        {
            renderHighway(viewer);
            egoCar.render(viewer);
            car1.render(viewer);
            car2.render(viewer);
            car3.render(viewer);
        }
    }

    void scanPoints()
    {
        scannedPoints = lidar->scan();
    }

    void renderScannedPoints(Color color = Color(-1, -1, -1))
    {
        if (renderSceneRays)
        {
            renderRays(viewer, cars[0].position + lidar->position, scannedPoints);
        }
        else
        {
            renderPointCloud(viewer, scannedPoints, "LIDAR point cloud", color);
        }
    }

    void processPoints()
    {
        if (useCustomRansac)
        {
            ransacProcessor.setDistanceTolerance(planeDetectorDistanceThreshold);

            ransacProcessor.segmentAndSplit(scannedPoints, cloudSegment);
        }
        else
        {
            processor.SegmentPlane(scannedPoints, planeDetectorMaxIterations, planeDetectorDistanceThreshold, cloudSegment);
        }
    }

    void clusterObstacles()
    {
        if (useCustomClustering)
        {
            euclideanClusteringProcessor.setClusterTolerance(clusterTolerance);
            euclideanClusteringProcessor.setMinClusterSize(minClusterSize);
            euclideanClusteringProcessor.setMaxClusterSize(maxClusterSize);

            euclideanClusteringProcessor.clustering(cloudSegment.obstacles, cloudSegment);
        }
        else
        {
            processor.Clustering(cloudSegment.obstacles, clusterTolerance, minClusterSize, maxClusterSize, cloudSegment);
        }
    }

    void renderProcessedPoints()
    {
        if (cloudSegment.planeCoeffs.size() >= 4)
        {
            std::cout << "Plane equation " <<
                cloudSegment.planeCoeffs[0] << " X + " <<
                cloudSegment.planeCoeffs[1] << " Y + " <<
                cloudSegment.planeCoeffs[2] << " Z + " <<
                cloudSegment.planeCoeffs[3] << "" << std::endl;
        }

        cloudSegment.calcPlaneTransform();

        Color clrPlane(0, 1, 0); // Green
        renderPointCloud(viewer, cloudSegment.plane, "Plane points", clrPlane);

        std::vector<TrackingObject> origState;

        int clustersNum = cloudSegment.obstacleClusters.size();
        for (int i = 0; i < clustersNum; ++i)
        {
            origState.push_back(cloudSegment.trackingObject(cloudSegment.obstacleClusters[i], i, usePCABoxes));
        }

        std::vector<TrackingObject> state;
        cloudSegment.mergeTrackingObjects(origState, state, clusterTolerance);

        trackingEng->updateState(state);

        int boxIdx = 0;
        for (auto& trackingObject: trackingEng->state)
        {
            if (trackingObject.invisibleFactor == 0)
            {
                renderPointCloud(viewer, cloudSegment.obstacleClusters[trackingObject.clusterIdx], "Obstacle " + std::to_string(trackingObject.trackingObjIdx), trackingObject.trackingColor);

                if (!usePCABoxes && trackingObject.collisionNested.size() > 0)
                {
                    renderBox(viewer, trackingObject.renderBoxQ(trackingObject.collisionShape), ++boxIdx, trackingObject.trackingColor, 0.2);

                    for (CollisionBox& nested: trackingObject.collisionNested)
                    {
                        renderBox(viewer, trackingObject.renderBoxQ(nested), ++boxIdx, trackingObject.trackingColor, 0.5);
                    }
                }
                else
                {
                    renderBox(viewer, trackingObject.renderBoxQ(trackingObject.collisionShape), ++boxIdx, trackingObject.trackingColor, 0.5);
                }
            }
        }

        Color clrNoise(0.5, 0.5, 0.5); // Gray
        renderPointCloud(viewer, cloudSegment.noiseCluster, "Noise points", clrNoise);

        renderBox(
            viewer,
            Box{-egoBoxLength / 2, -egoBoxWidth / 2, -egoBoxHeight, egoBoxLength / 2, egoBoxWidth / 2, 0},
            ++boxIdx,
            Color{0.25, 0, 0},
            0.5);
    }

public:
    Environment(const std::string& viwerName):
        viewer{new pcl::visualization::PCLVisualizer{viwerName}}
    {
        createTrackingEng();
    }

    ~Environment()
    {
        deleteLidar();
        deleteTrackingEng();
    }

    bool setRenderScene()
    {
        return renderScene;
    }

    void setRenderScene(const bool renderScene)
    {
        this->renderScene = renderScene;
    }

    bool getRenderSceneRays()
    {
        return renderSceneRays;
    }

    void setRenderSceneRays(bool renderSceneRays)
    {
        this->renderSceneRays = renderSceneRays;
    }

    double getGroundSlope()
    {
        return groundSlope;
    }

    void setGroundSlope(const double groundSlope)
    {
        this->groundSlope = groundSlope;
    }

    int getCameraDistance()
    {
        return cameraDistance;
    }

    void setCameraDistance(const int cameraDistance)
    {
        this->cameraDistance = cameraDistance;
    }

    int getPlaneDetectorMaxIterations()
    {
        return planeDetectorMaxIterations;
    }

    void setPlaneDetectorMaxIterations(int planeDetectorMaxIterations)
    {
        this->planeDetectorMaxIterations = planeDetectorMaxIterations;
    }

    float getPlaneDetectorDistanceThreshold()
    {
        return planeDetectorDistanceThreshold;
    }

    void setPlaneDetectorDistanceThreshold(float planeDetectorDistanceThreshold)
    {
        this->planeDetectorDistanceThreshold = planeDetectorDistanceThreshold;
    }

    bool getUseCustomRansac()
    {
        return useCustomRansac;
    }

    void setUseCustomRansac(bool useCustomRansac)
    {
        this->useCustomRansac = useCustomRansac;
    }

    bool getUseCustomClustering()
    {
        return useCustomClustering;
    }

    void setUseCustomClustering(bool useCustomClustering)
    {
        this->useCustomClustering = useCustomClustering;
    }

    bool getUsePCABoxes()
    {
        return usePCABoxes;
    }

    void setUsePCABoxes(bool usePCABoxes)
    {
        this->usePCABoxes = usePCABoxes;
    }

    double getClusterTolerance()
    {
        return clusterTolerance;
    }

    void setClusterTolerance(double clusterTolerance)
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

    float getGridCellSize()
    {
        return gridCellSize;
    }

    void setGridCellSize(float gridCellSize)
    {
        this->gridCellSize = gridCellSize;
    }

    float getBoxDistAhead()
    {
        return boxDistAhead;
    }

    void setBoxDistAhead(float boxDistAhead)
    {
        this->boxDistAhead = boxDistAhead;
    }

    float getBoxDistBack()
    {
        return boxDistBack;
    }

    void setBoxDistBack(float boxDistBack)
    {
        this->boxDistBack = boxDistBack;
    }

    float getBoxDistLeft()
    {
        return boxDistLeft;
    }

    void setBoxDistLeft(float boxDistLeft)
    {
        this->boxDistLeft = boxDistLeft;
    }

    float getBoxDistRight()
    {
        return boxDistRight;
    }

    void setBoxDistRight(float boxDistRight)
    {
        this->boxDistRight = boxDistRight;
    }

    float getBoxDistDown()
    {
        return boxDistDown;
    }

    void setBoxDistDown(float boxDistDown)
    {
        this->boxDistDown = boxDistDown;
    }

    float getBoxDistUp()
    {
        return boxDistUp;
    }

    void setBoxDistUp(float boxDistUp)
    {
        this->boxDistUp = boxDistUp;
    }

    float getEgoBoxLength()
    {
        return egoBoxLength;
    }

    void setEgoBoxLength(float egoBoxLength)
    {
        this->egoBoxLength = egoBoxLength;
    }

    float getEgoBoxWidth()
    {
        return egoBoxWidth;
    }

    void setEgoBoxWidth(float egoBoxWidth)
    {
        this->egoBoxWidth = egoBoxWidth;
    }

    float getEgoBoxHeight()
    {
        return egoBoxHeight;
    }

    void setEgoBoxHeight(float egoBoxHeight)
    {
        this->egoBoxHeight = egoBoxHeight;
    }

    std::string getRootFolderName()
    {
        return rootFolderName;
    }

    void setRootFolderName(std::string rootFolderName)
    {
        this->rootFolderName = rootFolderName;
    }

    bool getStreamFolder()
    {
        return streamFolder;
    }

    void setStreamFolder(bool streamFolder)
    {
        this->streamFolder = streamFolder;
    }

    bool getStreamRepeat()
    {
        return streamRepeat;
    }

    void setStreamRepeat(bool streamRepeat)
    {
        this->streamRepeat = streamRepeat;
    }

    bool getStepByStep()
    {
        return stepByStep;
    }

    void setStepByStep(bool stepByStep)
    {
        this->stepByStep = stepByStep;
    }

    int getStreamDelay()
    {
        return streamDelay;
    }

    void setStreamDelay(int streamDelay)
    {
        this->streamDelay = streamDelay;
    }

    void setParmMap(std::map<std::string, std::string>& parm)
    {
        for (auto& p: parm)
        {
            std::string uParm = strUpr(p.first);
            std::string uVal = strUpr(p.second);

            if (uParm == strUpr("RenderScene"))
            {
                renderScene = uVal == "TRUE" || uVal == "1";
            }
            else if (uParm == strUpr("RenderSceneRays"))
            {
                renderSceneRays = uVal == "TRUE" || uVal == "1";
            }
            else if (uParm == strUpr("CameraDistance"))
            {
                cameraDistance = std::stoi(uVal);
            }
            else if (uParm == strUpr("GroundSlope"))
            {
                groundSlope = std::stod(uVal);
            }
            else if (uParm == strUpr("PlaneDetectorMaxIterations"))
            {
                planeDetectorMaxIterations = std::stoi(uVal);
            }
            else if (uParm == strUpr("PlaneDetectorDistanceThreshold"))
            {
                planeDetectorDistanceThreshold = std::stod(uVal);
            }
            else if (uParm == strUpr("UseCustomRansac"))
            {
                useCustomRansac = uVal == "TRUE" || uVal == "1";
            }
            else if (uParm == strUpr("UseCustomClustering"))
            {
                useCustomClustering = uVal == "TRUE" || uVal == "1";
            }
            else if (uParm == strUpr("UsePCABoxes"))
            {
                usePCABoxes = uVal == "TRUE" || uVal == "1";
            }
            else if (uParm == strUpr("ClusterTolerance"))
            {
                clusterTolerance = std::stod(uVal);
            }
            else if (uParm == strUpr("MinClusterSize"))
            {
                minClusterSize = std::stoi(uVal);
            }
            else if (uParm == strUpr("MaxClusterSize"))
            {
                maxClusterSize = std::stoi(uVal);
            }
            else if (uParm == strUpr("GridCellSize"))
            {
                gridCellSize = std::stof(uVal);
            }
            else if (uParm == strUpr("BoxDistAhead"))
            {
                boxDistAhead = std::stof(uVal);
            }
            else if (uParm == strUpr("BoxDistBack"))
            {
                boxDistBack = std::stof(uVal);
            }
            else if (uParm == strUpr("BoxDistLeft"))
            {
                boxDistLeft = std::stof(uVal);
            }
            else if (uParm == strUpr("BoxDistRight"))
            {
                boxDistRight = std::stof(uVal);
            }
            else if (uParm == strUpr("BoxDistDown"))
            {
                boxDistDown = std::stof(uVal);
            }
            else if (uParm == strUpr("BoxDistUp"))
            {
                boxDistUp = std::stof(uVal);
            }
            else if (uParm == strUpr("EgoBoxLength"))
            {
                egoBoxLength = std::stof(uVal);
            }
            else if (uParm == strUpr("EgoBoxWidth"))
            {
                egoBoxWidth = std::stof(uVal);
            }
            else if (uParm == strUpr("EgoBoxHeight"))
            {
                egoBoxHeight = std::stof(uVal);
            }
            else if (uParm == strUpr("RootFolderName"))
            {
                rootFolderName = p.second;
            }
            else if (uParm == strUpr("StreamFolder"))
            {
                streamFolder = uVal == "TRUE" || uVal == "1";
            }
            else if (uParm == strUpr("StreamRepeat"))
            {
                streamRepeat = uVal == "TRUE" || uVal == "1";
            }
            else if (uParm == strUpr("StepByStep"))
            {
                stepByStep = uVal == "TRUE" || uVal == "1";
            }
            else if (uParm == strUpr("StreamDelay"))
            {
                streamDelay = std::stoi(uVal);
            }
            else
            {
                if (!ransacProcessor.setParm(p.first, uParm, p.second, uVal))
                {
                    std::cout << "Incorrect parameter " << p.first << endl;
                }
            }
        }
    }

    void simpleHighway()
    {
        // ----------------------------------------------------
        // -----Open 3D viewer and display simple highway -----
        // ----------------------------------------------------
        
        // RENDER OPTIONS
        initHighway();
        
        // TODO:: Create lidar sensor
        createLidar();

        // TODO:: Create point processor
        scanPoints();
        //renderScannedPoints(Color(1, 1, 0));

        // Process points
        processPoints();
        clusterObstacles();
        renderProcessedPoints();
    }

    //setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
    void initCamera(const CameraAngle setAngle)
    {
        viewer->setBackgroundColor (0, 0, 0);
        
        // set camera position and angle
        viewer->initCameraParameters();
        
        switch(setAngle)
        {
            case XY : viewer->setCameraPosition(-cameraDistance, -cameraDistance, cameraDistance, 1, 1, 0); break;
            case TopDown : viewer->setCameraPosition(0, 0, cameraDistance, 1, 0, 1); break;
            case Side : viewer->setCameraPosition(0, -cameraDistance, 0, 0, 0, 1); break;
            case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
        }

        if (setAngle != FPS)
        {
            viewer->addCoordinateSystem (1.0);
        }
    }

    void spin()
    {
        while (!viewer->wasStopped())
        {
            viewer->spinOnce();
        } 
    }

    void loadCityBlockPoints(const std::string& file)
    {
        scannedPoints = processor.loadPcd(file);
    }

    void filterPointsCloud()
    {
        Eigen::Vector4f minPoint{-boxDistBack, -boxDistRight, -boxDistDown, 1};
        Eigen::Vector4f maxPoint{boxDistAhead, boxDistLeft, boxDistUp, 1};
        
        Eigen::Vector4f egoMinPoint{-egoBoxLength / 2, -egoBoxWidth / 2, -egoBoxHeight, 1};
        Eigen::Vector4f egoMaxPoint{egoBoxLength / 2, egoBoxWidth / 2, 0, 1};

        scannedPoints = processor.FilterCloud(scannedPoints, gridCellSize, minPoint, maxPoint, egoMinPoint, egoMaxPoint);
    }

    void processCityBlock()
    {
        filterPointsCloud();
        //renderScannedPoints();

        // Process points
        processPoints();
        clusterObstacles();
        
        renderProcessedPoints();
    }

    void cleanupViewer()
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
    }

    void loadAndProcess()
    {
        std::vector<boost::filesystem::path> stream = processor.streamPcd(rootFolderName);
        auto streamIt = stream.begin();
        if (streamIt == stream.end())
        {
            std::cout << "Folder has no files to display." << std::endl;
            return;
        }

        bool stopUpdate = false;

        while (!viewer->wasStopped())
        {
            if (!stopUpdate)
            {
                cleanupViewer();

                loadCityBlockPoints(streamIt->string());
                processCityBlock();

                if (streamFolder)
                {
                    if (stepByStep)
                    {
                        std::string dummy;
                        std::cin >> dummy;
                    }

                    ++streamIt;
                    if (streamIt == stream.end())
                    {
                        if (streamRepeat)
                        {
                            streamIt = stream.begin();

                            createTrackingEng();
                        }
                        else
                        {
                            stopUpdate = true;
                        }
                    }
                }
                else
                {
                    stopUpdate = true;
                }
            }

            for (int i = 0 ; i < streamDelay; ++i)
            {
                viewer->spinOnce();
            }
        }
    }
};

std::map<std::string, std::string> parseParameters(int argc, char** argv)
{
    std::map<std::string, std::string> parmMap;
    int state = 0;
    std::string parmName;
    std::string parmValue;

    for (int i = 1; i < argc; ++i)
    {
        std::string s = argv[i];

        if (s == "--p" || s == "--P")
        {
            state = 1;
        }
        else if (state == 1)
        {
            parmName = s;
            ++state;
        }
        else if (state == 2)
        {
            parmValue = s;
            ++state;

            parmMap.insert(std::pair<std::string, std::string>{parmName, parmValue});
        }
    }

    return parmMap;
}

// MODEL OPTIONS
bool useCityBlocks = true;

int main (int argc, char** argv)
{
    srand(time(NULL));

    std::map<std::string, std::string> parm = parseParameters(argc, argv);

    std::cout << "starting enviroment" << std::endl;

    if (useCityBlocks)
    {
        Environment<pcl::PointXYZI> env{"3D Viewer"};

        CameraAngle setAngle = XY;
        env.initCamera(setAngle);
        env.setParmMap(parm);

        env.loadAndProcess();
    }
    else
    {
        Environment<pcl::PointXYZ> env{"3D Viewer"};

        CameraAngle setAngle = XY;
        env.initCamera(setAngle);
        env.setClusterTolerance(2);
        env.setMinClusterSize(3);
        env.setMaxClusterSize(200);
        env.setParmMap(parm);

        env.simpleHighway();
        
        env.spin();
    }
}