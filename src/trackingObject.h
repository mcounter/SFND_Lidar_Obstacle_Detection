#ifndef TRACKING_OBJECT_H_
#define TRACKING_OBJECT_H_

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "render/render.h"
#include "render/box.h"

struct CollisionBox
{
	Eigen::Vector3d minVert;
    Eigen::Vector3d maxVert;

    bool isCollisionBoxClose(const CollisionBox& nextBox, const double& distTolerance) const
    {
        for (int i = 0; i < 3; ++i)
        {
            if ((maxVert[i] < nextBox.minVert[i] && (nextBox.minVert[i] - maxVert[i]) > distTolerance) ||
                (nextBox.maxVert[i] < minVert[i] && (minVert[i] - nextBox.maxVert[i]) > distTolerance))
            {
                return false;
            }
        }

        return true;
    }
};

struct TrackingObject
{
    CollisionBox collisionShape;
    Eigen::Affine3d renderTransform;

    Eigen::Vector3d trackingCenter;

    int trackingObjIdx = 0;
    Color trackingColor = Color{0, 0, 0};
    int invisibleFactor = 0;
    int clusterIdx = 0;

    std::vector<CollisionBox> collisionNested;

    BoxQ renderBoxQ(const CollisionBox& box)
    {
        Eigen::Vector4d boxCenter{
            (box.maxVert[0] + box.minVert[0]) / 2.0,
            (box.maxVert[1] + box.minVert[1]) / 2.0,
            (box.maxVert[2] + box.minVert[2]) / 2.0,
            1};

        boxCenter = renderTransform * boxCenter;

        BoxQ boxQ;
        boxQ.bboxTransform = Eigen::Vector3f{float(boxCenter[0]), float(boxCenter[1]), float(boxCenter[2])};
        boxQ.cube_length = box.maxVert[0] - box.minVert[0];
        boxQ.cube_width = box.maxVert[1] - box.minVert[1];
        boxQ.cube_height = box.maxVert[2] - box.minVert[2];
        
        Eigen::Matrix3f rotMatrixf = renderTransform.matrix().cast<float>().block(0, 0, 3, 3);
        boxQ.bboxQuaternion = Eigen::Quaternionf(rotMatrixf);

        return boxQ;
    }

    bool isTrackingObjectClose(const TrackingObject& nextObj, const double& distTolerance) const
    {
        if (collisionNested.size() > 0)
        {
            for (const CollisionBox& curBox : collisionNested)
            {
                if (nextObj.collisionNested.size() > 0)
                {
                    for (const CollisionBox& nextBox : nextObj.collisionNested)
                    {
                        if (curBox.isCollisionBoxClose(nextBox, distTolerance))
                        {
                            return true;
                        }
                    }
                }
                else if (curBox.isCollisionBoxClose(nextObj.collisionShape, distTolerance))
                {
                    return true;
                }
            }
        }
        else
        {
            if (nextObj.collisionNested.size() > 0)
            {
                for (const CollisionBox& nextBox : nextObj.collisionNested)
                {
                    if (collisionShape.isCollisionBoxClose(nextBox, distTolerance))
                    {
                        return true;
                    }
                }
            }
            else if (collisionShape.isCollisionBoxClose(nextObj.collisionShape, distTolerance))
            {
                return true;
            }
        }
        
        return false;
    }

    void mergeWithObject(const TrackingObject& nextObj)
    {
        if (collisionNested.size() <= 0)
        {
            collisionNested.push_back(collisionShape);
        }

        if (nextObj.collisionNested.size() <= 0)
        {
            collisionNested.push_back(nextObj.collisionShape);
        }
        else
        {
            for (const CollisionBox& nextBox : nextObj.collisionNested)
            {
                collisionNested.push_back(nextBox);
            }
        }
        
        for (int i = 0; i < 3; ++i)
        {
            if (nextObj.collisionShape.minVert[i] < collisionShape.minVert[i])
            {
                collisionShape.minVert[i] = nextObj.collisionShape.minVert[i];
            }

            if (nextObj.collisionShape.maxVert[i] > collisionShape.maxVert[i])
            {
                collisionShape.maxVert[i] = nextObj.collisionShape.maxVert[i];
            }
        }
    }
};

#endif /* TRACKING_OBJECT_H_ */
