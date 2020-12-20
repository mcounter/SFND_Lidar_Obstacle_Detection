#ifndef TRACKING_ENG_H_
#define TRACKING_ENG_H_

#include <vector>
#include <algorithm>
#include <cmath>
#include "trackingObject.h"

struct TrackingProximity
{
    double dist;
    int objIdx1;
    int objIdx2;
};

class TrackingEng
{
private:
    const std::vector<Color> baseColors{
        Color{1, 0, 0}, // Red
        Color{0, 0, 1}, // Blue
        Color{1, 1, 0}, // Yellow
        Color{0, 1, 1}, // Aqua
        Color{1, 0, 1}, // Magenta
    };

    int maxInvisibleFactor = 5;
    double maxMatchRange = 3.0;

    std::vector<Color> colorStack;
    int nextColorIdx = 0;
    int nextObjIdx = 1;

    static bool sortByInvisibleFactor(const TrackingObject& obj1, const TrackingObject& obj2)
    {
        return obj1.invisibleFactor < obj2.invisibleFactor;
    }

    void sortAndReduce()
    {
        int expiredNum = 0;

        int n = state.size();
        for (int i = 0; i < n; ++i)
        {
            if (state[i].invisibleFactor >= maxInvisibleFactor)
            {
                ++expiredNum;
                colorStack.push_back(state[i].trackingColor);
            }
        }

        std::sort(state.begin(), state.end(), sortByInvisibleFactor);

        if (expiredNum > 0)
        {
            state.resize(state.size() - expiredNum);
        }
    }

    static bool sortByProximity(const TrackingProximity& obj1, const TrackingProximity& obj2)
    {
        return obj1.dist < obj2.dist;
    }

    void calcProximityVector(const std::vector<TrackingObject>& newState, std::vector<TrackingProximity>& proximity)
    {
        proximity.clear();

        int stateSize = state.size();
        int newStateSize = newState.size();

        for (int objIdx1 = 0; objIdx1 < stateSize; ++objIdx1)
        {
            for (int objIdx2 = 0; objIdx2 < newStateSize; ++objIdx2)
            {
                double dist = std::sqrt(
                    (state[objIdx1].trackingCenter[0] - newState[objIdx2].trackingCenter[0]) * (state[objIdx1].trackingCenter[0] - newState[objIdx2].trackingCenter[0]) +
                    (state[objIdx1].trackingCenter[1] - newState[objIdx2].trackingCenter[1]) * (state[objIdx1].trackingCenter[1] - newState[objIdx2].trackingCenter[1]) +
                    (state[objIdx1].trackingCenter[2] - newState[objIdx2].trackingCenter[2]) * (state[objIdx1].trackingCenter[2] - newState[objIdx2].trackingCenter[2]));

                if (dist <= maxMatchRange)
                {
                    proximity.push_back(TrackingProximity{dist, objIdx1, objIdx2});
                }
            }
        }

        std::sort(proximity.begin(), proximity.end(), sortByProximity);
    }

public:
    std::vector<TrackingObject> state;

    void updateState(const std::vector<TrackingObject>& newState)
    {
        std::vector<TrackingProximity> proximity;
        calcProximityVector(newState, proximity);

        int stateSize = state.size();
        std::vector<bool> touchedState(stateSize, false);

        int newStateSize = newState.size();
        std::vector<bool> touchedNewState(newStateSize, false);

        for (TrackingProximity& proximityObj: proximity)
        {
            if (!touchedState[proximityObj.objIdx1] && !touchedNewState[proximityObj.objIdx2])
            {
                touchedState[proximityObj.objIdx1] = true;
                touchedNewState[proximityObj.objIdx2] = true;

                TrackingObject newObj = newState[proximityObj.objIdx2];
                newObj.trackingObjIdx = state[proximityObj.objIdx1].trackingObjIdx;
                newObj.trackingColor = state[proximityObj.objIdx1].trackingColor;
                newObj.invisibleFactor = 0;

                state[proximityObj.objIdx1] = newObj;
            }
        }

        for (int i = 0; i < stateSize; ++i)
        {
            if (!touchedState[i])
            {
                ++(state[i].invisibleFactor);
            }
        }

        for (int i = 0; i < newStateSize; ++i)
        {
            if (!touchedNewState[i])
            {
                TrackingObject newObj = newState[i];
                newObj.trackingObjIdx = (nextObjIdx++);
                newObj.invisibleFactor = 0;

                if (colorStack.size() > 0)
                {
                    newObj.trackingColor = colorStack[colorStack.size() - 1];
                    colorStack.pop_back();
                }
                else
                {
                    newObj.trackingColor = baseColors[(nextColorIdx++) % baseColors.size()];
                }

                state.push_back(newObj);
            }
        }

        sortAndReduce();
    }
};

#endif /* TRACKING_ENG_H_ */
