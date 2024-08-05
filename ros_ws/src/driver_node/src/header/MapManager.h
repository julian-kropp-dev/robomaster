#pragma once
#include "Transformations.h"
#include <limits>
#include <fstream>
#include <sstream>
#include <sys/stat.h> 

class MapManager {
public:

    vector<Vector> WallPositions;

    vector<vector<int>> WallPoints;

    Vector WallPointsOffset;

    MapManager() {};

    MapManager(string filename);

    MapManager(vector<Vector> wallPositions);

    void createWallPoints();
    
    void createWallPoints(float resolution);
    
    void setWallPositions(vector<Vector> WallPositions);

    void filterMap(float upperBorder, float lowerBorder);

    void saveMap(string filename);

    bool loadMap(string filename);

    vector<vector<int>> drawMap(Vector Position, int width, int height);

};
