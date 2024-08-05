#pragma once
#include "MapManager.h"

class Navigation {
public:

    vector<vector<int>> WallPoints;

    vector<Vector> WallPositions;

    string MapFileName;

    MapManager map = MapManager();

    Navigation(string filename);

    void loadMap();

    void savePicture(vector<vector<int>> pixels, vector<int> colour, int width, int height, string path);

    void savePicture(vector<vector<float>> pixels, vector<int> colour, int width, int height, string path);

    void savePicture(vector<vector<double>> pixels, vector<int> colour, int width, int height, string path);

    void savePicture(vector<vector<bool>> pixels, vector<int> colour, int width, int height, string path);

    void saveColourPicture(vector<vector<vector<int>>> pixels, int width, int height, string path);
    
    void setWallPositions(vector<Vector> WallPositions);
    
    vector<vector<int>> getWallPoints();

    vector<vector<int>> findRoute(Vector start, Vector target, float resolution);

    vector<vector<int>> createImageOfMap(string filename, Vector Position, vector<int> colour, int width, int height);

    vector<vector<vector<int>>> combineImages(vector<vector<int>> image1, vector<int> colour1, vector<vector<int>> image2, vector<int> colour2);

};
