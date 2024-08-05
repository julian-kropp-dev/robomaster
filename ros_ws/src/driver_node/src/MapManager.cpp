#include "header/MapManager.h"

MapManager::MapManager(string filename) {
    if (loadMap(filename))
        createWallPoints();
}

MapManager::MapManager(vector<Vector> wallPositions) {
    this->WallPositions = wallPositions;

    createWallPoints();
}

void MapManager::createWallPoints() {

    Vector max = Vector(-std::numeric_limits<int>::max(), -std::numeric_limits<int>::max());
    Vector min = Vector(std::numeric_limits<int>::max(), std::numeric_limits<int>::max());
    for (int i = 0; i < WallPositions.size(); i++) {
        if (WallPositions[i].x > max.x)
            max.x = WallPositions[i].x;

        if (WallPositions[i].y > max.y)
            max.y = WallPositions[i].y;

        if (WallPositions[i].x < min.x)
            min.x = WallPositions[i].x;

        if (WallPositions[i].y < min.y)
            min.y = WallPositions[i].y;
    }
    WallPointsOffset = min;

    int sizeX = (int)round(max.x - min.x) + 2;
    int sizeY = (int)round(max.y - min.y) + 2;

    WallPoints = vector<vector<int>>(sizeX, vector<int>(sizeY, 0));
    for (int i = 0; i < WallPositions.size(); i++) {
        WallPoints[(int)round(WallPositions[i].x - WallPointsOffset.x)][(int)round(WallPositions[i].y - WallPointsOffset.y)] = 1;
    }
}

void MapManager::createWallPoints(float resolution) {
    createWallPoints();

    vector<vector<int>> oldWallPoints = WallPoints;
    int sizeX = (int)((float)oldWallPoints.size()*resolution);
    int sizeY = (int)((float)oldWallPoints[0].size()*resolution);
    
    WallPoints = vector<vector<int>>(sizeX, vector<int>(sizeY, 0));
    for (int x = 0; x < oldWallPoints.size(); x++) {
        for (int y = 0; y < oldWallPoints[0].size(); y++) {
            int newX = std::min((int)((float)x * resolution), sizeX-1);
            int newY = std::min((int)((float)y * resolution), sizeY-1);
            WallPoints[newX][newY] += oldWallPoints[x][y] * 10;
        }
    }
    
    WallPointsOffset.x = WallPointsOffset.x * resolution;
    WallPointsOffset.y = WallPointsOffset.y * resolution;
    WallPointsOffset.z = WallPointsOffset.z * resolution;
}

void MapManager::setWallPositions(vector<Vector> WallPositions) {
    this->WallPositions = WallPositions;
}

void MapManager::filterMap(float upperBorder, float lowerBorder) {
    for(int i = 0; i < WallPositions.size(); i++) {
        if(WallPositions[i].z < lowerBorder || WallPositions[i].z > upperBorder) {
            WallPositions.erase (WallPositions.begin()+i);
            i--;
        }
    }
}

vector<string> SplitString(string str, char seperator) {
    vector<string> split = vector<string>(0);
    istringstream iss(str);
    string s;
    while (getline(iss, s, seperator)) {
        split.push_back(s);
    }

    return split;
}

void MapManager::saveMap(string filename) {
#ifndef DEBUG

    ofstream outfile(filename);
    outfile << to_string(WallPositions.size()) << endl;

    for (int i = 0; i < WallPositions.size(); i++) {
        outfile << to_string(WallPositions[i].x) << " " << to_string(WallPositions[i].y) << " " << to_string(WallPositions[i].z) << endl;
    }

    outfile.close();
#endif // !DEBUG
}

bool MapManager::loadMap(string filename) {
#ifndef DEBUG
    WallPositions = vector<Vector>(0);

    struct stat buffer;
    if (stat(filename.c_str(), &buffer) == 0) {
        string str;
        ifstream ifs(filename, ifstream::in);

        getline(ifs, str);
        int WallPositionsSize = stoi(str);

        for (int i = 0; i < WallPositionsSize; i++) {
            getline(ifs, str);
            vector<string> xyz = SplitString(str, ' ');
            WallPositions.push_back(Vector(stof(xyz[0]), stof(xyz[1]), stof(xyz[2])));
        }
        cout << "Successfully loaded file!" << endl;
        return true;
    }
    else
    {
        saveMap(filename);
        cout << "Map file does not exist!" << endl;
        return false;
    }
#else
    return false;
#endif // !DEBUG
}

vector<vector<int>> MapManager::drawMap(Vector Position, int width, int height) {
    Vector center = Vector(width / 2.0f, height / 2.0f);
    vector<vector<int>> mapImage = vector<vector<int>>(width, vector<int>(height, 0));
    for (int i = 0; i < WallPositions.size(); i++) {
        Vector v = Vector(WallPositions[i].x - Position.x, WallPositions[i].y - Position.y);
        float mag = v.magnitude();
        v = v.normalize();
        v.rotate(-Position.z);
        v = v.mul(0.5f * mag);
        v = v.add(center);
        if (v.x > 0 && v.x < width && v.y > 0 && v.y < height)
            mapImage[width - int(v.x)][int(v.y)] = 1;
    }
    return mapImage;
}
