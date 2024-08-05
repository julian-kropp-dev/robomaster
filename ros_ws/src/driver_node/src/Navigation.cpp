#include "header/Navigation.h"

Navigation::Navigation(string filename) {
    this->MapFileName = filename;
}

void Navigation::loadMap() {
#ifndef DEBUG
    map = MapManager(MapFileName);
#endif // !DEBUG
    WallPositions = map.WallPositions;
    WallPoints = map.WallPoints;
    cout << "size: " << WallPositions.size() << endl;
}

void Navigation::savePicture(vector<vector<int>> pixels, vector<int> colour, int width, int height, string path) {
    ofstream outfile(path);
    outfile << "P3" << endl;
    outfile << to_string(width) << " " << to_string(height) << endl;
    outfile << "255" << endl;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            if (pixels[x][y] == 1) {
                outfile << to_string(colour[0]) << " " << to_string(colour[1]) << " " << to_string(colour[2]) << " ";
            }
            else {
                outfile << "0 0 0 ";
            }
        }
        outfile << endl;
    }

    outfile.close();
}

void Navigation::savePicture(vector<vector<float>> pixels, vector<int> colour, int width, int height, string path) {
    ofstream outfile(path);
    outfile << "P3" << endl;
    outfile << to_string(width) << " " << to_string(height) << endl;
    outfile << "255" << endl;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            if (pixels[x][y] > 0.0f) {
                outfile << to_string((int)((float)colour[0] * min(pixels[x][y], 1.0f))) << " " << to_string((int)((float)colour[1] * min(pixels[x][y], 1.0f))) << " " << to_string((int)((float)colour[2] * min(pixels[x][y], 1.0f))) << " ";
            }
            else {
                outfile << "0 0 0 ";
            }
        }
        outfile << endl;
    }

    outfile.close();
}

void Navigation::savePicture(vector<vector<double>> pixels, vector<int> colour, int width, int height, string path) {
    ofstream outfile(path);
    outfile << "P3" << endl;
    outfile << to_string(width) << " " << to_string(height) << endl;
    outfile << "255" << endl;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            if (pixels[x][y] > 0.0f) {
                outfile << to_string((int)((double)colour[0] * min(pixels[x][y], 1.0))) << " " << to_string((int)((float)colour[1] * min(pixels[x][y], 1.0))) << " " << to_string((int)((float)colour[2] * min(pixels[x][y], 1.0))) << " ";
            }
            else {
                outfile << "0 0 0 ";
            }
        }
        outfile << endl;
    }

    outfile.close();
}

void Navigation::savePicture(vector<vector<bool>> pixels, vector<int> colour, int width, int height, string path) {
    ofstream outfile(path);
    outfile << "P3" << endl;
    outfile << to_string(width) << " " << to_string(height) << endl;
    outfile << "255" << endl;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            if (pixels[x][y] == 1) {
                outfile << to_string(colour[0]) << " " << to_string(colour[1]) << " " << to_string(colour[2]) << " ";
            }
            else {
                outfile << "0 0 0 ";
            }
        }
        outfile << endl;
    }

    outfile.close();
}

void Navigation::saveColourPicture(vector<vector<vector<int>>> pixels, int width, int height, string path) {
    ofstream outfile(path);
    outfile << "P3" << endl;
    outfile << to_string(width) << " " << to_string(height) << endl;
    outfile << "255" << endl;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            outfile << to_string(pixels[x][y][0]) << " " << to_string(pixels[x][y][1]) << " " << to_string(pixels[x][y][2]) << " ";
        }
        outfile << endl;
    }

    outfile.close();
}

vector<vector<int>> Navigation::createImageOfMap(string filename, Vector Position, vector<int> colour, int width, int height) {
    vector<vector<int>> Edges = map.drawMap(Position, width, height);
    savePicture(Edges, colour, width, height, filename);
    return Edges;
}

vector<vector<vector<int>>> Navigation::combineImages(vector<vector<int>> image1, vector<int> colour1, vector<vector<int>> image2, vector<int> colour2) {
    vector<vector<vector<int>>> pixels = vector<vector<vector<int>>>(int(image1.size()), vector<vector<int>>(int(image2.size()), vector<int>(3, 0)));
    for (int x = 0; x < image1.size(); x++) {
        for (int y = 0; y < image1[0].size(); y++) {
            if (image1[x][y] == 1) {
                pixels[x][y] = colour1;
            }
            if (image2[x][y] == 1) {
                pixels[x][y][0] = min(pixels[x][y][0] + colour2[0], 255);
                pixels[x][y][1] = min(pixels[x][y][1] + colour2[1], 255);
                pixels[x][y][2] = min(pixels[x][y][2] + colour2[2], 255);
            }
        }
    }
    return pixels;
}

struct Node {
    int x;
    int y;
    int PathLength;
    int bestNode;

    Node(int PosX, int PosY, float length, int best) {
        x = PosX;
        y = PosY;
        PathLength = length;
        bestNode = best;
    }

    Node() {
        x = 0;
        y = 0;
        PathLength = 0;
        bestNode = 0;
    }

};

struct PathNode {
    int x;
    int y;
    int PathLength;
    int followingNode;
    int lastNode;

    PathNode(int PosX, int PosY, float length, int best, int lastbest) {
        x = PosX;
        y = PosY;
        PathLength = length;
        followingNode = best;
        lastNode = lastbest;
    }

    PathNode() {
        x = 0;
        y = 0;
        PathLength = 0;
        followingNode = 0;
        lastNode = 0;
    }

};

void Navigation::setWallPositions(vector<Vector> WallPositions) {
    this->map.setWallPositions(WallPositions);
}

vector<vector<int>> Navigation::getWallPoints() {
    return this->map.WallPoints;
}


// Simple implementation of the A*-algorithm
vector<vector<int>> Navigation::findRoute(Vector start, Vector target, float resolution)
{
    vector<PathNode> pathnodes(1);
    vector<Node> nodes(0);
    float PathLength = 0.0f;
    int PathID = 0;
    bool reachedTarget = false;

    map.WallPositions.push_back(start);
    map.WallPositions.push_back(target);

    map.createWallPoints(resolution);
    WallPoints = map.WallPoints;
    start.x = start.x*resolution - map.WallPointsOffset.x;
    start.y = start.y*resolution - map.WallPointsOffset.y;
    target.x = target.x*resolution - map.WallPointsOffset.x;
    target.y = target.y*resolution - map.WallPointsOffset.y;

    pathnodes[0].x = start.x;
    pathnodes[0].y = start.y;
    pathnodes[0].followingNode = -1;

    vector<vector<int>> walls2d = vector<vector<int>>(WallPoints.size(), vector<int>(WallPoints[0].size(), 0));
    for (int x = 0; x < WallPoints.size(); x++)
    {
        for (int y = 0; y < WallPoints[0].size(); y++)
        {
            walls2d[x][y] = WallPoints[x][y];
        }
    }

    while (reachedTarget == false)
    {
        for (int x = -1; x <= 1; x++)
        {
            for (int y = -1; y <= 1; y++)
            {
                if ((pathnodes[PathID].x + x) >= 0 && (pathnodes[PathID].y + y) >= 0 && (pathnodes[PathID].x + x) < walls2d.size() && (pathnodes[PathID].y + y) < walls2d[0].size())
                {
                    if (x == 0 && y == 0)
                    {
                    }
                    else
                    {
                        if (walls2d[pathnodes[PathID].x + x][pathnodes[PathID].y + y] < 65)
                        {
                            nodes.push_back(Node(pathnodes[PathID].x + x, pathnodes[PathID].y + y,
                                PathLength + round(Vector(pathnodes[PathID].x - (pathnodes[PathID].x + x), pathnodes[PathID].y - (pathnodes[PathID].y + y)).magnitude()),
                                pathnodes[PathID].followingNode));
                        }
                    }
                }
            }
        }

        float bestf = 900000.0;
        int bestNode = -1;
        for (int x = 0; x < nodes.size(); x++)
        {
            float f = Vector(nodes[x].x - target.x, nodes[x].y - target.y).magnitude() + nodes[x].PathLength;
            if (f < bestf && walls2d[nodes[x].x][nodes[x].y] < 65)
            {
                bestf = f;
                bestNode = x;
            }
        }

        if(bestNode >= 0) {
            PathLength = nodes[bestNode].PathLength;
            PathID++;
            pathnodes.push_back(PathNode(nodes[bestNode].x, nodes[bestNode].y, nodes[bestNode].PathLength, bestNode, nodes[bestNode].bestNode));
            PathID = pathnodes.size() - 1;
            walls2d[nodes[bestNode].x][nodes[bestNode].y] = 100;
            nodes[bestNode].PathLength = 523323213;
            if (pathnodes[PathID].x == (int)round(target.x) && pathnodes[PathID].y == (int)round(target.y))
            {
                reachedTarget = true;
            }
        }
        else {
            break;
        }
    }

    if(reachedTarget) {
        vector<vector<int>> path = vector<vector<int>>(PathLength + 1, vector<int>(3, 0));
        reachedTarget = false;
        path[PathLength][0] = (pathnodes[PathID].x + map.WallPointsOffset.x)/resolution;
        path[PathLength][1] = (pathnodes[PathID].y + map.WallPointsOffset.y)/resolution;
        path[PathLength][2] = pathnodes[PathID].lastNode;

        for (int i = PathLength - 1; i >= 0; i--)
        {
            path[i][0] = (nodes[path[i + 1][2]].x + map.WallPointsOffset.x)/resolution;
            path[i][1] = (nodes[path[i + 1][2]].y + map.WallPointsOffset.y)/resolution;;
            path[i][2] = nodes[path[i + 1][2]].bestNode;
            if (path[i][2] == -1)
            {
                i--;
                path[i][0] = (start.x + map.WallPointsOffset.x)/resolution;
                path[i][1] = (start.y + map.WallPointsOffset.y)/resolution;
                reachedTarget = true;
            }
        }
        return path;
    }
    return vector<vector<int>>(0, vector<int>(3, 0));
}
