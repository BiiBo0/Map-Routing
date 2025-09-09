#ifndef GRAPH_H
#define GRAPH_H

#include "output_file.h"
#include "../utils/profiler.h"
#include <vector>
#include <string>
#include <utility>
#include <unordered_map>
#include <queue>
#include <limits>
#include <cmath>
#include <unordered_set>
#include "visualization.h"
using namespace std;


struct edge_q{
    int to;
    double weight;
    edge_q(int to ,double w) :
    to(to), weight(w){}
    bool operator <(const edge_q&e) const{
        return weight > e.weight;
    }

};

struct intersection {
    int id;
    double x, y;
    intersection();
    intersection(int id ,double x, double y);
};

struct edge {
    int s_id;
    int d_id; /// to
    double length, speed;
    double time; // w
    vector<double>speeds;
    edge(int d_id, double length, double speed);
    edge(int d_id, double length, vector<double>speeds);

    edge(int d_id, double time);

    bool operator ==(const edge&e) const{
        return d_id == e.d_id;
    }
};

struct Result {
    vector<int> path;
    double veichle_distance;
    double veichle_time;
    double walking_time;
    double timer;
};

struct NodeSearchDest {
    std::unordered_set<int> nodes;
    std::unordered_map<int, std::pair<double, double>> walk_time_dist;

    NodeSearchDest(
        std::unordered_set<int> nodes_,
        std::unordered_map<int, std::pair<double, double>> walk_time_dist_
    ) : nodes(nodes_), walk_time_dist(walk_time_dist_) {}
};

const double WALKING_SPEED = 5.0;

class graph {
public:
    json j;
    vector<pair<intersection,vector<edge>>> roads;
    int s_id,d_id;
    double length,speed;
    graph(int s_id, int d_id, double length, double speed);
    graph();

    void loadgraph(const string& filename);
    vector<double> speeds;
    double speed_interval;
    graph(int s_id, int d_id, double length, double speed_interval,vector<double> speeds);
    void load_bonus_grpah(const string& filname);
    double speed_count;
};

double calculate_Euclideandistance(double x1, double x2, double y1, double y2);

struct NodeSearchSource {
    //for Bidirectional LEAVE IT
    vector<double> timeFwd;
    vector<double> timeBwd;
    vector<bool> visFwd;
    vector<bool> visBwd;

    priority_queue<edge_q> q;
    vector<pair<int,double>> destinations_time;
    vector<bool> vis;
    vector<double> time;
    vector<int> parents;
    vector<double> edge_length_to;


    unordered_set<int> valid_nodes_src;
    unordered_set<int> valid_nodes_dst;
    unordered_map<int, pair<double, double>> walk_time_dist_src;
    unordered_map<int, pair<double, double>> walk_time_dist_dst;

    double timer;
};


NodeSearchSource searchForSrc(
    double Qx, double Qy, double R,
    vector<pair<intersection,vector<edge>>> &roads,
    double Dx, double Dy
);

NodeSearchDest searchForDestinations(
    std::unordered_map<int, intersection>& vertices,
    std::unordered_map<int, std::vector<edge>> &roads,
    double Qx, double Qy, double R
);
        


Result dijkstra(
    vector<bool> &vis,
    vector<double> &time,
    vector<int> &parents,
    priority_queue<edge_q> &q,
    vector<pair<intersection, vector<edge>>> &roads,
    vector<pair<int,double>> &destinations_time,
    vector<double>& edge_length_to
);

double bidirectional_dijkstra(
    vector<pair<intersection, std::vector<edge>>> &roads,
    vector<double>& timeFwd,
    vector<double>& timeBwd,
    vector<bool>& visFwd,
    vector<bool> &visBwd
);

NodeSearchSource searchForNodesBI(
    double Qx, double Qy, double R,
    vector<pair<intersection,vector<edge>>> &roads,
    double Dx, double Dy
);

void dfsHelperFunction(int current,
    int target,
    double speed_interval,
    vector<pair<intersection, vector<edge>>>& roads,
    vector<bool>& visited,
    vector<int>& path,
    double currentTime,
    double currentDistance,
    vector<int>& bestPath,
    double& bestTotalTime,
    double& totalDistance
);

Result dfs(vector<double>& time,
    vector<pair<intersection, vector<edge>>>& roads,
    double speed_interval,
    double Dx,
    double Dy
);


NodeSearchSource searchForSrc_bonus(
    double Qx, double Qy, double R,
    vector<pair<intersection, vector<edge>>>& roads,
    double Dx, double Dy
);



#endif // GRAPH_H
