#include "graph.h"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <queue>
#include <unordered_set>
#include <functional>
#include <climits>
#include <atomic>
#include <thread>
#include <mutex>
#include <vector>
#include <queue>
#include <limits>
#include "visualization.h"

intersection::intersection() {}
intersection::intersection(int id, double x, double y) :id(id), x(x), y(y) {}

edge::edge(int d_id, double length, double speed)
    : d_id(d_id), length(length), speed(speed), time(length / speed) {}

edge::edge(int d_id, double length, vector<double> speed)
    : d_id(d_id), length(length), speeds(speed) {}

edge::edge(int d_id, double time) : d_id(d_id), length(0), speed(0), time(time) {}

graph::graph(int s_id, int d_id, double length, double speed_inter, vector<double> speeds)
    : s_id(s_id), d_id(d_id), length(length), speed_interval(speed_inter), speeds(speeds)
{
}
void graph::load_bonus_grpah(const string& filename) {
    ifstream my_file(filename);
    if (!my_file.is_open()) {
        cout << "File graph not found!" << endl;
    }
    int numof_intersections;
    my_file >> numof_intersections;
    roads.resize(numof_intersections + 1);

    for (int i = 0; i < numof_intersections; ++i) {
        int id;
        double x, y;
        my_file >> id >> x >> y;
        roads[id].first = intersection(id, x, y);
        j["vertices"].push_back(roads[id].first);
    }
    vector< graph> xx;
    int numof_edges, count;
    double time_interva;
    my_file >> numof_edges >> count >> time_interva;
    speed_interval = time_interva;
    for (int i = 0; i < numof_edges; ++i) {
        int s_id, d_id;
        double length;
        my_file >> s_id >> d_id >> length;
        vector<double>c;
        for (int i = 0;i < count;i++) {
            double x;
            my_file >> x;
            c.push_back(x);
        }
        edge road1(d_id, length, c);
        edge road2(s_id, length, c);
        road1.s_id = s_id;
        this->roads[s_id].second.push_back(road1);
        this->roads[d_id].second.push_back(road2);
        j["edges"].push_back(road1);
        graph x = {
      this->s_id = s_id,
       this->d_id = d_id,
       this->length = length,
       this->speed = speed
        };
        xx.push_back(x);
    }
    my_file.close();
}


graph::graph() {}
graph::graph(int s_id, int d_id, double length, double speed)
    : s_id(s_id), d_id(d_id), length(length), speed(speed) {}

void graph::loadgraph(const string &filename)
{

    ifstream my_file(filename);
    if (!my_file.is_open())
    {
        cout << "File graph not found!" << endl;
    }

    int numof_intersections;
    my_file >> numof_intersections;
    roads.resize(numof_intersections);
    for (int i = 0; i < numof_intersections; ++i) // O(V)
    {
        int id;
        double x, y;
        my_file >> id >> x >> y;
        roads[id].first = intersection(id, x, y);
        j["vertices"].push_back(roads[id].first);

    }
    vector< graph> xx;

    int numof_edges;
    my_file >> numof_edges;
    for (int i = 0; i < numof_edges; ++i) // O(E)
    {
        int s_id, d_id;
        double length, speed;
        my_file >> s_id >> d_id >> length >> speed;
        edge road1(d_id, length, speed);
        road1.s_id = s_id;
        edge road2(s_id, length, speed);
        this->roads[s_id].second.push_back(road1);
        j["edges"] .push_back(road1);
        this->roads[d_id].second.push_back(road2);
    }   
    //save_roads(roads);
    //save_edges(xx);

    my_file.close();
}

double calculate_Euclideandistance(double x1, double x2, double y1, double y2)
{
    double x_distance = x1 - x2;
    double y_distance = y1 - y2;
    return x_distance * x_distance + y_distance * y_distance;
}

NodeSearchSource searchForSrc(
    double Qx, double Qy, double R,
    vector<pair<intersection, vector<edge>>> &roads,
    double Dx, double Dy)
{
    Profiler timer("src");
    int N = roads.size();
    priority_queue<edge_q> q;
    vector<pair<int,double>> destinations_time;
    vector<bool> vis;
    vector<double> time;
    vector<int> parents;
    vector<double> edge_length_to;
    time.reserve(N);
    parents.reserve(N);
    edge_length_to.reserve(N);

    R /= 1000;
    R*=R;
    for (int id = 0; id < roads.size(); ++id) 
    {
        /*
            Best case: EXACT O (V)
            Worst Case: Exact O(V log S) where S is starting nodes.
        */
        const auto &[intersection_, edges] = roads[id];
        double distance_src = calculate_Euclideandistance(intersection_.x, Qx, intersection_.y, Qy);
        edge_length_to.push_back(0);
        if (distance_src <= R)
        {
            double time_src = sqrt(distance_src) / WALKING_SPEED;
            q.push(edge_q(id,time_src));
            time.push_back(time_src);
            parents.push_back(-1);
            continue;
        }

        double distance_dst = calculate_Euclideandistance(intersection_.x, Dx, intersection_.y, Dy);

        if (distance_dst <= R)
        {
            double time_dst = sqrt(distance_dst) / WALKING_SPEED;
            destinations_time.push_back({ id, time_dst });
        }

        time.push_back(INT_MAX);
        parents.push_back(-2);

    }
    NodeSearchSource search;
    search.parents = parents;
    search.q = q;
    search.time = time;
    search.vis = vis;
    search.destinations_time = destinations_time;
    search.edge_length_to = edge_length_to;
    search.timer = timer.stop();
    return search;
}

Result dijkstra(
    vector<bool> &vis,
    vector<double> &time,
    vector<int> &parents,
    priority_queue<edge_q> &q,
    vector<pair<intersection, vector<edge>>> &roads,
    vector<pair<int,double>> &destinations_time,
    vector<double>& edge_length_to
)
{
    /*
        E log V + D + 2P
    // Total: O((V + E) log V + D + P)

    */

    Profiler tim("timer");
    while (!q.empty())
    {
        edge_q minEdge = q.top();
        int minIdx = minEdge.to;
        double ti = minEdge.weight;
        q.pop();
        if (ti > time[minIdx])
            continue;
        // relaxation
        for (auto &e : roads[minIdx].second)
        {
            int neighbour = e.d_id;
            double t = e.time;
            if (time[neighbour] > time[minIdx] + t)
            {
                time[neighbour] = time[minIdx] + t;
                parents[neighbour] = minIdx;
                edge_length_to[neighbour] = e.length;
                q.push(edge_q(neighbour, time[neighbour]));
            }
        }
    }
    // E log V
    double minTime = 1e9;
    int minDestination = -1;
    double destination_walking_time = -1;

    for(size_t i = 0; i < destinations_time.size(); ++i){ // EXACT O(D)
        auto& [id,dst_time] = destinations_time[i];
        double total_time = dst_time + time[id];
        if(total_time < minTime){
            minTime = total_time;
            minDestination = id;
            destination_walking_time = dst_time;
        }
    }

    vector<int> path;
    int current = minDestination;
    double totalDistance = 0;
    while (current != -1) // EXACT O(P)
    {
        path.push_back(current);
        int parent = parents[current];
        if (parent == -1)
            break;

        totalDistance+= edge_length_to[current];
        current = parent;
    }

    reverse(path.begin(), path.end()); // EXACT O(P)
    Result res;
    res.path = path;
    res.veichle_distance = totalDistance;
    res.veichle_time = minTime - destination_walking_time - time[path[0]];
    res.walking_time = destination_walking_time + time[path[0]] ;
    res.timer = tim.stop();
    return res;
}


NodeSearchSource searchForNodesBI(
    double Qx, double Qy, double R,
    vector<pair<intersection, vector<edge>>> &roads,
    double Dx, double Dy)
{
    const double INF = INT_MAX;


    int N = roads.size();
    vector<double> timeFwd;
    vector<double> timeBwd;
    vector<bool> visFwd;
    vector<bool> visBwd;

    vector<pair<int,double>> destinations_time_fwd;
    vector<pair<int,double>> destinations_time_bwd;
    priority_queue<edge_q> fwd;
    priority_queue<edge_q> bwd;

    timeFwd.reserve(N);
    timeBwd.reserve(N);
    visFwd.reserve(N);
    visBwd.reserve(N);
    R /= 1000;

    for (int id = 0; id < roads.size(); ++id)
    {
        const auto &[intersection_, edges] = roads[id];
        double distance_src = calculate_Euclideandistance(intersection_.x, Qx, intersection_.y, Qy);
        double distance_dst = calculate_Euclideandistance(intersection_.x, Dx, intersection_.y, Dy);
        visFwd.push_back(0);
        visBwd.push_back(0);
        if (distance_src <= R)
        {
            double time_src = distance_src / WALKING_SPEED;
            // walk_time_dst_src[id] = {time_src, distance_src};
            roads[N - 2].second.push_back(edge(id, time_src));
            roads[id].second.push_back(edge(N - 2, time_src));
        }

        if (distance_dst <= R)
        {
            double time_dst = distance_dst / WALKING_SPEED;
            // walk_time_dist_dst[id] = {time_dst, distance_dst};
            roads[N - 1].second.push_back(edge(id, time_dst));
            roads[id].second.push_back(edge(N - 1, time_dst));
        }

        timeFwd.push_back(INF);
        timeBwd.push_back(INF);

    }
    visFwd.push_back(0);
    visFwd.push_back(0);
    visBwd.push_back(0);
    visBwd.push_back(0);

    timeFwd.push_back(0); // N-2
    timeFwd.push_back(INF); // N - 1
    timeBwd.push_back(INF);
    timeBwd.push_back(0); // N - 1 
    NodeSearchSource search;
    // search.walk_time_dist_src = walk_time_dst_src;
    // search.walk_time_dist_dst = walk_time_dist_dst;
    search.timeBwd = timeBwd;
    search.timeFwd = timeFwd;
    search.visBwd = visBwd;
    search.visFwd = visFwd;
    return search;
}

double bidirectional_dijkstra(
    std::vector<std::pair<intersection, std::vector<edge>>> &roads,
    vector<double>& timeFwd,
    vector<double>& timeBwd,
    vector<bool>& visFwd,
    vector<bool> &visBwd
)
{
    const int N = roads.size();
    const int virtual_destination = N - 1;
    const int virtual_source = N - 2;
    const double INF = std::numeric_limits<double>::infinity();
    priority_queue<edge_q> pqFwd, pqBwd;

    // Initialize
    pqFwd.push({virtual_source, 0});
    pqBwd.push({virtual_destination, 0});

    double best_cost = INF;
    bool forward_step = true;  

    while (!pqFwd.empty() && !pqBwd.empty()) {


        // Alternate processing direction each iteration
        if (forward_step) {
            // Process FORWARD search
            auto [u, t] = pqFwd.top();
            pqFwd.pop();

            if (!visFwd[u]) {
                visFwd[u] = true;
                // Check if backward search already saw this node
                if (visBwd[u]) {
                    best_cost = std::min(best_cost, timeFwd[u] + timeBwd[u]);
                    break;
                }
                // Expand forward neighbors
                for (const auto &e : roads[u].second) {
                    const int v = e.d_id;
                    const double new_time = timeFwd[u] + e.time;
                    if (new_time < timeFwd[v]) {
                        timeFwd[v] = new_time;
                        pqFwd.push({v, new_time});
                    }
                }
            }
        } else {
            // Process BACKWARD search
            auto [u, t] = pqBwd.top();
            pqBwd.pop();

            if (!visBwd[u]) {
                visBwd[u] = true;
                // Check if forward search already saw this node
                if (visFwd[u]) {
                    best_cost = std::min(best_cost, timeFwd[u] + timeBwd[u]);
                    break;
                }
                // Expand backward neighbors (same as forward in undirected graph)
                for (const auto &e : roads[u].second) {
                    const int v = e.d_id;
                    const double new_time = timeBwd[u] + e.time;
                    if (new_time < timeBwd[v]) {
                        timeBwd[v] = new_time;
                        pqBwd.push({v, new_time});
                    }
                }
            }
        }
        forward_step = !forward_step;  // Toggle direction
    }

    while (!pqFwd.empty()){
        int node_num = pqFwd.top().to;
        pqFwd.pop();
        // 0 src 1 dest
        // This path can't be shorter because both distances are longer
        if (!visBwd[node_num]){
            continue;
        }
        // No need to evaluate rest of queue cuz it won't produce a shorter path
        // Assuming all positive edge weights 
        if (timeFwd[node_num] > best_cost){
            break;
        }
        if (timeBwd[node_num] != INF){
            if (timeFwd[node_num] + timeBwd[node_num] < best_cost){
                best_cost = timeFwd[node_num] + timeBwd[node_num];
            }
        }
    }
    while (!pqBwd.empty()){
        int node_num = pqBwd.top().to;
        pqBwd.pop();
        // This path can't be shorter because both distances are longer
        if (!visFwd[node_num]){
            continue;
        }
        // No need to evaluate rest of queue cuz it won't produce a shorter path
        // Assuming all positive edge weights 
        if (timeBwd[node_num] > best_cost){
            break;
        }
        if (timeFwd[node_num] != INF){
            if (timeFwd[node_num] + timeBwd[node_num] < best_cost){
                best_cost = timeFwd[node_num] + timeBwd[node_num] ;
            }
        }
    }


    if (best_cost == INF) {
        std::cout << "ABORTING: No Possible path" << std::endl;
        exit(1);
    }
    return best_cost * 60;  // Convert hours to minutes
}

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
)
{

    path.push_back(current);
    visited[current] = true;

    if (current == target) {
        if (currentTime < bestTotalTime) {
            bestTotalTime = currentTime;
            totalDistance = currentDistance;
            bestPath = path;
        }
    }
    else {

        for (const auto& edge : roads[current].second) {
            int neighbor = edge.d_id;

            if (visited[neighbor])
                continue;

            int period = min((int)(currentTime / speed_interval),
                (int)(edge.speeds.size()) - 1);
            double speed = edge.speeds[period];
            double travelTime = (edge.length / speed) * 60;

            dfsHelperFunction(neighbor, target, speed_interval, roads, visited, path,
                currentTime + travelTime, currentDistance + edge.length,
                bestPath, bestTotalTime, totalDistance);
        }
    }

    visited[current] = false;
    path.pop_back();
}

Result dfs(
    vector<double>& time,
    vector<pair<intersection, vector<edge>>>& roads,
    double speed_interval,
    double Dx,
    double Dy
)
{

    double target;
    int source;
    for (int i = 0;i < roads.size();++i) {
        if (roads[i].first.x == Dx && roads[i].first.y == Dy) {
            target = roads[i].first.id;
        }
    }

    double temp = INT_MAX;
    // o(v)
    for (int i = 0; i < time.size(); i++) {
        if (time[i] < temp) {
            source = i;
            temp = time[i];
        }
    }

    vector<int> bestPath;
    double bestTotalTime = numeric_limits<double>::max();
    double totalDistance = 0;

    vector<bool> visited(roads.size(), false);
    vector<int> currentPath;

    dfsHelperFunction(source, target, speed_interval, roads, visited, currentPath, 0.0, 0.0,
        bestPath, bestTotalTime, totalDistance);

    Result result;
    result.path = bestPath;
    result.veichle_time = bestTotalTime;
    result.veichle_distance = totalDistance;

    return result;
}
NodeSearchSource searchForSrc_bonus(
    double Qx, double Qy, double R,
    vector<pair<intersection, vector<edge>>>& roads,
    double Dx, double Dy
) {
    priority_queue<edge_q> q;
    unordered_map<int, pair<double, double>> walk_time_dist_src;
    unordered_map<int, pair<double, double>> walk_time_dist_dst;
    vector<bool> vis(roads.size() + 2, 0);
    vector<double> time(roads.size() + 2, INT_MAX);
    vector<int> parents(roads.size() + 2, 0);
    R /= 1000;  // Convert to km

    for (int id = 0; id < roads.size() - 2; ++id) {
        const auto& [intersection_, edges] = roads[id];
        double distance_src = calculate_Euclideandistance(intersection_.x, Qx, intersection_.y, Qy);
        double distance_dst = calculate_Euclideandistance(intersection_.x, Dx, intersection_.y, Dy);

        if (distance_src <= R) {
            double time_src = distance_src / WALKING_SPEED;
            walk_time_dist_src[id] = { time_src, distance_src };
            time[id] = time_src;
            parents[id] = -1;
            q.push(edge_q(id, time_src));
        }

        if (distance_dst <= R) {
            double time_dst = distance_dst / WALKING_SPEED;
            walk_time_dist_dst[id] = { time_dst, distance_dst };
            roads[roads.size() - 1].second.push_back(edge(id, time_dst));
            roads[id].second.push_back(edge(roads.size() - 1, time_dst));
        }
    }

    NodeSearchSource search;
    search.parents = parents;
    search.q = q;
    search.time = time;
    search.vis = vis;
    search.walk_time_dist_src = walk_time_dist_src;
    search.walk_time_dist_dst = walk_time_dist_dst;
    return search;
}

