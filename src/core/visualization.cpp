#include "visualization.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include "json.hpp"
#include <cstdlib> 
#include "graph.h"
#include "query.h"

using namespace std;
//using json = nlohmann::json;
void to_json(json& h, const intersection& v) {
    h = json{ {"id", v.id}, {"x", v.x}, {"y", v.y} };
}
//void to_json(json& h, const graph& e) {
//    h = json{ {"src", e.s_id}, {"dest", e.d_id}, {"distance", e.length}, {"speed", e.speed} };
//}
void to_json(json& h, const query& q) {
    h = json{
    {"start_x", q.x_source},
    {"start_y", q.y_source},
    {"end_x", q.x_dest},
    {"end_y", q.y_dest}
    };
}
void to_json(json& h, const Result& o) {
    h = json{
        {"path", o.path}
    };
}
void to_json(json& h, const edge& e) {
    h = json{
        {"src", e.s_id}, {"dest", e.d_id}, {"distance", e.length}, {"speed", e.speed}
    };
}
//json j;
//void save_roads(vector<pair<intersection, vector<edge>>> roads)
//{
//
//    for (const auto& road : roads) {
//        j["vertices"].push_back(road.first);
//    }
//
//  /*  for (const auto& road : roads) {
//        j["edges"].push_back(road.second);
//    }*/
//}
//void save_edges(vector<graph> g)
//{
//    j["edges"] = g;
//}
//void save_q(const query& q)
//{
//    j["query"] = q;
//}
//void save_Rs(const Result& result)
//{
//    j["result"] = result;
//}
void saveToJson(const string& filename,json h)
    {
    ofstream outfile(filename);
    if (!outfile) {
        cerr << "Failed to open output file: " << filename << endl;
        exit(1);
    }

    outfile << setw(4) << h << endl;
    cout << "Data and result saved to " << filename << endl;
}
//void  visualization::My_map(query q, Result* res , string roads_path)
//{
//    graph road_network;
//    if (roads_path != "lol")
//    { 
//        road_network.loadgraph(roads_path + ".txt");
//    }
//    //save_q(q);
//    if (res == nullptr) {
//        NodeSearchSource source_res = searchForSrc(q.x_source, q.y_source, q.R, road_network.roads, q.x_dest, q.y_dest);
//
//        Result result = dijkstra(
//            source_res.vis,
//            source_res.time,
//            source_res.parents,
//            source_res.q,
//            road_network.roads,
//            source_res.destinations_time,
//            source_res.edge_length_to
//        );
//        //save_Rs(result);
//    }
//    else
//    {
//        //save_Rs(*res);
//
//    }
//    //saveToJson("suiiii.json");
//}