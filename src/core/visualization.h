#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include "json.hpp"
#include <cstdlib>

// Forward declarations for types
struct intersection;
class graph;
struct Result;
class query;
struct edge;

using json = nlohmann::json;

// Forward declarations for JSON serialization
void to_json(json& h, const intersection& v);
void to_json(json& h, const graph& e);
void to_json(json& h, const Result& o);
void to_json(json& h, const query& q);
void to_json(json& h, const edge& e);
void saveToJson(const std::string& filename, json h);

class visualization
{
public:
    visualization() = default;
    ~visualization() = default;
    
    // Member variable for JSON data
    json j;
    
    // Method to create and save map visualization
    // If roads_path is provided, loads the graph from that file
    // If res is provided, uses that result, otherwise computes a new one
    void My_map(query q, Result* res = nullptr, std::string roads_path = "lol");
};

