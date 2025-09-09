#pragma once

#include "query.h"
#include "graph.h"
#include "output_file.h"
#include <thread>
#include "visualization.h"
#include <mutex>
#include "../utils/profiler.h" // if you have a timing profiler class
// #define GRAPH_PATH "C:\\Users\\Eslam\\Desktop\\temp\\map_routing\\maps\\sample\\map"
// #define QUERY_PATH "C:\\Users\\Eslam\\Desktop\\temp\\map_routing\\queries\\sample\\queries"
// #define OUTPUT_EXPECTED_PATH "C:\\Users\\Eslam\\Desktop\\temp\\map_routing\\outputs\\expected\\sample\\output"
#define GRAPH_PATH "/home/ismail/Projects/Map_Routing/maps/sample/map"
#define QUERY_PATH "/home/ismail/Projects/Map_Routing/queries/sample/queries"
#define OUTPUT_EXPECTED_PATH "/home/ismail/Projects/Map_Routing/outputs/expected/sample/output"
const int num_threads = thread::hardware_concurrency();
class runTests {
public:
    void SimpleTest();
    void MediumTest();
    void hardTest();
    void bonusTest();

    bool checkCorrectness(output_expected outputs_file, output_expected outputs_file_expected);
};

void ExcuteProblem(int hardniessLevelSelection);
