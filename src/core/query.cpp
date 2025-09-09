#include "query.h"

#include <fstream>
#include <iostream>
#include "../utils/profiler.h"
using namespace std;

query::query() = default;

query::query(double x1, double y1, double x2, double y2, double r) {
    x_source = x1;
    y_source = y1;
    x_dest = x2;
    y_dest = y2;
    R = r;
}

vector<query> query::loadQueriesFromFile(const string& filename) {
    Profiler timer("Loading queries");
    vector<query> queries;
    ifstream my_file(filename);

    if (!my_file.is_open()) {
        cout << "File query not found!" << endl;
        return queries;
    }

    int test;
    my_file >> test;

    for (int i = 0; i < test; i++) {
        double x1, y1, x2, y2;
        double r;
        my_file >> x1 >> y1 >> x2 >> y2 >> r;
        queries.emplace_back(x1, y1, x2, y2, r);
    }

    my_file.close();
    return queries;
}
