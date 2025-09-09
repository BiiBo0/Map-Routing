#pragma once

#include <vector>
#include <string>

class query {
public:
    double x_source, y_source, x_dest, y_dest;
    double R;

    // Constructors
    query();
    query(double x1, double y1, double x2, double y2, double r);

    // Static method to load queries from file
    static std::vector<query> loadQueriesFromFile(const std::string& filename);
};
