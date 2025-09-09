#ifndef OUTPUT_FILE_H
#define OUTPUT_FILE_H

#include <vector>
#include <string>
#include "query.h"

struct output_query {
    std::vector<int> path;
    double total_time;
    double walking_distance;
    double vehicle_distance;
    double total_distacne;

    output_query();
    output_query(std::vector<int> path, double total_time, double walking_distance, double vehicle_distance, double total_distacne);
    bool operator==(const output_query& other) const;
};

struct output_expected {
    std::vector<output_query> outputs;
    double total_time_witout_IO;
    double total_time_with_IO;

    output_expected();
    bool operator==(const output_expected& other) const;
};

// Read and write functions
output_expected readoutputexpected(std::string filename);
void writeoutput(output_expected out, std::string filename);
void write_output_file(const std::string& filename, const std::vector<output_query>& queries, const std::vector<output_query>& results);

#endif // OUTPUT_FILE_H
