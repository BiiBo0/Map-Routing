#include "output_file.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <vector>
#include <algorithm>
#include <cmath>

#include <cctype>  // for isspace


using namespace std;

// --- output_query methods ---
output_query::output_query() {
    path = vector<int>();
    total_time = 0;
    walking_distance = 0;
    vehicle_distance = 0;
    total_distacne = 0;
}

output_query::output_query(vector<int> path, double total_time, double walking_distance, double vehicle_distance, double total_distacne) {
    this->path = path;
    this->total_time = total_time;
    this->walking_distance = walking_distance;
    this->vehicle_distance = vehicle_distance;
    this->total_distacne = total_distacne;
}

bool output_query::operator==(const output_query& expected) const {
    bool equal = true;
    const double EPS = 0.01;

     if (path != expected.path) {
         std::cout << "Path mismatch:\n";
         std::cout << "  expected: ";
         for (auto id : expected.path) std::cout << id << " ";
         std::cout << "\n  actual:   ";
         for (auto id : path) std::cout << id << " ";
         std::cout << "\n";
         equal = false;
     }

    if (std::abs(total_time - expected.total_time) > EPS) {
        std::cout << "Total Time mismatch:\n";
        std::cout << "  expected: " << expected.total_time << " actual: " << total_time << "\n";
        equal = false;
    }

     if (std::abs(walking_distance - expected.walking_distance) > EPS) {
         std::cout << "Walking Distance mismatch:\n";
         std::cout << "  expected: " << expected.walking_distance << " actual: " << walking_distance << "\n";
         equal = false;
     }

    if (std::abs(vehicle_distance - expected.vehicle_distance) > EPS) {
         std::cout << "Vehicle Distance mismatch:\n";
         std::cout << "  expected: " << expected.vehicle_distance << " actual: " << vehicle_distance << "\n";
          equal = false;
    }

    if (std::abs(total_distacne - expected.total_distacne) > EPS) {
         std::cout << "Total Distance mismatch:\n";
         std::cout << "  expected: " << expected.total_distacne << " actual: " << total_distacne << "\n";
         equal = false;
     }

    return equal;
}


output_expected::output_expected() {
    outputs = vector<output_query>();
    total_time_witout_IO = 0.0;
    total_time_with_IO = 0.0;
}

bool output_expected::operator==(const output_expected& other) const {
    return outputs == other.outputs;
    // io and  inverse
        
}

// --- File IO ---
output_expected readoutputexpected(string filename) {
    output_expected outputs_file;
    ifstream file(filename);
    string line;
    while (getline(file, line)) {
        if (line.empty() || line == "\r") {
            continue;
        }
        if (line.find("ms") != string::npos) {
            outputs_file.total_time_witout_IO = stod(line.substr(0, line.find(" ")));
            getline(file, line);
            if (line.empty() || line == "\r") getline(file, line);
            outputs_file.total_time_with_IO = stod(line.substr(0, line.find(" ")));
            break;
        }

        output_query o;
        stringstream ss(line);
        int node;
        while (ss >> node) {
            o.path.push_back(node);
        }

        getline(file, line);
        o.total_time = stod(line.substr(0, line.find(" ")));
        getline(file, line);
        o.total_distacne = stod(line.substr(0, line.find(" ")));
        getline(file, line);
        o.walking_distance = stod(line.substr(0, line.find(" ")));
        getline(file, line);
        o.vehicle_distance = stod(line.substr(0, line.find(" ")));
        outputs_file.outputs.push_back(o);
    }

    file.close();
    return outputs_file;
}

void writeoutput(output_expected out, string filename) {
    ofstream file(filename);
    if (!file.is_open()) {
        throw runtime_error("Could not open file for writing: " + filename);
    }

    file << fixed << setprecision(2);

    for (const auto& o : out.outputs) {
        for (size_t i = 0; i < o.path.size(); ++i) {
            file << o.path[i] << (i + 1 < o.path.size() ? ' ' : '\n');
        }
        file << o.total_time << " mins\n";
        file << o.total_distacne << " km\n";
        file << o.walking_distance << " km\n";
        file << o.vehicle_distance << " km\n";
        file << '\n';
    }

    file << fixed << setprecision(0);
    file << out.total_time_witout_IO << " ms\n\n"
         << out.total_time_with_IO << " ms\n";

    file.close();
}

// Optional: implement later
void write_output_file(const string& filename, const vector<output_query>& queries, const vector<output_query>& results) {
    // You can fill this in later if needed
}
