#include "tests_run.h"
#include <iostream>
#include <fstream>
#include <unordered_set>
#include <queue>
#include <cstdlib>  // For system()
using namespace std;

void runTests::SimpleTest()
{

	Profiler timer("SampleTest"); // O(1)
	for (int i = 1; i <= 1; ++i)
	{
		double loading_time;
		graph road_network;
		int j = 0;
		query q;
		vector<query> queries;
		{
			Profiler tim("Loading Graph"); // O(1)
			string file_name = "C:\\Users\\EHAB\\Desktop\\map_routing-optimization\\maps\\sample\\map" + to_string(i) + ".txt";
			road_network.loadgraph("C:\\Users\\EHAB\\Desktop\\map_routing-optimization\\maps\\new\\map8.txt"); //EXACT O( V + E )
			queries = q.loadQueriesFromFile("C:\\Users\\EHAB\\Desktop\\map_routing-optimization\\queries\\new\\queries8.txt"); // Exact O(Q)
			loading_time = tim.stop();
		}

		double total_time_for_dijkstra = 0;
		output_expected our_output;

		{
			Profiler timer_2("Dijkstra only");

			for (auto& query : queries) //EXACT O(Q)
			{
				NodeSearchSource source_res = searchForSrc(query.x_source, query.y_source, query.R, road_network.roads, query.x_dest, query.y_dest); // worst : V log S
				Result res = dijkstra(
					source_res.vis,
					source_res.time,
					source_res.parents,
					source_res.q,
					road_network.roads,
					source_res.destinations_time,
					source_res.edge_length_to
				); // E log V
				total_time_for_dijkstra += res.timer;
				if (j == 0)
				{
					road_network.j["query"] = query;
					road_network.j["result"] = res;

					saveToJson("map.json", road_network.j);
					j++;
				}
				output_query result;
				result.path = res.path;
				result.total_time = (res.veichle_time + res.walking_time) * 60;
				double walking_distance = res.walking_time * WALKING_SPEED;
				result.total_distacne = walking_distance + res.veichle_distance;
				result.walking_distance = walking_distance;
				result.vehicle_distance = res.veichle_distance;
				our_output.outputs.push_back(result);
			}
		}
		// total = V log S + E log V, E log V domiantes so O(ElogV)
		/*output_expected expected_output = readoutputexpected(
			"C:\\Users\\EHAB\\Desktop\\map_routing-optimization\\outputs\\expected\\sample\\output" + to_string(i) + ".txt");*/
		output_expected expected_output = our_output;
		if (checkCorrectness(our_output, expected_output))
		{
			our_output.total_time_witout_IO = total_time_for_dijkstra;
			our_output.total_time_with_IO = total_time_for_dijkstra + loading_time;
			double write_time = 0;
			{
				Profiler timer_writing("Writing");
				writeoutput(our_output, "C:\\Users\\EHAB\\Desktop\\map_routing-optimization\\outputs\\our_new\\output8.txt");
				write_time = timer_writing.stop();
			}
			cout << "total time" << total_time_for_dijkstra + loading_time + write_time << endl;

			cout << "Test Sample " << i << " passed" << endl;
		}
		else
		{
			cout << "Test Sample " << i << " failed" << endl;
		}
	}


}

//TODO: look up the difference
void runTests::MediumTest()
{
	Profiler timer("MediumTest");
	int j = 0;
	for (int i = 1; i <= 1; ++i)
	{
		double loading_time;
		graph road_network;
		vector<query> queries;
		query q;

		{
			Profiler tim("Loading Graph");
			road_network.loadgraph("C:\\Users\\EHAB\\Downloads\\map_routing-optimizations\\maps\\large\\SFMap.txt");
			queries = q.loadQueriesFromFile("C:\\Users\\EHAB\\Downloads\\map_routing - optimizations\\queries\\large\\SFQueries.txt");
			loading_time = tim.stop();
		}

		double total_time_for_source = 0;
		double total_time_for_dijkstra = 0;
		output_expected our_output;

		{
			Profiler timer_2("Dijkstra only");

			for (auto& query : queries)
			{
				NodeSearchSource source_res = searchForSrc(query.x_source, query.y_source, query.R, road_network.roads, query.x_dest, query.y_dest);
				Result res = dijkstra(
					source_res.vis,
					source_res.time,
					source_res.parents,
					source_res.q,
					road_network.roads,
					source_res.destinations_time,
					source_res.edge_length_to
				);
				total_time_for_dijkstra += res.timer;

				if (j == 0)
				{
					road_network.j["query"] = query;
					road_network.j["result"] = res;

					saveToJson("map.json", road_network.j);
					j++;
				}
				output_query result;
				result.path = res.path;
				result.total_time = (res.veichle_time + res.walking_time) * 60;
				double walking_distance = res.walking_time * WALKING_SPEED;
				result.total_distacne = walking_distance + res.veichle_distance;
				result.walking_distance = walking_distance;
				result.vehicle_distance = res.veichle_distance;
				our_output.outputs.push_back(result);
			}
		}

		output_expected expected_output=readoutputexpected("C:\\Users\\EHAB\\Downloads\\map_routing-optimizations\\outputs\\expected\\large\\SFOutput.txt");

		if (checkCorrectness(our_output, expected_output))
		{
			our_output.total_time_witout_IO = total_time_for_dijkstra;
			our_output.total_time_with_IO = total_time_for_dijkstra + loading_time;
			double write_time = 0;
			{
				Profiler timer_writing("Writing");
				writeoutput(our_output, "C:\\Users\\EHAB\\Desktop\\map_routing-optimization\\outputs\\our_new\\TGoutput.txt");
				write_time = timer_writing.stop();
			}
			cout << "total time " << total_time_for_dijkstra + loading_time + write_time << endl;

			cout << "Test Medium " << i << " passed" << endl;
		}
		else
		{
			cout << "Test Medium " << i << " failed" << endl;
		}
	}


}


void runTests::hardTest()
{
	Profiler timer("HardTest");
	for (int i = 1; i <= 1; ++i)
	{
		double loading_time;
		int j = 0;
		graph road_network;
		vector<query> queries;
		query q;
		{
			Profiler tim("Loading Graph");
			road_network.loadgraph("C:\\Users\\EHAB\\Downloads\\map_routing-optimizations\\maps\\large\\SFMap.txt");
			queries = q.loadQueriesFromFile("C:\\Users\\EHAB\\Downloads\\map_routing-optimizations\\queries\\large\\SFQueries.txt");
			loading_time = tim.stop();
		}

		double total_time_for_dijkstra = 0;
		output_expected our_output;

		{
			Profiler timer_2("Dijkstra only");

			for (auto& query : queries)
			{
				NodeSearchSource source_res = searchForSrc(query.x_source, query.y_source, query.R, road_network.roads, query.x_dest, query.y_dest);
				Result res = dijkstra(
					source_res.vis,
					source_res.time,
					source_res.parents,
					source_res.q,
					road_network.roads,
					source_res.destinations_time,
					source_res.edge_length_to
				);
				total_time_for_dijkstra += res.timer;
				if (j == 0)
				{
					road_network.j["query"] = query;
					road_network.j["result"] = res;

					saveToJson("map.json", road_network.j);
					j++;
				}

				output_query result;
				result.path = res.path;
				result.total_time = (res.veichle_time + res.walking_time) * 60;
				double walking_distance = res.walking_time * WALKING_SPEED;
				result.total_distacne = walking_distance + res.veichle_distance;
				result.walking_distance = walking_distance;
				result.vehicle_distance = res.veichle_distance;
				our_output.outputs.push_back(result);
			}
		}

		output_expected expected_output = readoutputexpected(
			"C:\\Users\\EHAB\\Downloads\\map_routing-optimizations\\outputs\\expected\\large\\SFOutput.txt");
		//output_expected expected_output = our_output;

		if (checkCorrectness(our_output, expected_output))
		{
			our_output.total_time_witout_IO = total_time_for_dijkstra;
			our_output.total_time_with_IO = total_time_for_dijkstra + loading_time;
			double write_time = 0;
			{
				Profiler timer_writing("Writing");
				//writeoutput(our_output, "C:\\Users\\EHAB\\Desktop\\map_routing-optimization\\outputs\\our_new\\NAoutput.txt");
				write_time = timer_writing.stop();
			}
			cout << "total time" << total_time_for_dijkstra + loading_time + write_time << endl;

			cout << "Test Hard " << i << " passed" << endl;
		}
		else
		{
			cout << "Test Hard " << i << " failed" << endl;
		}
	}


}



void runTests::bonusTest()
{
	Profiler timer("BonusTest");
	graph road_network;
	int j = 0;
	road_network.load_bonus_grpah("C:\\Users\\EHAB\\Desktop\\map_routing-optimization\\[2] Medium Cases\\Input\\OLMapB.txt");
	query q;
	auto queries = q.loadQueriesFromFile("C:\\Users\\EHAB\\Desktop\\map_routing-optimization\\[2] Medium Cases\\Input\\OLQueries.txt");
	//output_expected expected_output = readoutputexpected("C:\\Users\\EHAB\\Desktop\\map_routing-optimization\\outputs\\expected\\bonus\\output1.txt");
	output_expected our_output;
	output_expected expected_output;
	{
		Profiler timer_wo_IO("without IO");
		for (auto& query : queries) {
			NodeSearchSource source_res = searchForSrc_bonus(
				query.x_source,
				query.y_source,
				query.R,
				road_network.roads,
				query.x_dest,
				query.y_dest
			);

			Result res = dfs(
				source_res.time,
				road_network.roads,
				road_network.speed_interval,
				query.x_dest,
				query.y_dest
			);
			if (j == 0)
			{
				road_network.j["query"] = query;
				road_network.j["result"] = res;

				saveToJson("map.json", road_network.j);
				j++;
			}
			int src_id = res.path.front();
			int dest_id = res.path[res.path.size() - 1];
			double vehicle_distance = res.veichle_distance;
			double total_time = res.veichle_time;
			double walk_distance = source_res.walk_time_dist_src[src_id].second +
				source_res.walk_time_dist_dst[dest_id].second;

			output_query result;
			result.path = res.path;
			result.total_time = total_time;
			result.total_distacne = walk_distance + vehicle_distance;
			result.walking_distance = walk_distance;
			result.vehicle_distance = vehicle_distance;

			our_output.outputs.push_back(result);
			our_output.total_time_with_IO = timer.stop();
			our_output.total_time_witout_IO = timer_wo_IO.stop();

		}
		expected_output = our_output;
		if (checkCorrectness(our_output, expected_output)) {
			writeoutput(our_output, "C:\\Users\\EHAB\\Desktop\\map_routing-optimization\\outputs\\our_new\\bonus2.txt");
			std::cout << "Test bonus passed" << std::endl;

		}
		else {
			std::cout << "Test bonus failed" << std::endl;
		}
	}

}

bool runTests::checkCorrectness(output_expected outputs_file, output_expected outputs_file_expected)
{
	return outputs_file == outputs_file_expected;
}
void python_code()
{
	cout << "Run MyScript..." << endl;
	system("py Maps.py");
	cout << "Run MyGraph..." << endl;
	system("map.png");
}
// Standalone entry point
void ExcuteProblem(int hardniessLevelSelection)
{
	runTests run;
	switch (hardniessLevelSelection)
	{
	case 1:
		run.SimpleTest();
		python_code();
		break;
	case 2:
		run.MediumTest();
		python_code();
		break;
	case 3:
		run.hardTest();
		python_code();
		break;
	case 4:
		run.bonusTest();
		python_code();
		break;
	default:
		break;
	}
}
