#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <math.h> 
#include <vector>
#include <cstdlib> 
#include <sstream>
#include <map>
#include <fstream>
#include <ctime>
#include <set>
#include <limits>
#include <cmath>
#include <algorithm>

#define DRONE_WEIGHT 10
#define DRONE_AVERAGE_SPEED 15 // km/hour
#define SEED 1 // seed time in srand()

using namespace std;

//======================================================================

typedef struct{
	string edge_id;
	string from;
	string to;
	vector<string> lanes;
	vector<double> length; 
}edge;

typedef struct{
	string lane_id;
	double length;
	struct junction *from;
	struct junction *to;
}lane;

typedef struct junction{
	string junction_id;
    int id;
	double x;
	double y;
	int outcomingRoads;
	int area_id;
	vector<lane *> outLanes;
	vector<lane *> backLanes;
}junction;

typedef struct destination{
    double x;
    double y;
    double packet;
    int packet_id;

    bool operator<(const destination& rhs) const{ //sorts the packets in descending order
        return packet > rhs.packet;        
    }   
}destination;

typedef struct dijkstra_node{
    junction *node;
    double distance_to_node;
    bool operator<(const dijkstra_node& rhs) const{
        return distance_to_node < rhs.distance_to_node;
    }
}dijkstra_node;

typedef struct bounding_area{
	double from_x;
	double to_x;
	double from_y;
	double to_y;
	int area_id;
	multiset<destination> sorted_destinations;
    bool operator<(const bounding_area& rhs) const{ //sorts the areas in descending order
    	double sum = 0;
    	double sum2 = 0;
		for(multiset<destination>::iterator it = sorted_destinations.begin(); it != sorted_destinations.end(); ++it){
			sum += (*it).packet;
		}
		for(multiset<destination>::iterator it = rhs.sorted_destinations.begin(); it != rhs.sorted_destinations.end(); ++it){
			sum2 += (*it).packet;
		}
        return sum > sum2;        
    } 
}bounding_area;


//======================================================================

// Function prototypes
string getFile( string filename );                         											// Reads whole file into a string buffer
vector<string> getData( const string &text, string tag );  											// Gets collection of items between given tags
vector<string> split( string str, char sep ); 			   											// Tokenize the vector of strings with delimiter ' '

junction *junction_separator(string s, map<string,lane*> &lane_map);						   		// Separates junction strings in substrings
edge *edge_separator(string s);                             										// Separates edge strings in substrings
lane *lanes_separator(string s, vector<edge *> &edges);	  											// Separates lane strings in sunstrings

void plotGraph(map<string,junction*>& junction_map, vector<junction *>& prev, junction *source, junction *dest, junction *real_dest); //plots a graph with junctions, lanes and distances

void create_random_destinations(double min_x, double min_y, double max_x, double max_y, vector<destination> &destinations);	//creates random destinations
void create_zipfian_destinations(double min_x, double min_y, double max_x, double max_y, vector<destination> &destinations);			//creates zipfian destinations
void create_random_packets(vector<destination> &destinations); 										//creates a random weight packet for each destination
void create_zipfian_packets(vector<destination> &destinations);										//creates a zipfian-weight packet for each destination
int zipf(double theta, int parts);
double rand_min_to_max(double Min, double Max); 													//generates a uniform random number between min and max

void separate_area(double xmin, double ymin, double xmax, double ymax, vector<bounding_area> &areas); //separates graph in n areas
void sort_destinations_per_area(vector<destination> &destinations, vector<bounding_area> &areas); //sorts the destinations for each area
void sort_areas(vector<bounding_area> &areas, multiset<bounding_area> &sorted_areas); //sorts areas by their total weight
void junctions_per_area(map<string,junction*>& junction_map, vector<bounding_area> &areas); //finds the junctions which belong to each area

junction *find_closest_junction_from_destination(destination dest, vector<double> &dist, map<string,junction*>& junction_map); //finds the closest junction only when there is a path 
junction *find_closest_junction(destination dest, map<string,junction*>& junction_map); //finds the closest junction regardless of whether there is a path or not
junction *go_back(junction *curr_junction, double &distance_covered_by_carrier); //goes back when the current junction has < 1 outgoing lanes 

double euclidean_distance (double x_junction, double y_junction, double x_dest, double y_dest); //computes euclidean distance between two points
double energy_consumption(double distance, double packet); // computes the energy consumption for a round trip
double flight_time(double distance);

void dijkstra_algorithm(map<string,junction*>& junction_map, junction *source, vector<junction *>& prev, vector<double> &dist); //finds all the closest paths from the source junction
bool exists_in_q(junction *node, multiset<dijkstra_node> &q); 
void decreace_priority(junction *node, double priority, multiset<dijkstra_node> &q );

void send_one_packet_at_a_time(multiset<bounding_area> &sorted_areas, map<string,junction*> &junction_map);

//======================================================================

//Global variables 
string filename;
int n = 1; //number of areas = n*n
int destinations_num = 10; //number of destinations
int debug_mode = 0; 
double theta = 1;
int parts = 20;
int destination_algorithm = 0;
int packet_algorithm = 0;
double max_packet_weight = 25;
double min_packet_weight = 0.1;

vector<double> sum_probs;     // Pre-calculated sum of probabilities
double converted_drone_speed = (DRONE_AVERAGE_SPEED*1000.0) / 3600.0; // m/s
//======================================================================

int main(int argc, char *argv[]){
    
    srand(SEED);

    double max_x = 0;
    double max_y = 0;
    double min_x = numeric_limits<double>::max();
    double min_y = numeric_limits<double>::max();

    vector<edge *> edges;
    vector<destination> destinations;
    vector<bounding_area> areas;
    
    multiset<bounding_area> sorted_areas;

    vector<bounding_area> sorted_destinations_per_area;

    map<string,lane*> lane_map;
    map<string,junction*> junction_map;
    map<string,lane*>::iterator lane_it;

   	FILE *myFile;
	char input_file[10];
	char input[100];
	vector<string> tokens;

	if(argc == 2){
		strcpy(input_file, argv[1]);
	}else if( argc > 2 ) {
      	printf("Too many arguments supplied.\n");
      	exit(-1);
    }else{
    	printf("One argument expected.\n");
      	exit(-1);
    }	

    myFile = fopen(input_file, "r");

    while(1){
    	if(fgets(input, 100, (FILE*)myFile)!= NULL){
    		
    		tokens = split(input, ' ');
    		if(tokens.size() != 2 ){
    			continue;
    		}
    		for(size_t i = 0; i < tokens.size(); i++){
        		if(tokens[i] == "input_file"){
        			filename = tokens[i+1].substr(0, tokens[i+1].size()-1);      			
        		}else if(tokens[i] == "algorithm_for_destinations"){
        			string temp = tokens[i+1].substr(0, tokens[i+1].size()-1);
        			if(temp == "random"){
        				destination_algorithm = 0;
        			}else{
        				destination_algorithm = 1;
        			}
        			
        		}else if( tokens[i] == "algorithm_for_packets"){
        			string temp = tokens[i+1].substr(0, tokens[i+1].size()-1);
        			if(temp == "random"){
        				packet_algorithm = 0;
        			}else{
        				packet_algorithm = 1;
        			}
        			
        		}else if(tokens[i] == "N_for_areas"){
        			n = stoi(tokens[i+1]);
        			
        		}else if(tokens[i] == "number_of_destinations"){
        			destinations_num = stoi(tokens[i+1]);
        		}else if(tokens[i] == "debug_mode"){
        			debug_mode = stoi(tokens[i+1]);
        		}else if(tokens[i] == "theta"){
        			theta = stod(tokens[i+1]);
        		}else if(tokens[i] == "parts"){
        			parts = stoi(tokens[i+1]);
        		}else if(tokens[i] == "max_packet_weight"){
        			max_packet_weight = stod(tokens[i+1]);
        		}else if(tokens[i] == "min_packet_weight"){
        			min_packet_weight = stod(tokens[i+1]);
        		}
        	}
    		
    	}else{
    		break;
    	}
    }  

    if(min_packet_weight <= 0 || max_packet_weight <= 0 ){
    	cout << "packet weight must be greater than 0\n";
    	exit(-1);
    }

    if(max_packet_weight <= min_packet_weight){
    	cout << "Max packet weight must be greater than min packet weight\n";
    	exit(-1);
    }

    if(theta < 0 || theta > 1){
    	cout << "Theta must be between 0 and 1\n";
    	exit(-1);
    }

    if(parts <= 0){
    	cout << "Parts must be greater than 0\n";
    	exit(-1);
    }

    if(n <= 0){
    	cout << "n must be greater than 0\n";
    	exit(-1);
    }

    if(destinations_num <= 0){
    	cout << "Number of destinations must be greater than 0\n";
    	exit(-1);
    }

    if(filename.empty()){
    	cout << "Filename is empty!\n";
    	exit(-1);
    }

    string text = getFile( filename );

    //Edges
    vector<string> all2 = getData( text, "edge" );
    for ( string &s : all2 ) {
   	    edges.push_back(edge_separator(s)); 
    }   

    //Lanes  
    vector<string> all3 = getData( text, "lane");
    for ( string &s : all3 ) {
   	    lane *new_lane = lanes_separator(s, edges); 
   		lane_map.insert (pair<string,lane*>(new_lane->lane_id,new_lane) );		
    }

    //Junctions
    vector<string> all = getData( text, "junction" );
    for ( string &s : all ) {
   		junction *new_junction = junction_separator(s, lane_map);
   		new_junction->outcomingRoads = 0;
   		new_junction->area_id = -1;
   		junction_map.insert (pair<string,junction*>(new_junction->junction_id,new_junction) );

        if(max_x < new_junction->x){ //find max value of x
            max_x = new_junction->x;
        }

        if(max_y < new_junction->y){ //find max value of y 
            max_y = new_junction->y;
        }

        if(min_x > new_junction->x){ //find min value of x
            min_x = new_junction->x;
        }

        if(min_y > new_junction->y){ //find min value of y 
            min_y = new_junction->y;
        }

    }

    //Iterate the edges to get the outcoming lanes
    for(size_t i = 0; i < edges.size(); i++){ //for every edge
        map<string,junction*>::iterator from_it = junction_map.find(edges[i]->from); //find the from-junction
		map<string,junction*>::iterator to_it = junction_map.find(edges[i]->to); //find the to-junction
		
		if (from_it != junction_map.end() && to_it != junction_map.end()) { //if from-junction exists and to-junction exist
			from_it->second->outcomingRoads++;
			for(size_t j = 0; j < edges[i]->lanes.size(); j++){ //for every lane of the edge
			    lane_it = lane_map.find(edges[i]->lanes[j]); //get its pointer
			    
			    if(lane_it != lane_map.end()) {
					lane_it->second->from = from_it->second; //set the from pointer to be the from-junction
					lane_it->second->to = to_it->second; //set the to pointer to be the to-junction
					from_it->second->outLanes.push_back(lane_it->second); //add it to the outcoming lanes
				
					//create back lanes 
					lane *new_lane = new lane;
					new_lane->lane_id = lane_it->second->lane_id;
					new_lane->length = lane_it->second->length;
					new_lane->from = to_it->second;
					new_lane->to = from_it->second;
					to_it->second->backLanes.push_back(new_lane);
			    }		
			}
		}
	}  


	//functions for destinations and packets
	if(destination_algorithm == 0) {
		create_random_destinations(min_x, min_y, max_x, max_y, destinations);
	} else if(destination_algorithm == 1){
		create_zipfian_destinations(min_x, min_y, max_x, max_y, destinations);
	}else{
		cout << "Give correct algorithm for destinations\n";
		exit(-1);
	}
    
    if(packet_algorithm == 0){
    	create_random_packets(destinations);
    }else if(packet_algorithm == 1){
    	create_zipfian_packets(destinations);
    }else{
    	cout << "Give correct algorithm for packets\n";
    	exit(-1);
    }

    separate_area(min_x, min_y, max_x, max_y, areas); // separates graph in areas

	junctions_per_area(junction_map, areas);

	sort_destinations_per_area(destinations, areas); 


	sort_areas(areas, sorted_areas); //sorts areas by their total weight 

    send_one_packet_at_a_time(sorted_areas, junction_map);



    //free memory allocations
	for(size_t i = 0; i < edges.size(); i++) { //cleanup edges
		delete edges[i];
	}
	edges.clear();

	int counter = 0;

    for(map<string,junction*>::iterator it = junction_map.begin(); it != junction_map.end(); ++it){
    	if(!it->second->outLanes.empty() || !it->second->backLanes.empty()){
    		counter ++;
    	}
	    for(size_t i = 0; i < it->second->outLanes.size(); i++) {
		    delete it->second->outLanes[i];
	    }
	    for(size_t i = 0; i < it->second->backLanes.size(); i++) {
		    delete it->second->backLanes[i];
	    }
	    it->second->backLanes.clear();
	    it->second->outLanes.clear();
	    delete it->second;
    }
    
    junction_map.clear();

    destinations.clear();
    
    cout << "\nNumber of Junctions: " << counter << '\n';

    return(0);

}

//======================================================================

void send_one_packet_at_a_time(multiset<bounding_area> &sorted_areas, map<string,junction*> &junction_map){
	
	map<string,junction*>::iterator it_junction = junction_map.begin();
	
	vector<double> dist;
    vector<junction *> prev;

	junction *curr_junction = it_junction->second;
	double distance_covered_by_carrier = 0;

	double total_energy_consumption = 0;
	double total_flight_time = 0;
	vector<int> delivered_packets;

	double distance_sum = 0;
	int execution_count = 0;


	for(multiset<bounding_area>::iterator it_area = sorted_areas.begin(); it_area != sorted_areas.end(); ++it_area){

		cout << "\nArea id: " << (*it_area).area_id << '\n';

		for(multiset<destination>::iterator it_dest = (*it_area).sorted_destinations.begin(); it_dest != (*it_area).sorted_destinations.end(); ++it_dest){

			if ( find(delivered_packets.begin(), delivered_packets.end(), (*it_dest).packet_id) != delivered_packets.end() ){
				continue;
			}

			if(prev.size() == 0 && curr_junction->outcomingRoads == 0){ // when dijkstra algorithm is not executed and no dijkstra path exists
				cout << "Deadend!\n";
				cout << "No dijkstra path exists. Trying a new path.\n";
    			while(curr_junction->outcomingRoads <= 1){
    		
    				curr_junction = go_back(curr_junction, distance_covered_by_carrier);
    				if(curr_junction == NULL){
    					cout << "The junction was invalid! Terminating!\n";
    					exit(-1);
    				}
    				cout << "Going back to: " << curr_junction->junction_id << '\n';
    			} 			
    		}else if(prev.size() != 0 && curr_junction->outcomingRoads == 0){ //when dijkstra path exists
    			cout << "Deadend!\n";

    			bool no_road_found = false;
    			while(curr_junction->outcomingRoads <= 1){
    				
    				if(prev[curr_junction->id] != NULL) { //we reached the source junction
    					double prev_distance = dist[curr_junction->id];
    					curr_junction = prev[curr_junction->id];
    					double new_distance = dist[curr_junction->id];
	    				distance_covered_by_carrier += abs(prev_distance - new_distance);
	    				cout << "Distance covered by drone carrier: " << distance_covered_by_carrier << '\n';
	    				cout << "Going back to: " << curr_junction->junction_id << '\n';
    				}
    				else {
    					no_road_found = true;
    					break;
    				}	
    			}

    			if(no_road_found) {
    				cout << "Cannot go back using the dijkstra path. Trying a new path.\n";
					while(curr_junction->outcomingRoads <= 1){  		
	    				curr_junction = go_back(curr_junction, distance_covered_by_carrier);
	    				if(curr_junction == NULL){
	    					cout << "The junction was invalid! Terminating!\n";
	    					exit(-1);
	    				}
	    				cout << "Going back to: " << curr_junction->junction_id << '\n';
    				}
    			}
    		} 	   	

    		dijkstra_algorithm(junction_map, curr_junction, prev, dist);

			junction *next_junction = find_closest_junction_from_destination((*it_dest), dist, junction_map);

			if(next_junction == NULL) {
				cout << "The junction was invalid! Terminating!\n";
    			exit(-1);
			}

			junction *real_closest_junction = find_closest_junction((*it_dest), junction_map);

			if(real_closest_junction != NULL){
				double distance1 = euclidean_distance(next_junction->x, next_junction->y, (*it_dest).x, (*it_dest).y);
				double distance2 = euclidean_distance(real_closest_junction->x, real_closest_junction->y, (*it_dest).x, (*it_dest).y);
				distance_sum += abs(distance1 - distance2);
				execution_count++;
			}

			if(debug_mode == 1){
				plotGraph(junction_map, prev, curr_junction, next_junction, real_closest_junction);
			}

			curr_junction = next_junction;

			distance_covered_by_carrier += dist[curr_junction->id];
			cout << "Distance covered by drone carrier: " << distance_covered_by_carrier << '\n';

			cout << "Delivering packet to destination: " << curr_junction->junction_id << " with weight: " << (*it_dest).packet << '\n';
			double distance = euclidean_distance (curr_junction->x, curr_junction->y, (*it_dest).x, (*it_dest).y);
			total_energy_consumption += energy_consumption(distance, (*it_dest).packet);
			total_flight_time += flight_time(distance);
			
			delivered_packets.push_back((*it_dest).packet_id);
		
			for(multiset<bounding_area>::iterator it_extra_area = sorted_areas.begin(); it_extra_area != sorted_areas.end(); ++it_extra_area){
				for(multiset<destination>::iterator it_extra_dest = (*it_extra_area).sorted_destinations.begin(); it_extra_dest != (*it_extra_area).sorted_destinations.end(); ++it_extra_dest){
				
					if(find(delivered_packets.begin(), delivered_packets.end(), (*it_extra_dest).packet_id) != delivered_packets.end() ) {
						continue;
					}

					junction *extra_closest_junction = find_closest_junction((*it_extra_dest), junction_map);
					if(extra_closest_junction != NULL && extra_closest_junction == curr_junction){
						cout << "Delivering more packets to destination: " << extra_closest_junction->junction_id << " with weight: " << (*it_extra_dest).packet << '\n';
						double distance = euclidean_distance (curr_junction->x, curr_junction->y, (*it_extra_dest).x, (*it_extra_dest).y);
						total_energy_consumption += energy_consumption(distance, (*it_extra_dest).packet);
						total_flight_time += flight_time(distance);
						delivered_packets.push_back((*it_extra_dest).packet_id);
					}

				}
			}

			

			//debug code
			if(debug_mode == 1){
				char c;
				cout << "give char n " << '\n';
				cin >> c;
				if( c == 'n'){
					continue;
				}else {
					break;
				}				
			}

		}
	}
	cout << "\nTotal energy consumption in Watts: " << total_energy_consumption << '\n';
	cout << "Energy consumption Watt/hour approximately: " << total_energy_consumption / (total_flight_time / 3600) << '\n';
	cout << "Total drone flight time in hours: " << total_flight_time / 3600 << '\n';
	cout << "Throughput packets per hour: " << destinations_num / (total_flight_time / 3600) << '\n';
	cout << "Average Error Distance in meters from Real Junction: " << (distance_sum / execution_count) << "\n";

}

//======================================================================

double flight_time(double distance){
	return((2*distance) / converted_drone_speed);
}

//======================================================================

junction *go_back(junction *curr_junction, double &distance_covered_by_carrier){
	
	if(curr_junction->backLanes.size() == 0){
		return(NULL);
	}

	for(size_t i = 0; i < curr_junction->backLanes.size(); i++){
		if(curr_junction->backLanes[i]->to->outcomingRoads > 1){
			distance_covered_by_carrier += curr_junction->backLanes[i]->length;
			cout << "Distance covered by drone carrier: " << distance_covered_by_carrier << '\n';
			return(curr_junction->backLanes[i]->to);
		}
	}
	distance_covered_by_carrier += curr_junction->backLanes[0]->length;
	cout << "Distance covered by drone carrier: " << distance_covered_by_carrier << '\n';
	return(curr_junction->backLanes[0]->to);

}

//======================================================================

junction *find_closest_junction_from_destination(destination dest, vector<double> &dist, map<string,junction*>& junction_map){
	
	double min_x = numeric_limits<double>::max();
	junction *min_junction = NULL;

	for(size_t i = 0; i < dist.size(); i++){
		if(dist[i] != numeric_limits<double>::max()){
			for(map<string,junction*>::iterator it = junction_map.begin(); it != junction_map.end(); ++it){
				if (int(i) == it->second->id){
					double norm = euclidean_distance(it->second->x, it->second->y, dest.x, dest.y);
					if(norm < min_x){
						min_x = norm;
						min_junction = it->second;
					}
					break;
				}
			}
		}
	}

	return(min_junction);
}

//======================================================================

junction *find_closest_junction(destination dest, map<string,junction*>& junction_map){
	
	double min_x = numeric_limits<double>::max();
	junction *min_junction = NULL;


	for(map<string,junction*>::iterator it = junction_map.begin(); it != junction_map.end(); ++it){
		if(it->second->outLanes.size() == 0 && it->second->backLanes.size() == 0) {
			continue;
		}
		double norm = euclidean_distance(it->second->x, it->second->y, dest.x, dest.y);
		if(norm < min_x){
			min_x = norm;
			min_junction = it->second;
		}			
	}

	return(min_junction);
}

//======================================================================

double euclidean_distance (double x_junction, double y_junction, double x_dest, double y_dest){
	
	return sqrt(pow(x_dest - x_junction,2) + pow(y_dest - y_junction,2));

}

//======================================================================

double energy_consumption(double distance, double packet){ //Watt = Joule / s = (kg*m^2) / s^3
	
	double energy_Joule1 = pow(converted_drone_speed,2) * (packet + DRONE_WEIGHT); 
	double energy_Joule2 = pow(converted_drone_speed,2) * DRONE_WEIGHT;
	double energy_Watt = (energy_Joule1 + energy_Joule2) / (distance / converted_drone_speed);
	return(energy_Watt);

}

//======================================================================

void separate_area(double xmin, double ymin, double xmax, double ymax, vector<bounding_area> &areas){
	int counter = 0;
	for(int i = 0; i < n; i++){
		for(int j = 0; j < n; j++, counter++){
			bounding_area area;
			area.from_x = xmin + ((xmax - xmin) / n) * i;
			area.to_x = xmin + ((xmax - xmin) / n) * (i + 1);
			area.from_y = ymin + ((ymax - ymin) / n) * j;
			area.to_y = ymin + ((ymax - ymin) / n) * (j + 1);
			area.area_id = counter;
			areas.push_back(area);
 		}	
	}

}

//====================================================================== 
void junctions_per_area(map<string,junction*>& junction_map, vector<bounding_area> &areas){

	for(map<string,junction*>::iterator it = junction_map.begin(); it != junction_map.end(); ++it){
		for(size_t i = 0; i < areas.size(); i++) {
			if(it->second->x >= areas[i].from_x && it->second->x <= areas[i].to_x
				&& it->second->y >= areas[i].from_y && it->second->y <= areas[i].to_y){
				it->second->area_id = areas[i].area_id;
				break;
			}

			if((abs(it->second->x - areas[i].from_x) < 1e-7 && abs(it->second->y - areas[i].from_y) < 1e-7)
				|| (abs(it->second->x - areas[i].to_x) < 1e-7 && abs(it->second->y - areas[i].to_y) < 1e-7)) {
				it->second->area_id = areas[i].area_id;
				break;
			}
		}
	}

	for(map<string,junction*>::iterator it = junction_map.begin(); it != junction_map.end(); ++it){
		if(it->second->area_id < 0) {
			cout << "AREA WITH NEGATIVE ID: " << it->second->junction_id << " X: " << it->second->x << " Y: " << it->second->y << "\n";
		}
	}
}

//====================================================================== 

void sort_destinations_per_area(vector<destination> &destinations, vector<bounding_area> &areas) {

	for(size_t i = 0; i < destinations.size(); i++){
		for(size_t j = 0; j < areas.size(); j++) {
			if(destinations[i].x >= areas[j].from_x && destinations[i].x <= areas[j].to_x 
			&& destinations[i].y >= areas[j].from_y && destinations[i].y <= areas[j].to_y) {
				areas[j].sorted_destinations.insert(destinations[i]);
				break;
			}

			if((abs(destinations[i].x - areas[j].from_x) < 1e-7 && abs(destinations[i].y - areas[j].from_y) < 1e-7)
				|| (abs(destinations[i].x - areas[j].to_x) < 1e-7 && abs(destinations[i].y - areas[j].to_y) < 1e-7)) {
				areas[j].sorted_destinations.insert(destinations[i]);
				break;
			}
		}
	}

}

//======================================================================

void sort_areas(vector<bounding_area> &areas, multiset<bounding_area> &sorted_areas){

	for(size_t i = 0; i < areas.size(); i++){
		sorted_areas.insert(areas[i]);
	}

}

//======================================================================

void dijkstra_algorithm(map<string,junction*>& junction_map, junction *source, vector<junction *>& prev, vector<double> &dist){

	if(source == NULL) {
		cout << "The junction was invalid! Terminating!\n";
	    exit(-1);
	}

	cout << "Performing dijkstra for source junction " << source->junction_id << "\n";
    
    multiset<dijkstra_node> q;

    dist.resize(junction_map.size());

    prev.clear();
    prev.resize(junction_map.size());

    dist[source->id] = 0;

    for(map<string,junction*>::iterator it = junction_map.begin(); it != junction_map.end(); ++it){
        dijkstra_node new_dijkstra_node;
        prev[it->second->id] = NULL;
        if(it->second->junction_id != source->junction_id){
            dist[it->second->id] = numeric_limits<double>::max();
        }

        new_dijkstra_node.node = it->second;
        new_dijkstra_node.distance_to_node = dist[it->second->id];
        q.insert(new_dijkstra_node);
    }

    while(!q.empty()) {
        
        junction *u = q.begin()->node;

        q.erase(q.begin());
        for(size_t i = 0; i < u->outLanes.size(); i++){
            junction *v = u->outLanes[i]->to;
            if(exists_in_q(v,q) == false){
                continue;
            }
            double alt = dist[u->id] + u->outLanes[i]->length;

            if(alt < dist[v->id]){
                dist[v->id] = alt;
                prev[v->id] = u;
                decreace_priority(v, alt, q);
            }
        }
    }

}

//======================================================================

bool exists_in_q(junction *node, multiset<dijkstra_node> &q){
    
    for(multiset<dijkstra_node>::iterator it = q.begin(); it != q.end(); ++it){
       if(it->node == node){
            return true;
       }
    }
    
    return false;
}

//======================================================================

void decreace_priority(junction *node, double priority, multiset<dijkstra_node> &q ){
    multiset<dijkstra_node>::iterator find_it = q.end();
    for(multiset<dijkstra_node>::iterator it = q.begin(); it != q.end(); ++it){
        if(it->node == node){
            find_it = it;
            break;
        }
    }

    if(find_it != q.end()){
        q.erase(find_it);
        dijkstra_node new_node;
        new_node.node = node;
        new_node.distance_to_node = priority;
        q.insert(new_node);
    }
}

//======================================================================

int zipf(double theta, int parts){                      

  // Compute normalization constant on first call only
  	if(sum_probs.empty()){
  		sum_probs.resize(parts + 1);
  		double c = 0;          
    	for (int i = 1; i <= parts; i++){
    		c = c + (1.0 / pow((double) i, theta));
    	}
    	c = 1.0 / c;

    	sum_probs[0] = 0;
    	for (int i = 1; i <= parts; i++) {
      		sum_probs[i] = sum_probs[i-1] + c / pow((double) i, theta);
    	}
  	}

  	// Pull a uniform random number (0 < z < 1)
  	double z;
  	do{
    	z = rand_min_to_max(0, 1);
  	}while ((z == 0) || (z == 1));

  	// Map z to the value
  	int low = 1, high = parts, mid;
  	int zipf_value;               // Computed exponential value to be returned
  	do{
    	mid = floor((low+high)/2);
    	if (sum_probs[mid] >= z && sum_probs[mid-1] < z) {
      		zipf_value = mid;
      		break;
    	} else if (sum_probs[mid] >= z) {
      		high = mid-1;
    	} else {
      		low = mid+1;
    	}
  	}while (low <= high);

  // Assert that zipf_value is between 1 and N
  if(!((zipf_value >=1) && (zipf_value <= parts))){
  	cout << "Zipfian value out of range!\n";
  	exit(-1);
  }

  return (zipf_value);
}

//======================================================================    

void create_zipfian_destinations(double min_x, double min_y, double max_x, double max_y, vector<destination> &destinations){
    
    for( int i = 0; i < destinations_num; i++){
        destination new_destination; 

        new_destination.x = zipf(theta, parts) * ((max_x - min_x) / parts) + min_x;
        new_destination.y = zipf(theta, parts) * ((max_y - min_y) / parts) + min_y;
        new_destination.packet_id = i;
        destinations.push_back(new_destination);
    }   
    
}

//======================================================================

double rand_min_to_max(double Min, double Max){

    return Min + (rand() / ( RAND_MAX / (Max-Min) ) ) ;  
}

//======================================================================
    
void create_random_destinations(double min_x, double min_y, double max_x, double max_y, vector<destination> &destinations){   

    for(int i = 0; i < destinations_num; i++){
        destination new_destination;

        double num_x = rand_min_to_max(min_x, max_x);
        double num_y = rand_min_to_max(min_y, max_y);
        new_destination.x = num_x;
        new_destination.y = num_y;
        new_destination.packet_id= i;
        destinations.push_back(new_destination);
    }

}

//======================================================================

void create_random_packets(vector<destination> &destinations){

    for(size_t i = 0; i < destinations.size(); i++){
        
        destinations[i].packet = rand_min_to_max(min_packet_weight, max_packet_weight);
    }
}

//======================================================================

void create_zipfian_packets(vector<destination> &destinations){

	for(size_t i = 0; i < destinations.size(); i++){
		destinations[i].packet = zipf(theta, parts) * ((max_packet_weight - min_packet_weight) / parts) + min_packet_weight;
	}

}

//====================================================================== 

junction *junction_separator(string s, map<string,lane*> &lane_map){
    
	vector<string> str;
	map<string,lane*>::iterator it;
    static int counter = 0;

	junction *new_junction = new junction;
	if (new_junction == NULL) {
        printf("Out of memory!\n");
        exit(-1);
    }

   	size_t pos = s.find("id=\"");
   	if(pos != string::npos){
   		size_t pos2 = s.find("\"", pos+4);
   		if (pos != string::npos){
   			string result = s.substr(pos+4, pos2-(pos+4));
   			new_junction->junction_id = result;
            new_junction->id = counter;
            counter++;
   		}
   	}

   	pos = s.find("x=\"");
   	if(pos != string::npos){
   		size_t pos2 = s.find("\"", pos+3);
   		if (pos != string::npos){
   			string result = s.substr(pos+3, pos2-(pos+3));
   			double x = stod (result); 
   			new_junction->x = x;
   		}
   	}

   	pos = s.find("y=\"");
   	if(pos != string::npos){
   		size_t pos2 = s.find("\"", pos+3);
   		if (pos != string::npos){
   			string result = s.substr(pos+3, pos2-(pos+3));
   			double y = stod (result); 
   			new_junction->y = y;
   		}
   	} 

   	return(new_junction);

}

//======================================================================

edge *edge_separator(string s){
	
    edge *new_edge = new edge;
	if (new_edge == NULL) {
        printf("Out of memory!\n");
        exit(-1);
    }


   	size_t pos = s.find("id=\"");
   	if(pos != string::npos){
   		size_t pos2 = s.find("\"", pos+4);
   		if (pos != string::npos){
    		string result = s.substr(pos+4, pos2-(pos+4));
    		new_edge->edge_id = result;			  				
   		}
   	}

   	pos = s.find("from=\"");
   	if(pos != string::npos){
   		size_t pos2 = s.find("\"", pos+6);
   		if (pos != string::npos){
   			string result = s.substr(pos+6, pos2-(pos+6));
   			new_edge->from = result;
   		}
   	}

   	pos = s.find("to=\"");
   	if(pos != string::npos){
   		size_t pos2 = s.find("\"", pos+4);
   		if (pos != string::npos){
   			string result = s.substr(pos+4, pos2-(pos+4));
   			new_edge->to = result;
   		}
   	}
    
    return(new_edge);

}

//======================================================================

lane *lanes_separator(string s, vector<edge *> &edges){
	
    vector<string> str1;

    lane *new_lane = new lane;
	if (new_lane == NULL) {
        printf("Out of memory!\n");
        exit(-1);
    }


    edge *find_edge = NULL;

   	size_t pos = s.find("id=\"");
   	if(pos != string::npos){
   		size_t pos2 = s.find("\"", pos+4);
   		if (pos != string::npos){
   			string result = s.substr(pos+4, pos2-(pos+4));  
			
			int match_length = numeric_limits<int>::max();
            for(size_t i = 0; i < edges.size(); i++){
		 		int current_length_diff = abs((int)result.size() - (int)edges[i]->edge_id.size());
                if(result.find(edges[i]->edge_id) == 0 &&  current_length_diff < match_length){ //find the best match
                    find_edge = edges[i];                
		     		match_length = current_length_diff;
		        }
            }
        	if(find_edge != NULL) {
				find_edge->lanes.push_back(result);
	    	}
   		}
    }
   	

   	pos = s.find("length=\"");
   	if(pos != string::npos){
   		size_t pos2 = s.find("\"", pos+8);
   		if (pos != string::npos){
   			string result = s.substr(pos+8, pos2-(pos+8));
   			double len = stod (result); 
            if(find_edge != NULL){
                find_edge->length.push_back(len);
            }											
   		}
   	}

   	

	for(size_t i = 0; i < edges.size(); i++){
   		for(size_t j = 0; j < edges[i]->lanes.size(); j++){
   			new_lane->lane_id = edges[i]->lanes[j];
   			new_lane->length = edges[i]->length[j];
   		}
        new_lane->from = NULL;
        new_lane->to = NULL;	
    }

	return(new_lane);

}

//======================================================================

vector<string> split( string str, char sep){
	vector<string> ret ;

	istringstream stm(str) ;
	string token ;
	while(getline( stm, token, sep ) ){
        ret.push_back(token);
	}

	return(ret);
}

//======================================================================

void plotGraph(map<string,junction*>& junction_map, vector<junction *>& prev, junction *source, junction *dest, junction *real_dest){

    ofstream graph_file;

    graph_file.open( "graph" );

    graph_file.precision( 3 );
    graph_file << fixed;
    graph_file << "digraph g {\nrankdir=LR;\nranksep=equally;\n";

    for(map<string,junction*>::iterator it = junction_map.begin(); it != junction_map.end(); ++it) {
	    if(it->second->outLanes.empty() && it->second->backLanes.empty()) {
		 continue;   
	    }
	    graph_file << "subgraph \"cluster_" << to_string(it->second->area_id) << "\" {style=bold; color=black; label=\"" << to_string(it->second->area_id) << "\";\n";
	    if(source == it->second) {
			graph_file << "node[shape=circle, color=green, style=bold];\n";    
	    }else if (dest == it->second) {
	    	graph_file << "node[shape=circle, color=blue, style=bold];\n";
	    }else if(real_dest == it->second){
	    	graph_file << "node[shape=circle, color=red, style=bold];\n";
	    }
	    else {
			graph_file << "node[shape=circle, color=black, style=bold];\n";
	    }
	    
	    
	    graph_file << "\"" << it->second->junction_id << "\"\n";
	    graph_file << "}\n";
    }


    for(map<string,junction*>::iterator it = junction_map.begin(); it != junction_map.end(); ++it) {
        
        for(size_t i = 0; i < it->second->outLanes.size(); i++) {
            if(it->second->outLanes[i]->to != NULL) {
                graph_file << "\"" << it->second->junction_id << "\"" <<" -> " << "\"" << it->second->outLanes[i]->to->junction_id << "\"" << " [style=bold, color=blue, label=\""
                    << it->second->outLanes[i]->lane_id << " (" << it->second->outLanes[i]->length << ") " << "\"];\n";
            }
        }
        
        /*for(size_t i = 0; i < it->second->backLanes.size(); i++) {
            if(it->second->backLanes[i]->to != NULL) {
                graph_file << "\"" << it->second->junction_id << "\"" <<" -> " << "\"" << it->second->backLanes[i]->to->junction_id << "\"" << " [style=bold, color=orange, label=\""
                    << "backlane " << it->second->backLanes[i]->lane_id << " (" << it->second->backLanes[i]->length << ") " << "\"];\n";
            }
        }*/
    }


    for(size_t i = 0; i < prev.size(); i++){
        for(map<string,junction*>::iterator it = junction_map.begin(); it != junction_map.end(); ++it){
            if (int(i) == it->second->id && prev[i] != NULL){
                graph_file << "\"" << it->second->junction_id << "\"" <<" -> " << "\"" << prev[i]->junction_id << "\"" << " [style=bold, color=green];\n";
            }
        }
    }

    junction *curr_junction = dest;
    while(curr_junction != NULL){
    	if(prev[curr_junction->id] != NULL){
    		graph_file << "\"" << curr_junction->junction_id << "\"" <<" -> " << "\"" << prev[curr_junction->id]->junction_id << "\"" << " [style=bold, color=red];\n";
    	}
    	curr_junction = prev[curr_junction->id];
    }
    
    graph_file << "}";

    graph_file.close();

    FILE *pipe = popen( "xdot graph &", "w" );
    fclose( pipe );
}

//======================================================================

string getFile( string filename ){
    string buffer;
    char c;

    ifstream in( filename );   
    if ( !in ) { 
   	    cout << filename << " not found" << '\n';   
   	    exit( 1 ); 
    }
    while ( in.get( c ) ) buffer += c;
    in.close();

    return (buffer);
}


//======================================================================

vector<string> getData( const string &text, string tag ){                                                          
    vector<string> collection;
    size_t pos = 0, start;

    while ( true ){
        string tag_start = "<" + tag;
        start = text.find(tag_start , pos );   if ( start == string::npos ) return collection;
        start += tag_start.size();
        pos = text.find( ">" , start );
      
        if(pos != string::npos && int(pos) - 1 > 0 && int(pos) - 1 < int(text.size()) && text[int(pos) - 1] == '/') pos--;
        if(start < text.size() && text[start] == ' ') start++;
  
        if ( pos == string::npos ) return collection;
        collection.push_back( text.substr( start, pos - start ) );
    }
}


//======================================================================
 
