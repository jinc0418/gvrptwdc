/************************************************************************************************
Copyright:
Author:jincheng
Date:2018-8-30
Descriptions: 
the project is designed to solve the large scale G-VRPTWDC using R&R combined with LS.
this file includes declaration of all classes and some normal 
functions as well as all the necessary libs
************************************************************************************************/
#pragma once
#include<cstdio>
#include<fstream>
#include <iostream>
#include <ctime>
#include <vector>
#include <map>
#include <unordered_map>
#include <list>
#include <limits.h>
#include <algorithm>
#include<string>
#include<sstream>
#include<stdio.h>
#include<set>
#include<cstring>
#include<iterator>
#include <iomanip>

using namespace std;

vector<std::string> split(const  std::string& s, const std::string& delim);

/*includes all the necessary infomation from original data*/
class Data {
public:
	//information are rolled in here
	Data(int cluster_num);
	Data();
	int total_num;//say 1601
	int cus_num;//say 1500
	int ** dists_time;
	int ** dists;
	int demand_tw0[2000];
	int demand_tw1[2000];
	double demand_m[2000];
	double demand_v[2000];
	double pointx[2000];
	double pointy[2000];
	~Data();
};

/*obtain the inital solution rt*/
class Initial {
public:
	Data data;
	int service = 30;
	double wait_per_min = 0.4;
	int length;
	double capacity_m;
	double capacity_v;
	double cost_per_meter;
	int fixed;
	int veichle_type;
	int cluster_num;

	//for vehicle type one
	set<int> nodes_unexplored;
	vector<vector<int>>routes;

	//for vechicle type two
	set<int> nodes_unexplored2;
	vector<vector<int>>routes2;

	//Sort all the nodes in acending order refer to its distance from depot
	vector <int> dist2deport_order;

	Initial(Data &data, int cluster);
	void reset_vtype(int type);
	bool load_m_feasiable(vector<int> &path);
	bool load_v_feasiable(vector<int> &path);
	bool time_window_feasiable(vector<int> &path);
	bool distance_feasiable(vector<int> &path);
	bool feasiable(vector<int> &path, vector<int> &path1);

	//put all customs into two types,normal one or the one must use veichle type2
	void compute_nodes_unexplored_type();

	//get dist2deport_order
	void compute_dists2deport_order();
	void return_routes(set<int>&nodes);
	void initial_routes();
};

/* a struct includes the index of a node in a route,the node,
and the distance cost of a remove or insert operation */
struct inital_increase_dists
{
public:
	int index;
	int node;
	int dist_cost;
};

/* a struct includes the distance cost of a remove or
insert operation of the charge stations */
struct inital_increase_station_dists
{
public:
	int dist_cost;
	int station;
};

/* a struct includes the route index,the index of a node in a route,
the node,and the distance cost of a remove or insert operation */
struct Station_Remove
{
public:
	int route_index;
	int node_index;
	int node;
	int dist_cost;
};

/* a struct includes the route index,the index of a node in a route,and the node */
struct LS
{
public:
	int route_index;
	int node_index;
	int node;
};

struct LS_TIME
{
public:
	int route_index;
	int node_index;
	int node;
	int arr_time;
};

class ROUTE1
{
public:
	int veichle_type;
	vector<int> path;
	ROUTE1(vector<int>&route, int vtype);
};
class ROUTE
{
public:
	int veichle_type;
	vector <int> path;//always feasiable
	vector <int> changed_path;
	bool changed = false;
	bool remove_flag = false;
	vector<int> path_time;
	vector<int> changed_path_time;

	ROUTE(vector<int>route);
	ROUTE(ROUTE1 &route);
	ROUTE(ROUTE &route,int i);
	ROUTE(ROUTE &route, int i ,int j);
	ROUTE();
};

/*add noise to the R&R alogrithms*/
class Noise_Maker {
public:
	double noise_prob = 0.1;
	double noise_level = 0.15;
	static int max_dist;
	int make_noise();
};

/*includes all the operators of the ruin&recreate as well as local search */
class R_R
{
public:
	Data data;
	set<int> nodes_for_vehicle2;
	int service = 30;
	double wait_per_min = 0.4;
	int length = 100000;
	double capacity_m = 2.0;
	double capacity_v = 12.0;
	double cost_per_meter = 0.012;
	int charge_cost_per = 50;
	int fixed = 200;
	int cluster_num;
	/*keep the temperate solution and the best solution of the problem*/
	vector <ROUTE> solution;
	vector <ROUTE> best_solution;
	double best_cost;
	double temp_cost;
	int sigma = 40;//max station to move
	int omega = 12;//max routes to move
	int beta = 50;//max cus to move
	set<int> nodes_unexplored;//always keeps the nodes unexplored of the current solution
	vector <int> dist2deport_order;//same as initial class,sadly proved to be useless
	vector <vector<int>> neighbours;//compute the neighbors of all customs

	//************memembers and functions boundary line**************************//

	R_R(Data &data, int cluster, Initial initial);

	//if the temperate solutoin is better,then assign it to the best solution
	void updata_best_solu();

	/* used in LS ,the difference between reset_best_solu is the route format */
	void reset_best_solu();

	/*this function was used at the end which assign the best solution
	to the current solution for output convenience */
	void reset_best_solution();

	/* return the temperate solution to its origin state*/
	void reset_solution();

	bool load_m_feasiable(vector<int> &path);
	bool load_v_feasiable(vector<int> &path);
	bool time_window_feasiable(vector<int> &path);
	bool distance_feasiable(vector<int> &path);
	bool feasiable(vector<int> &path,int type);
	int compute_dists(vector<int> &path);
	double compute_cost(vector<int> &path, int vtype);

	/*compute the costs of temperate solution refer to changed path*/
	double compute_all_costs();

	/*compute the costs of temperate solution refer to path*/
	double compute_all_costs1();

	//exchange one point randomly from two routes
	void inter_tour_exchange(int iterations, int penalty = 0);

	/*exchange one point randomly from two routes,the difference between up is that
	first it transverse all the routes ,then randomly choose another route*/
	void inter_tour_exchange1(int iterations, int penalty = 0);

	/*same as inter_tour_exchange1 except for exchange two successful nodes */
	void inter_tour_exchange_two_points(int iterations, int penalty = 0);

	/*exchange one point with the 500 nearest points of a randomly chosen point*/
	void inter_tour_exchange_nearest100(int iterations, int penalty = 0);

	void inter_tour_exchange_nearest_all (int penalty = 0);
	void inter_tour_exchange_time_related(int penalty = 0);

	/*exchange two points of a randomly chosen route,the distance is non_exact*/
	void within_tour_exchange(int iterations,int penalty = 0);

	/*same as within_tour_exchange except for first transverse all the routes,
	the distance is non_exact */
	void within_tour_exchange1(int penalty = 0);

	/* actual distance used*/
	void within_tour_exchange2(int penalty = 0);

	/* unused which was copied from TSPTW*/
	void within_tour_opt();
	
	/* remove and insert charge stations: unused */
	void sr(int choice);
	bool si(int choice);

	/*remove and insert all the charge stations*/
	void si_all();
	void sr_all();
	bool si_route(int choice, ROUTE &route);// used within si_all

	//used in cr in order that a route is infeasiable after remove operation
	bool modify_si(vector<int> &path,int type);

	/* remove routes :unused */
	void rr(int choice);

	/* remove customs:random remove,worst remove and cluster remove of which the last is best */
	void cr(int choice);

	/* reomve and insert customs point by point choice0 means greedy while 1 means the second best*/
	void cr_point(int choice);
	bool ci_point(int node,double cost_down,ROUTE & item,int route_index,int choice);

	/* insert customs unexplored,choice 0 means greedy insert, choice 2 means regret2 while choice 3 regret 3 */
	void ci(int choice);
	//used within ci in special cases 
	void ci_post();

	/* the entry function of R_R includes all the parameters */
	void run_r_r();

	/* greedy create routes for the unexplored nodes when none can be inserted
	in exsiting routes from scratch using the same way as inital solution */
	vector<ROUTE1> create_routes();
	bool feasiable_initial(ROUTE1 &path, ROUTE1 &path1);

	/* same as initial */
	void compute_dists2deport_order();
	void compute_neighbours();

	/* kmeans method*/
	vector<double> cluster_center(vector<int> & route);
	vector <int> kmeans(vector<int> routes);

	/* for the final result output purpose*/
	bool print_feasiable(vector<int>&path, int temp_time);
	string get_time_back(int time);

	void charge_dispose();
};