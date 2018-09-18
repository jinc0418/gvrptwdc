/************************************************************************************************
Copyright:
Author:jincheng
Date:2018-8-30
Descriptions:
the project is designed to solve the large scale G-VRPTWDC using R&R combined with LS.
this file is the implemention of the inital solution which return the intial routes in a vector 
form with a veichle type flag at the end of each route
************************************************************************************************/
#include"header.h"
#include"template.cpp"

Initial::Initial(Data &data1,int cluster)
{
	data = data1;
	cluster_num = cluster;
}

void Initial::reset_vtype(int type)
{
	if (type == 1)
	{
		veichle_type = 1;
		capacity_m = 2.0;
		capacity_v = 12.0;
		length = 100000;
		cost_per_meter = 0.012;
		fixed = 200;
	}
	else if (type == 2)
	{
		veichle_type = 2;
		capacity_m = 2.5;
		capacity_v = 16.0;
		length = 120000;
		cost_per_meter = 0.014;
		fixed = 300;
	}
	else
		cout << "error type!";
}

/* refer to : "Symmetry helps: Bounded bi-directional dynamic programming for
the elementary shortest path problem with resource constraints" saying that the 
total resource of pick up ablility is always non_weaker than the delivery thus 
every point delivery assumption is non_less than pick up*/
bool Initial::load_m_feasiable(vector<int> &path)
{
	double m_p = 0, m_d = 0, m_d_temp = 0;
	for (auto i : path)
	{
		if (i > data.cus_num - 200 && i <= data.cus_num)
		{
			m_p += data.demand_m[i];
		}
		else if (i <= data.cus_num - 200)
		{
			m_d_temp = m_d + data.demand_m[i];
		}
		m_d = (m_p > m_d_temp ? m_p : m_d_temp);
		m_d_temp = m_d;
	}
	if (m_d > capacity_m || m_p > capacity_m)
	{
		return false;
	}
	return true;
}
bool Initial::load_v_feasiable(vector<int> &path)
{
	double v_p = 0, v_d = 0, v_d_temp = 0;
	for (auto i : path)
	{
		if (i > data.cus_num - 200 && i <= data.cus_num)
		{
			v_p += data.demand_v[i];
		}
		else if (i <= data.cus_num - 200)
		{
			v_d_temp = v_d + data.demand_v[i];
		}
		v_d = (v_p > v_d_temp ? v_p : v_d_temp);
		v_d_temp = v_d;
	}
	if (v_d > capacity_v || v_p > capacity_v)
	{
		return false;
	}
	return true;
}

/* to judge whether a route satify the time window demand */
bool Initial::time_window_feasiable(vector<int> &path)
{
	int temp = 450;
	int a, b;
	for (size_t i = 0; i < path.size()-1; i++)
	{
		a = temp + service + data.dists_time[path[i]][path[i + 1]];
		b = data.demand_tw0[path[i + 1]];
		temp = (a > b ? a : b);
		if (temp > data.demand_tw1[path[i + 1]])
		{
			return false;
		}
	}
	return true;
}

/* whether a route satify the distance demand*/
bool Initial::distance_feasiable(vector<int> &path)
{
	int temp_dist = 0;
	for (size_t i = 0; i < path.size() - 1; i++)
	{
		temp_dist += data.dists[path[i]][path[i + 1]];
		if (temp_dist > length)
		{
			return false;
		}
		if (path[i + 1] > data.cus_num)
		{
			temp_dist = 0;
		}
	}
	return true;
}

/* if the route is feasiable return true ,otherwise try to insert stations to modify if cannot return false */
bool Initial::feasiable(vector<int> &path, vector<int> &path1)
{
	if ((!time_window_feasiable(path)) || (!load_m_feasiable(path)) || (!load_v_feasiable(path)))
	{
		return false;
	}
	if (distance_feasiable(path))
	{
		path1 = path;
		return true;
	}
	int temp_dist = 0;
	for (size_t i = 0; i < path.size(); i++)
	{
		int node = path[i];
		int node_next = path[i + 1];
		temp_dist += data.dists[node][node_next];
		if (temp_dist > length)
		{
			vector<inital_increase_station_dists> increase_station_dist;
			for (int station = data.cus_num+1; station < data.total_num; station++)
				increase_station_dist.push_back(inital_increase_station_dists{ data.dists[node][station] + data.dists[station][node_next] - data.dists[node][node_next],station });
			sort(increase_station_dist.begin(), increase_station_dist.end(), [&](inital_increase_station_dists i1, inital_increase_station_dists i2) {return i1.dist_cost < i2.dist_cost; });
			for (auto item : increase_station_dist)
			{
				vector<int>route_juge_temp = path;
				route_juge_temp.insert(route_juge_temp.begin() + i + 1, item.station);
				if (time_window_feasiable(route_juge_temp) && distance_feasiable(route_juge_temp))
				{
					path1 = route_juge_temp;
					return true;
				}
			}
			return false;
		}
		if (node_next > data.cus_num)
		{
			temp_dist = 0;
		}
	}
	return false;
}

/* put all the customs into two bags,normal one or the one must use veichle type2 */
void Initial::compute_nodes_unexplored_type()
{
	for (int i = 1; i <= data.cus_num; i++)
	{
		if (data.demand_m[i] > capacity_m || data.demand_v[i] > capacity_v)
		{
			nodes_unexplored2.insert(i);
		}
		else
		{
			nodes_unexplored.insert(i);
		}
	}
}

/* Sort all the nodes in acending order refer to its distance from depot
since the tranigle inequality does not meet,the distance and back is used*/
void Initial::compute_dists2deport_order()
{
	vector<int> dists0;
	for (int i = 0; i < data.total_num; i++)
	{
		dists0.push_back(data.dists[0][i]+data.dists[i][0]);
	}
	dist2deport_order = argsort<int>(dists0);
}

/* return the minimum index of a structor */
int min_dists(vector<inital_increase_dists> &increase_station_dist)
{
	int best = 0;
	for (size_t i=0;i<increase_station_dist.size();i++)
	{
		if (increase_station_dist[i].dist_cost < increase_station_dist[best].dist_cost)
		{
			best = i;
		}
	}
	return best;
}

/* generate initial solutions:every time a route was generated greedy as long as possible.
The first node was the unexplored one closest to the depot, then insert nodes to the route according
to the minimum cost increased if distance infeasiable then modify it by inserting charge stations.
A new route was created unless no customs can be inserted in the current route*/
void Initial::return_routes(set<int>&nodes)
{
	// judge whether a new route is needed
	bool same_route_flag = false;
	while (nodes.size()>0)
	{
		vector<int> route{ 0 };
		for (auto first_node : dist2deport_order)
		{
			if (nodes.find(first_node) != nodes.end())
			{
				route.push_back(first_node);
				break;
			}
		}
		same_route_flag = true;
		while (same_route_flag)
		{
			same_route_flag = false;
			vector<inital_increase_dists> increase_dist_lists;
			for (size_t i = 0; i < route.size(); i++)
			{
				for (int node : nodes)
				{
					int dists_cost;
					if (i == route.size() - 1)
						dists_cost = data.dists[route[i]][node] + data.dists[node][0] - data.dists[route[i]][0];
					else
						dists_cost = data.dists[route[i]][node] + data.dists[node][route[i + 1]] - data.dists[route[i]][route[i + 1]];
					increase_dist_lists.push_back(inital_increase_dists{ int(i), node, dists_cost });//no need to judge
				}
			}
			sort(increase_dist_lists.begin(), increase_dist_lists.end(), [](const inital_increase_dists &i1,const inital_increase_dists &i2) {return i1.dist_cost < i2.dist_cost; });
			for (auto item : increase_dist_lists)
			{
				if (item_in_vectors<int>(item.node, route))
					continue;
				vector<int> route_judge = route;

				route_judge.insert(route_judge.begin()+item.index + 1, item.node);
				route_judge.push_back(0);
				
				if (feasiable(route_judge, route))
				{
					route.pop_back();
					same_route_flag = true;
					break;
				}
			}
		}

		/* due to some unknown reasons, successful charge stations may occur at the end of a route,so it was disposed that 
		all the same successful stations are removed thus only one best is retained */
		int count = 0;
		int len = route.size();
		for (size_t i = 0; i < len; i++)
		{
			if (route[len - i - 1] <= data.cus_num)
			{
				break;
			}
			count++;
		}
		for (int i = 0; i < count; i++)
		{
			route.pop_back();
		}
		size_t route_len = route.size();
		int temp_dist_best = 9999999, temp_dist = 9999999;
		vector<int> route1 = route;
		for (int station = data.cus_num + 1; station < data.total_num; station++)
		{
			vector<int> temp1 = route1;
			temp1.push_back(station);
			temp1.push_back(0);
			temp_dist = data.dists[route[route_len-1]][station] + data.dists[station][0];
			if (temp_dist < temp_dist_best && distance_feasiable(temp1))
			{
				temp_dist_best = temp_dist;
				route = temp1;
				route.pop_back();
			}
		}
		route.push_back(0);

		/* to dispose a special case in which 0 i 0 infeasiable */
		if (route.size() == 3 && data.dists[0][route[1]] + data.dists[route[1]][0] > length)
		{
			vector<inital_increase_dists> increase_station_dist;
			for (int station = data.cus_num + 1; station < data.total_num; station++)
			{
				vector<int> temp_route = route;
				vector<int> temp_route1 = route;
				temp_route.insert(temp_route.begin() + 1, station);
				if (data.dists[station][route[1]] + data.dists[route[1]][0] <= length && time_window_feasiable(temp_route))
				{
					increase_station_dist.push_back(inital_increase_dists{1,station,data.dists[0][station] + data.dists[station][route[1]] - data.dists[0][route[1]]});
				}
				temp_route1.insert(temp_route1.begin() + 2, station);
				if (data.dists[0][route[1]] + data.dists[route[1]][station] <= length && time_window_feasiable(temp_route1))
				{
					increase_station_dist.push_back(inital_increase_dists{ 2,station,data.dists[route[1]][station] + data.dists[station][0] - data.dists[route[1]][0] });
				}
			}
			if (increase_station_dist.size() > 0)
			{
				auto temp = increase_station_dist[min_dists(increase_station_dist)];
				route.insert(temp.index+route.begin(), temp.node);
			}
			else
			{
				cout << "initial error!";
				system("pause");
			}
		}

		/* put 10000 at the end if a route is type one ,20000 otherwise.
		routes contains all the type one nodes while routes2 contains type2 nodes*/
		route.push_back(veichle_type * 10000);
		if (veichle_type == 1)
		{
			routes.push_back(route);
		}
		else if(veichle_type==2)
		{
			routes2.push_back(route);
		}
		else
		{
			cout << "veichle type error!" << endl;
		}

		/* when a route is generated ,update the exploration lists */
		set<int> route_set;
		vector<int> route_vec;
		for (auto item : route)
		{
			route_set.insert(item);
		}
		set_difference(nodes.begin(), nodes.end(), route_set.begin(), route_set.end(), back_inserter(route_vec));
		nodes.clear();
		for (auto item : route_vec)
		{
			nodes.insert(item);
		}
	}
}

/* here we put the two types of customs into two groups and dispose it seperately */
void Initial::initial_routes()
{
	reset_vtype(1);
	compute_nodes_unexplored_type();
	compute_dists2deport_order();
	return_routes(nodes_unexplored);
	reset_vtype(2);
	return_routes(nodes_unexplored2);
	
}