/************************************************************************************************
Copyright:
Author:jincheng
Date:2018-8-30
Descriptions:
The project is designed to solve the large scale G-VRPTWDC using R&R combined with LS.
This file includes all the functions of R_R operators and some other nessceary functions
************************************************************************************************/
#include"header.h"
#include"template.cpp"
using namespace std;

/*add noise to the R&R alogrithms
with noise_level 0.15 noise probability 0.1 as defalut*/
int Noise_Maker::make_noise()
{
	if (rand() % 100 / (double)101 < noise_prob)
	{
		return noise_level * max_dist * (rand() % 100 / (double)101);
	}
	else
		return 0;
}

/* compute Euclid distance of a certain two points for the purpose of cluser remove */
double compute2points_dists(double x1, double y1, double x2, double y2)
{
	return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

/* return the geometric center of a route  */
vector<double> R_R::cluster_center(vector<int> & route)
{
	vector<double> center;
	double x = 0, y = 0;
	size_t num = route.size();
	for (auto &item : route)
	{
		x += data.pointx[item];
		y += data.pointy[item];
	}
	center.push_back(x / num);
	center.push_back(y / num);
	return center;
}

/* return the maxmium index of a structor */
int max_dists(vector<Station_Remove> &increase_station_dist)
{
	int best = 0;
	for (size_t i = 0; i<increase_station_dist.size(); i++)
	{
		if (increase_station_dist[i].dist_cost > increase_station_dist[best].dist_cost)
		{
			best = int(i);
		}
	}
	return best;
}

/* return the minmium index of a structor */
int min_dists(vector <Station_Remove> &cus_dist_lists)
{
	int best = 0;
	for (size_t i = 0; i<cus_dist_lists.size(); i++)
	{
		if (cus_dist_lists[i].dist_cost < cus_dist_lists[best].dist_cost)
		{
			best = int(i);
		}
	}
	return best;
}

/* Sort all the nodes in acending order refer to its distance from depot multiplied by the coffecience 
of the veichle type  since the tranigle inequality does not meet,the distance and back is used*/
void R_R::compute_dists2deport_order()
{
	vector<int> dists0;
	for (int i = 0; i < data.total_num; i++)
	{
	dists0.push_back(data.dists[0][i] + data.dists[i][0]);
	}
	dist2deport_order = argsort<int>(dists0);
	/*for (int i = 0; i < data.total_num; i++)
	{
		int coef = 12;
		if (nodes_for_vehicle2.find(i) != nodes_for_vehicle2.end())
		{
			coef = 14;
		}
		dists0.push_back((data.dists[0][i] + data.dists[i][0])*coef);
	}*/
	dist2deport_order = argsort<int>(dists0);

}

/* compute the neighbours of all nodes for cluster reomve */
void R_R::compute_neighbours()
{
	for (size_t k = 0; k < data.total_num; k++)
	{
		vector<int> dists0;
		for (int i = 0; i < data.total_num; i++)
		{
			dists0.push_back(data.dists[k][i] + data.dists[i][k]);
		}
		neighbours.push_back(argsort<int>(dists0));
	}
}

/* the main body of kmeans_2 in order to seperate a route into two parts 
and randomly return  half part of a route to remove */
vector<int> R_R::kmeans(vector<int> route)
{
	/* first remove the zeros and assume the route length excluding depot larger than 2
	if only one point remained, then return it ,if two points remained,then randomly return
	one of them, otherwise run kmeans algrithm*/
	route.pop_back();
	route.erase(route.begin());
	if (route.size() == 1)
	{
		return vector<int>{route[0]};
	}
	if (route.size() == 2)
	{
		if (rand() % 100 / (double)101 < 0.5)
			return vector<int>{route[0]};
		else
			return vector<int>{route[1]};
	}

	/* chose the initial center of two clusters*/
	double cluster1x = data.pointx[route[0]], cluster1y = data.pointy[route[0]];
	double cluster2x = data.pointx[route[1]], cluster2y = data.pointy[route[1]];
	vector<int> part1;
	vector<int> part2;
	vector<int> part1_old, part2_old;
	int test = 0;

	/* when the two parts keeps unchanged or the iteration reach max_iter,stops */
	int max_iter = 10;
	for (size_t i = 0; i < max_iter; i++)
	{
		part1_old = part1;
		part2_old = part2;
		part1.clear();
		part2.clear();
		for (int item : route)
		{
			if (compute2points_dists(data.pointx[item], data.pointy[item], cluster1x, cluster1y)
				<compute2points_dists(data.pointx[item], data.pointy[item], cluster2x, cluster2y)) {
				part1.push_back(item);
			}
			else {
				part2.push_back(item);
			}
		}
		if (part1 == part1_old) {
			break;
		}
		vector<double> temp = cluster_center(part1);
		cluster1x = temp[0];
		cluster1y = temp[1];
		vector<double> temp1 = cluster_center(part2);
		cluster2x = temp1[0];
		cluster2y = temp1[1];

	}

	/* randomly return a part of the route */
	if (rand() % 100 / (double)101 < 0.5)
		return part1;
	else
		return part2;
}

/* used when new routes must be created:if a route is distance infeasiable,insert charge station to modify */
bool R_R::feasiable_initial(ROUTE1 &path, ROUTE1 &path1)
{
	if (path.veichle_type == 1)
	{
		capacity_m = 2.0; capacity_v = 12.0; length = 100000;
	}
	else if (path.veichle_type == 2)
	{
		capacity_m = 2.5; capacity_v = 16.0; length = 120000;
	}
	if ((!time_window_feasiable(path.path)) || (!load_m_feasiable(path.path)) || (!load_v_feasiable(path.path)))
	{
		return false;
	}
	if (distance_feasiable(path.path))
	{
		path1.path = path.path;
		return true;
	}
	int temp_dist = 0;
	for (size_t i = 0; i < path.path.size(); i++)
	{
		int node = path.path[i];
		int node_next = path.path[i + 1];
		temp_dist += data.dists[node][node_next];
		if (temp_dist > length)
		{
			vector<inital_increase_station_dists> increase_station_dist;
			for (int station = data.cus_num + 1; station < data.total_num; station++)
				increase_station_dist.push_back(inital_increase_station_dists{ data.dists[node][station] + data.dists[station][node_next] - data.dists[node][node_next],station });
			sort(increase_station_dist.begin(), increase_station_dist.end(), [&](inital_increase_station_dists i1, inital_increase_station_dists i2) {return i1.dist_cost < i2.dist_cost; });
			for (auto &item : increase_station_dist)
			{
				vector<int>route_juge_temp = path.path;
				route_juge_temp.insert(route_juge_temp.begin() + i + 1, item.station);
				if (time_window_feasiable(route_juge_temp) && distance_feasiable(route_juge_temp))
				{
					path1.path = route_juge_temp;
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
int min_dists_initial(vector<inital_increase_dists> &increase_station_dist)
{
	int best = 0;
	for (size_t i = 0; i<increase_station_dist.size(); i++)
	{
		if (increase_station_dist[i].dist_cost < increase_station_dist[best].dist_cost)
		{
			best = i;
		}
	}
	return best;
}

/* generate solution from initial results,compute dists2depot order and neighbors as well*/
R_R::R_R(Data &data1, int cluster, Initial initial)
{
	data = data1;
	cluster_num = cluster;
	for (auto &item : initial.routes)
	{
		solution.push_back(ROUTE(item));
	}

	/* reload constructor ROUTE(vector) */
	for (auto &item : initial.routes2)
	{
		solution.push_back(item);
	}
	for (int i = 1; i <= data.cus_num; i++)
	{
		if (data.demand_m[i] > capacity_m || data.demand_v[i] > capacity_v)
		{
			nodes_for_vehicle2.insert(i);
		}
	}
	compute_dists2deport_order();
	compute_neighbours();
}

/* if a route cannot be repaired,return the solution to its origin state */
void R_R::reset_solution()
{
	for (auto &item : solution)
	{
		//item.changed_path = item.path;
		//item.changed = false;
		if (item.changed)
		{
			item.changed_path = item.path;
			item.changed = false;
		}
		item.remove_flag = false;
	}
	nodes_unexplored.clear();
}

/* same as inital */
bool R_R::load_m_feasiable(vector<int> &path)
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
bool R_R::load_v_feasiable(vector<int> &path)
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
bool R_R::time_window_feasiable(vector<int> &path)
{
	int temp = 450;
	int a, b;
	for (size_t i = 0; i < path.size() - 1; i++)
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
bool R_R::distance_feasiable(vector<int> &path)
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
bool R_R::feasiable(vector<int> &path, int type)
{
	if (type == 1)
	{
		capacity_m = 2.0; capacity_v = 12.0; length = 100000;
	}
	else if (type == 2)
	{
		capacity_m = 2.5; capacity_v = 16.0; length = 120000;
	}
	if ((!time_window_feasiable(path)) || (!load_m_feasiable(path)) || (!load_v_feasiable(path)) || (!distance_feasiable(path)))
	{
		return false;
	}
	return true;
}

/* compute the total dists of a route: unused */
int R_R::compute_dists(vector<int> &path)
{
	int temp_dist = 0;
	for (size_t i = 0; i < path.size() - 1; i++)
	{
		temp_dist += data.dists[path[i]][path[i + 1]];
	}
	return temp_dist;
}

/* compute the real cost of a certain route*/
double R_R::compute_cost(vector<int> &path, int vtype)
{
	if (vtype == 1)
	{
		cost_per_meter = 0.012; fixed = 200;
	}
	else if (vtype == 2)
	{
		cost_per_meter = 0.014; fixed = 300;
	}
	int temp_dist = 0;
	int temp_time = 450;
	int wait_time = 0;
	int count = 0;
	for (size_t i = 0; i < path.size() - 1; i++)
	{
		if (path[i] > data.cus_num)
			count++;
		temp_dist += data.dists[path[i]][path[i + 1]];
		int temp = data.dists_time[path[i]][path[i + 1]] + service + temp_time;
		if (temp < data.demand_tw0[path[i + 1]])
		{
			temp_time = data.demand_tw0[path[i + 1]];
			wait_time += data.demand_tw0[path[i + 1]] - temp;
		}
		else
			temp_time = temp;
	}
	return (temp_dist*cost_per_meter + fixed + count*charge_cost_per + wait_time*wait_per_min);
}

/* compute the total solution costs refer to changed path*/
double R_R::compute_all_costs()
{
	double total_costs = 0;
	for (size_t i=0;i<solution.size();i++)
	{
		auto & item = solution[i];
		double temp = compute_cost(item.changed_path, item.veichle_type);
		if (temp < 0)
		{
			print_vec(item.changed_path);
			cout << i;
			system("pause");
		}
		total_costs += temp;
	}
	return total_costs;
}

/*compute the total solution costs refer to path*/
double R_R::compute_all_costs1()
{
	double total_costs = 0;
	for (auto &item : solution)
	{
		total_costs += compute_cost(item.path, item.veichle_type);
	}
	return total_costs;
}

/* remove a number of charge stations,but unused here
      0:random,1:worst distance remove*/
void R_R::sr(int choice)
{
	int num2move = rand() % sigma;
	if (choice == 0)
	{
		random_shuffle(solution.begin(), solution.end());
		for (ROUTE &route : solution)
		{
			if (num2move < 1)
			{
				break;
			}
			for (size_t index = 0; index<route.path.size() - 1; index++)
			{
				if (route.path[index] > data.cus_num && route.changed_path[index] != 0)
				{
					route.changed_path.erase(route.changed_path.begin() + index);
					route.changed = true;
					num2move--;
					break;
				}
			}
		}
	}
	else if (choice == 1)
	{
		for (int i = 0; i < num2move; i++)
		{
			vector <Station_Remove> station_remove;
			for (size_t route_index = 0; route_index < solution.size(); route_index++)
			{
				for (size_t node_index = 0; node_index < solution[route_index].changed_path.size() - 1; node_index++)
				{
					int node = solution[route_index].changed_path[node_index];
					if (node > data.cus_num)
					{
						int next_node = solution[route_index].changed_path[node_index + 1];
						int pre_node = solution[route_index].changed_path[node_index - 1];
						int dist = data.dists[pre_node][node] + data.dists[node][next_node] - data.dists[pre_node][next_node];
						station_remove.push_back(Station_Remove{ int(route_index),int(node_index),node,dist });
					}
				}
			}
			if (station_remove.size() > 0)
			{
				Station_Remove item = station_remove[max_dists(station_remove)];
				int index = item.route_index;
				int index2 = item.node_index;
				solution[index].changed_path.erase(solution[index].changed_path.begin() + index2);
				solution[index].changed = true;
			}
		}
	}
	else
	{
		cout << "error sr choice!" << endl;
		system("pause");
	}
}

/* remove all the charge stations  */
void R_R::sr_all()
{
	random_shuffle(solution.begin(), solution.end());
	for (ROUTE &route : solution)
	{
		for (size_t index = 1; index<route.changed_path.size() - 1; index++)
		{
			if (route.changed_path[index] > data.cus_num)
			{
				route.changed_path.erase(route.changed_path.begin() + index);
				route.changed = true;
				index--;
			}
		}
	}
}

/* insert charge stations for all routes if needed:used after sr_all */
void R_R::si_all()//del every one and only one station in a  route if it has
{
	for (auto &route : solution)
	{
		if (route.veichle_type == 1)
		{
			capacity_m = 2.0; capacity_v = 12.0; length = 100000;
		}
		else if (route.veichle_type == 2)
		{
			capacity_m = 2.5; capacity_v = 16.0; length = 120000;
		}
		if (!si_route(0, route))
		{
			cout << "all stations remove failed!" << endl;
			print_vec(route.path);
			print_vec(route.changed_path);
			route.changed_path = route.path;
			route.changed = false;
		}
	}
	for (auto &item : solution)
	{
		item.changed = false;
		item.path = item.changed_path;
		item.remove_flag = false;
	}
}

/* insert station to a certain route,return false if the route is infeasiable or cannot be repaired
   0:best insert 1:greedy insert 有木有可能删掉点后反而time window feasiable*/
bool R_R::si_route(int choice, ROUTE &route)
{
	if (choice == 0)
	{
		if (!time_window_feasiable(route.changed_path))
		{
			return false;
		}
		if (!distance_feasiable(route.changed_path))
		{
			vector<inital_increase_dists> increase_station_dist;
			for (size_t i = 1; i < route.changed_path.size(); i++)
			{
				for (int station = data.cus_num + 1; station < data.total_num; station++)
				{
					increase_station_dist.push_back(inital_increase_dists{ int(i),station, data.dists[route.changed_path[i - 1]][station] + data.dists[station][route.changed_path[i]] - data.dists[route.changed_path[i - 1]][route.changed_path[i]] });
				}
			}
			sort(increase_station_dist.begin(), increase_station_dist.end(), [&](inital_increase_dists i1, inital_increase_dists i2) {return i1.dist_cost < i2.dist_cost; });
			for (auto &item : increase_station_dist)
			{
				vector<int>route_juge_temp = route.changed_path;
				route_juge_temp.insert(route_juge_temp.begin() + item.index, item.node);
				if (time_window_feasiable(route_juge_temp) && distance_feasiable(route_juge_temp))
				{
					route.changed_path = route_juge_temp;
					return true;
				}
			}
			return false;
		}
		return true;
	}
	else if (choice == 1)
	{
		bool route_flag = false;
		if (!distance_feasiable(route.changed_path))
		{
			int temp_dist = 0;
			for (size_t i = 0; i < route.changed_path.size() - 1; i++)
			{
				int node = route.changed_path[i];
				int node_next = route.changed_path[i + 1];
				temp_dist += data.dists[node][node_next];
				if (temp_dist > length)
				{
					for (size_t insert_pos = 0; insert_pos <= i; insert_pos++)
					{
						vector<inital_increase_station_dists> increase_station_dist;
						for (int station = data.cus_num + 1; station < data.total_num; station++)
							increase_station_dist.push_back(inital_increase_station_dists{ data.dists[node][station] + data.dists[station][node_next] - data.dists[node][node_next],station });
						sort(increase_station_dist.begin(), increase_station_dist.end(), [&](inital_increase_station_dists i1, inital_increase_station_dists i2) {return i1.dist_cost < i2.dist_cost; });
						for (auto &item : increase_station_dist)
						{
							vector<int>route_juge_temp = route.changed_path;
							route_juge_temp.insert(route_juge_temp.begin() + i + 1 - insert_pos, item.station);
							if (time_window_feasiable(route_juge_temp) && distance_feasiable(route_juge_temp))
							{
								route.changed_path = route_juge_temp;
								route_flag = true;
								break;
							}
						}
						if (route_flag)
							break;
					}
				}
				if (route_flag)
					break;
				if (route.changed_path[i + 1] > data.cus_num)
					temp_dist = 0;
			}
			if (!route_flag)
				return false;
		}
		return true;
	}
}

/* greedy insert to repair routes */
bool R_R::modify_si(vector<int> &route, int type)
{
	if (type == 1)
	{
		capacity_m = 2.0; capacity_v = 12.0; length = 100000;
	}
	else if (type == 2)
	{
		capacity_m = 2.5; capacity_v = 16.0; length = 120000;
	}
	if (!time_window_feasiable(route) || !load_m_feasiable(route) || !load_v_feasiable(route))
		return false;
	if (!distance_feasiable(route))
	{
		int temp_dist = 0;
		for (size_t i = 0; i < route.size() - 1; i++)
		{
			int node = route[i];
			int node_next = route[i + 1];
			temp_dist += data.dists[node][node_next];
			if (temp_dist > length)
			{
				for (size_t insert_pos = 0; insert_pos <= i; insert_pos++)
				{
					vector<inital_increase_station_dists> increase_station_dist;
					for (int station = data.cus_num + 1; station < data.total_num; station++)
						increase_station_dist.push_back(inital_increase_station_dists{ data.dists[node][station] + data.dists[station][node_next] - data.dists[node][node_next],station });
					sort(increase_station_dist.begin(), increase_station_dist.end(), [&](inital_increase_station_dists i1, inital_increase_station_dists i2) {return i1.dist_cost < i2.dist_cost; });
					for (auto &item : increase_station_dist)
					{
						vector<int>route_juge_temp = route;
						route_juge_temp.insert(route_juge_temp.begin() + i + 1 - insert_pos, item.station);
						if (time_window_feasiable(route_juge_temp) && distance_feasiable(route_juge_temp))
						{
							route = route_juge_temp;
							return true;
						}
					}
				}
				return false;
			}
			if (route[i + 1] > data.cus_num)
				temp_dist = 0;
		}
		return false;
	}
	return true;
}

/* insert charge stations:unused*/
bool R_R::si(int choice)//0:greedy 1:best insertion  recommend greedy method
{
	if (choice == 0)
	{
		for (auto &route : solution)
		{
			bool route_flag = false;
			if (route.changed)
			{
				if (route.veichle_type == 1)
				{
					capacity_m = 2.0; capacity_v = 12.0; length = 100000;
				}
				else if (route.veichle_type == 2)
				{
					capacity_m = 2.5; capacity_v = 16.0; length = 120000;
				}
				if (!time_window_feasiable(route.changed_path) || !load_m_feasiable(route.changed_path) || !load_v_feasiable(route.changed_path))
					return false;
				if (!distance_feasiable(route.changed_path))
				{
					int temp_dist = 0;
					for (size_t i = 0; i < route.changed_path.size() - 1; i++)
					{
						int node = route.changed_path[i];
						int node_next = route.changed_path[i + 1];
						temp_dist += data.dists[node][node_next];
						if (temp_dist > length)
						{
							for (size_t insert_pos = 0; insert_pos <= i; insert_pos++)
							{
								vector<inital_increase_station_dists> increase_station_dist;
								for (int station = data.cus_num + 1; station < data.total_num; station++)
									increase_station_dist.push_back(inital_increase_station_dists{ (data.dists[node][station] + data.dists[station][node_next] - data.dists[node][node_next])*(route.veichle_type * 2 + 10),station });
								std::sort(increase_station_dist.begin(), increase_station_dist.end(), [&](inital_increase_station_dists i1, inital_increase_station_dists i2) {return i1.dist_cost < i2.dist_cost; });
								for (auto &item : increase_station_dist)
								{
									vector<int>route_juge_temp = route.changed_path;
									route_juge_temp.insert(route_juge_temp.begin() + i + 1 - insert_pos, item.station);
									if (time_window_feasiable(route_juge_temp) && distance_feasiable(route_juge_temp))
									{
										route.changed_path = route_juge_temp;
										route_flag = true;
										break;
									}
								}
								if (route_flag)
									break;
							}
						}
						if (route_flag)
							break;
						if (route.changed_path[i + 1] > data.cus_num)
							temp_dist = 0;
					}
					if (!route_flag)
						return false;
				}
			}
		}
		return true;
	}
	else if (choice == 1)//has some bugs
	{
		for (auto &route : solution)
		{
			bool route_flag = false;
			if (route.changed)
			{
				if (route.veichle_type == 1)
				{
					capacity_m = 2.0; capacity_v = 12.0; length = 100000;
				}
				else if (route.veichle_type == 2)
				{
					capacity_m = 2.5; capacity_v = 16.0; length = 120000;
				}
				if (!time_window_feasiable(route.changed_path) || !load_m_feasiable(route.changed_path) || !load_v_feasiable(route.changed_path))
					return false;
				if (!distance_feasiable(route.changed_path))
				{
					int temp_dist = 0;
					for (size_t i = 0; i < route.changed_path.size() - 1; i++)
					{
						int node = route.changed_path[i];
						int node_next = route.changed_path[i + 1];
						temp_dist += data.dists[node][node_next];
						if (temp_dist > length)
						{
							vector<inital_increase_dists> increase_station_dist;
							for (size_t insert_pos = 0; insert_pos <= i; insert_pos++)
							{
								for (int station = data.cus_num + 1; station < data.total_num; station++)
									increase_station_dist.push_back(inital_increase_dists{ int(i + 1 - insert_pos),station, (data.dists[node][station] + data.dists[station][node_next] - data.dists[node][node_next])*(route.veichle_type * 2 + 10) });
							}
							sort(increase_station_dist.begin(), increase_station_dist.end(), [&](inital_increase_dists i1, inital_increase_dists i2) {return i1.dist_cost < i2.dist_cost; });
							for (auto &item : increase_station_dist)
							{
								vector<int>route_juge_temp = route.changed_path;
								route_juge_temp.insert(route_juge_temp.begin() + item.index, item.node);
								if (time_window_feasiable(route_juge_temp) && distance_feasiable(route_juge_temp))
								{
									route.changed_path = route_juge_temp;
									route_flag = true;
									break;
								}
							}
							if (route_flag)
								break;
						}
						if (route_flag)
							break;
						if (route.changed_path[i + 1] > data.cus_num)
							temp_dist = 0;
					}
					if (!route_flag)
						return false;
				}
			}
		}
		return true;
	}
	else
	{
		std::cout << "si choice error!" << endl;
		system("pause");
	}
}

/* remove routes :unused */
void R_R::rr(int choice)//0:random remove, 1:greedy remove,nonsense since the shortest path has no point to share!
{
	//srand(time(NULL));
	vector <int> temp_list;
	if (choice == 0)
	{
		int count = 0;
		//random_shuffle(solution.begin(),solution.end());
		int num2move = rand() % omega + 1;
		for (int i = 0; i < num2move; i++)
		{
			solution[i].remove_flag = true;
			extend_vec<int>(temp_list, solution[i].changed_path);
		}
		for (auto &item : solution)
		{
			if (item.remove_flag)
				count++;
		}
	}
	else if (choice == 1)
	{
		sort(solution.begin(), solution.end(), [](ROUTE &i1, ROUTE &i2) {return i1.path.size() < i2.path.size(); });
		int num2move = rand() % omega + 1;
		for (int i = 0; i < num2move; i++)
		{
			solution[i].remove_flag = true;
			extend_vec<int>(temp_list, solution[i].changed_path);
		}
	}
	else {
		cout << "rr choice error!" << endl;
		system("pause");
	}
	for (auto &item : temp_list)
	{
		if (item > 0 && item <= data.cus_num)
			nodes_unexplored.insert(item);
	}
}

/*cluster reomve
  0:random remove 1:worst distance remove  2:cluster_remove
  only cluster remove was used here*/
void R_R::cr(int choice) 
{
	Noise_Maker noise;
	int num2move = rand() % beta + 1;
	if (choice == 0) {
		random_shuffle(solution.begin(), solution.end());
		for (ROUTE &route : solution)
		{
			if (num2move < 1)
			{
				break;
			}
			int num = rand() % (route.path.size() - 2) + 1;
			int node = route.changed_path[num];
			if (node == 0) {
				cout << "error cr node cannot be zero!";
				cout << num;
				print_vec(route.changed_path);
				print_vec(route.path);
				system("pause");
			}
			else if (node <= data.cus_num) {
				nodes_unexplored.insert(node);
			}
			route.changed_path.erase(route.changed_path.begin() + num);
			route.changed = true;
			num2move--;
			if ((route.changed_path.size() == 3 && route.changed_path[1] > data.cus_num) || (route.changed_path.size() == 2)) {
				route.remove_flag = true;
			}
		}
		if (!si(0)) {
			reset_solution();
		}
	}
	else if (choice == 1)//worst remove 
	{
		vector <Station_Remove> cus_dist_lists;
		for (size_t route_index = 0; route_index < solution.size(); route_index++)
		{
			vector <int> &route = solution[route_index].changed_path;
			for (size_t index = 1; index < route.size() - 1; index++)
			{
				cus_dist_lists.push_back(Station_Remove{ int(route_index),int(index),route[index],(data.dists[route[index - 1]][route[index]]
					+ data.dists[route[index]][route[index + 1]] - data.dists[route[index - 1]][route[index + 1]])*(10 + solution[route_index].veichle_type) + noise.make_noise() * 13 });
			}
		}
		while (num2move > 0)
		{
			int index = max_dists(cus_dist_lists);
			Station_Remove item = cus_dist_lists[index];
			vector<int> &path = solution[item.route_index].changed_path;
			if ((path.size() == 3 && path[1] > data.cus_num) || (path.size() == 2))
			{
				print_vec(path);
				cout << solution[item.route_index].remove_flag;
				system("pause");
			}
			if (item.node == 0) {
				cout << "error worst cr,node equal to zero:";
				system("pause");
			}
			else if (item.node <= data.cus_num) {
				path.erase(path.begin() + item.node_index);
				nodes_unexplored.insert(item.node);
			}
			else if (item.node > data.cus_num) {
				path.erase(path.begin() + item.node_index);
			}
			solution[item.route_index].changed = true;
			if (!modify_si(path, solution[item.route_index].veichle_type))
			{
				reset_solution();
				break;
			}
			num2move--;
			for (auto &cus : cus_dist_lists)
			{
				auto iter = cus_dist_lists.begin();
				while (iter != cus_dist_lists.end())
				{
					if ((*iter).route_index == item.route_index)
						iter = cus_dist_lists.erase(iter);
					else
						++iter;
				}
			}
			if ((path.size() == 3 && path[1] > data.cus_num) || (path.size() == 2)) {
				solution[item.route_index].remove_flag = true;
			}
			else {
				for (size_t index = 1; index < path.size() - 1; index++)
				{
					cus_dist_lists.push_back(Station_Remove{ int(item.route_index),int(index),path[index],(data.dists[path[index - 1]][path[index]]
						+ data.dists[path[index]][path[index + 1]] - data.dists[path[index - 1]][path[index + 1]])*(10 + solution[item.route_index].veichle_type) + noise.make_noise() * 13 });
				}
			}
		}
	}
	else if (choice == 2) {
		//map tell every node's route_index 
		unordered_map<int, int> route_index_lists;
		for (size_t i = 0; i < solution.size(); i++)
		{
			for (int item : solution[i].changed_path)
			{
				route_index_lists[item] = i;
			}
		}
		vector<int> removed_node;
		vector<int> removed_route;

		// flag whether the process terminate
		bool jump = false;
		while (num2move > 0)
		{
			if (removed_node.size() == 0)
			{
				int route_index = rand() % solution.size();
				vector<int> & path = solution[route_index].changed_path;
				removed_route.push_back(route_index);
				vector<int> nodes2move = kmeans(path);

				/* keep the removed node in removed_node */
				for (auto item : nodes2move)
				{
					if (item <= data.cus_num) {
						removed_node.push_back(item);
					}
				}

				/* actual remove done */
				auto it = path.begin();
				while (it != path.end())
				{
					if (item_in_vectors((*it), nodes2move))
					{
						path.erase(it);
						num2move--;
					}
					else {
						it++;
					}
				}
				solution[route_index].changed = true;

				/* judge whether the route need to be removed */
				if (path.size() == 2 || (path.size() == 3 && path[1] > data.cus_num))
				{
					solution[route_index].remove_flag = true;
				}

				/* if the removed path is infeasible then quite and reset solution */
				if (!modify_si(path, solution[route_index].veichle_type))
				{
					reset_solution();
					jump = true;
					break;
				}
			}
			else
			{
				/* choose a start point randomly from removed_node */
				int rad = rand() % removed_node.size();
				auto iter = removed_node.begin();
				int target_request = *(iter + rad);
				int route_index;

				/* choose the new_request from the neighbours of the target node
				only if the new node never occurred before or in any of the removed_route*/
				for (auto new_request : neighbours[target_request])
				{
					if ((!item_in_vectors(new_request, removed_node)) && (!item_in_vectors(route_index_lists[new_request], removed_route)))
					{
						route_index = route_index_lists[new_request];
						break;
					}
				}
				vector<int> & path = solution[route_index].changed_path;
				solution[route_index].changed = true;
				removed_route.push_back(route_index);

				/* only do kmeans for valuable routes */
				if (path.size() == 2 || (path.size() == 3 && path[1] > data.cus_num))
				{
					solution[route_index].remove_flag = true;
				}
				else
				{
					vector<int> nodes2move = kmeans(path);
					for (auto item : nodes2move)
					{
						if (item <= data.cus_num) {
							removed_node.push_back(item);
						}
					}
					auto it = path.begin();
					while (it != path.end())
					{
						if (item_in_vectors((*it), nodes2move))
						{
							path.erase(it);
							num2move--;
						}
						else {
							it++;
						}
					}

					if (path.size() == 2 || (path.size() == 3 && path[1] > data.cus_num))
					{
						solution[route_index].remove_flag = true;
					}
					if (!modify_si(path, solution[route_index].veichle_type))
					{
						reset_solution();
						jump = true;
						break;
					}
				}
			}
		}
		if (!jump) {
			for (int node : removed_node)
			{
				nodes_unexplored.insert(node);
			}
		}
	}
	else {
		cout << "cr choice error!" << endl;
	}
}

/* remove all the customs point by point and insert it point by point
if one point moved successfuly improve the result,then move to another route,
otherwise keep tranverse over the path, if the removed path violate the time window
or distance condition ,then move to another route as well*/
void R_R::cr_point(int choice)
{
	for (size_t route_index = 0; route_index<solution.size(); route_index++)
	{
		ROUTE & item = solution[route_index];
		for (auto j = 1; j < item.path.size() - 1; j++)
		{
			int node = item.changed_path[j];
			int veichle = item.veichle_type;
			vector<int> &temp_route = item.path;//attention of the note
			if (node > data.cus_num)
			{
				continue;
			}
			item.changed_path.erase(item.changed_path.begin() + j);
			if (!time_window_feasiable(item.changed_path) || !distance_feasiable(item.changed_path))//exclude the condition in which the route not feasiable
			{
				item.changed_path = item.path;
				break;
			}

			/* keep track of the removed node,the dist costs decreased by the move ,the refernce of the origin solution and the route index */
			if (ci_point(node, (data.dists[temp_route[j - 1]][node]
				+ data.dists[node][temp_route[j + 1]] - data.dists[temp_route[j - 1]][temp_route[j + 1]])*(10 + 2 * veichle), item, route_index, choice))
			{
				// if improved ,move to another route
				break;
			}
			else
			{
				// if unimproved ,move to the next point to remove
				item.changed_path = temp_route;
			}
		}
	}

	/* at the end of remove,clear the routes which only contains zero or charge stations */
	auto iter = solution.begin();
	while (iter != solution.end())
	{
		if ((*iter).remove_flag) {
			iter = solution.erase(iter);
		}
		else {
			if ((*iter).changed) {
				(*iter).path = (*iter).changed_path;
				(*iter).changed = false;
			}
			++iter;
		}
	}
}

/* choice 0 means greedy insert whle 1 means randomly */
bool R_R::ci_point(int node, double orig_cost_down, ROUTE & origin_route, int route_origin_index,int choice)
{
	vector <Station_Remove> cus_dist_lists;
	for (size_t route_index = 0; route_index < solution.size(); route_index++)
	{
		//exclude the same path for convenience
		if (route_origin_index == route_index)continue;
		int vtype = solution[route_index].veichle_type;
		vector <int> &route = solution[route_index].path;
		for (size_t index = 1; index < route.size(); index++)
		{
			vector <int> temp_route = route;
			int pre = temp_route[index - 1];
			int after = temp_route[index];

			/* use the graph info to help judge:if the point already violate the timewindow,then give it up*/
			if (data.demand_tw0[pre] + 30 > data.demand_tw1[node] || data.demand_tw0[node] + 30 > data.demand_tw1[after]) {
				continue;
			}
			temp_route.insert(temp_route.begin() + index, node);
			if (feasiable(temp_route, vtype))
				cus_dist_lists.push_back(Station_Remove{ int(route_index),int(index),node,(data.dists[temp_route[index - 1]][temp_route[index]]
					+ data.dists[temp_route[index]][temp_route[index + 1]] - data.dists[temp_route[index - 1]][temp_route[index + 1]])*(10 + 2 * vtype) });
		}
	}
	if (cus_dist_lists.size() > 0 &&choice==0)
	{
		double cost_rise;
		int index = min_dists(cus_dist_lists);
		Station_Remove item = cus_dist_lists[index];
		ROUTE & route_after = solution[item.route_index];
		if (item.dist_cost < orig_cost_down)//improve the results
		{
			origin_route.path = origin_route.changed_path;
			route_after.path.insert(route_after.path.begin() + item.node_index, node);
			route_after.changed_path = route_after.path;

			/* judge whether origin routes need to be removed*/
			if (origin_route.changed_path.size() == 2 || (origin_route.changed_path.size() == 3 && origin_route.changed_path[1] > data.cus_num))
			{
				origin_route.remove_flag = true;
			}
			return true;
		}
	}
	/* useless*/
	if (cus_dist_lists.size() > 0 && choice == 1)
	{
		double cost_rise;
		int index = rand()%cus_dist_lists.size();
		double prob = rand() % 101 / double(100);
		if (prob > 0.5) {
			index = min_dists(cus_dist_lists);
		}
		Station_Remove item = cus_dist_lists[index];
		ROUTE & route_after = solution[item.route_index];
		if (item.dist_cost < orig_cost_down)//imply that exchange can be made
		{
			origin_route.path = origin_route.changed_path;
			route_after.path.insert(route_after.path.begin() + item.node_index, node);
			route_after.changed_path = route_after.path;
			if (origin_route.changed_path.size() == 2 || (origin_route.changed_path.size() == 3 && origin_route.changed_path[1] > data.cus_num))
			{
				origin_route.remove_flag = true;
			}
			return true;
		}
	}
	return false;
}

vector<ROUTE1> R_R::create_routes()
{
	vector<ROUTE1> routes;
	set<int> &nodes = nodes_unexplored;
	bool same_route_flag = false;
	while (nodes.size()>0)
	{
		vector<int> temp{ 0 };
		int vtype_prob = 1;
		/*if (rand() % 101 / double(100) > 0.5)
		{
			vtype_prob = 2;
		}*/
		ROUTE1 route(temp, 1);
		/*for (auto first_node : nodes)
		{
			if (nodes_for_vehicle2.find(first_node) != nodes_for_vehicle2.end())
			{
				route.veichle_type = 2;
			}
			route.path.push_back(first_node);
			break;
		}*/
		for (auto first_node : dist2deport_order)
		{
			if (nodes.find(first_node) != nodes.end())
			{
				if (nodes_for_vehicle2.find(first_node) != nodes_for_vehicle2.end())
				{
					route.veichle_type = 2;
				}
				route.path.push_back(first_node);
				break;
			}
		}
		same_route_flag = true;
		while (same_route_flag)
		{
			same_route_flag = false;
			vector<inital_increase_dists> increase_dist_lists;
			for (size_t i = 0; i < route.path.size(); i++)
			{
				for (int node : nodes)
				{
					int dists_cost;
					if (i == route.path.size() - 1)
						dists_cost = data.dists[route.path[i]][node] + data.dists[node][0] - data.dists[route.path[i]][0];
					else
						dists_cost = data.dists[route.path[i]][node] + data.dists[node][route.path[i + 1]] - data.dists[route.path[i]][route.path[i + 1]];
					increase_dist_lists.push_back(inital_increase_dists{ int(i), node, dists_cost });
				}
			}
			sort(increase_dist_lists.begin(), increase_dist_lists.end(), [](const inital_increase_dists &i1, const inital_increase_dists &i2) {return i1.dist_cost < i2.dist_cost; });
			for (auto &item : increase_dist_lists)
			{
				if (item_in_vectors<int>(item.node, route.path))
					continue;
				ROUTE1 route_judge = route;

				route_judge.path.insert(route_judge.path.begin() + item.index + 1, item.node);
				route_judge.path.push_back(0);

				if (feasiable_initial(route_judge, route))
				{
					route.path.pop_back();
					same_route_flag = true;
					break;
				}
			}
		}
		if (route.veichle_type == 1)
		{
			capacity_m = 2.0; capacity_v = 12.0; length = 100000;
		}
		else if (route.veichle_type == 2)
		{
			capacity_m = 2.5; capacity_v = 16.0; length = 120000;
		}
		int count = 0;
		int len = route.path.size();
		for (size_t i = 0; i < len; i++)
		{
			if (route.path[len - i - 1] <= data.cus_num)
			{
				break;
			}
			count++;
		}
		for (int i = 0; i < count; i++)
		{
			route.path.pop_back();
		}
		size_t route_len = route.path.size();
		int temp_dist_best = 9999999, temp_dist = 9999999;
		vector<int> route1 = route.path;
		for (int station = data.cus_num + 1; station < data.total_num; station++)
		{
			vector<int> temp1 = route1;
			temp1.push_back(station);
			temp1.push_back(0);
			temp_dist = data.dists[route.path[route_len - 1]][station] + data.dists[station][0];
			if (temp_dist < temp_dist_best && distance_feasiable(temp1))
			{
				temp_dist_best = temp_dist;
				route.path = temp1;
				route.path.pop_back();
			}
		}
		route.path.push_back(0);
		if (route.path.size() == 3 && data.dists[0][route.path[1]] + data.dists[route.path[1]][0] > length)
		{
			vector<inital_increase_dists> increase_station_dist;
			for (int station = data.cus_num + 1; station < data.total_num; station++)
			{
				vector<int> temp_route = route.path;
				vector<int> temp_route1 = route.path;
				temp_route.insert(temp_route.begin() + 1, station);
				if (data.dists[station][route.path[1]] + data.dists[route.path[1]][0] <= length && time_window_feasiable(temp_route))
				{
					increase_station_dist.push_back(inital_increase_dists{ 1,station,data.dists[0][station] + data.dists[station][route.path[1]] - data.dists[0][route.path[1]] });
				}
				temp_route1.insert(temp_route1.begin() + 2, station);
				if (data.dists[0][route.path[1]] + data.dists[route.path[1]][station] <= length && time_window_feasiable(temp_route1))
				{
					increase_station_dist.push_back(inital_increase_dists{ 2,station,data.dists[route.path[1]][station] + data.dists[station][0] - data.dists[route.path[1]][0] });
				}
			}
			if (increase_station_dist.size() > 0)
			{
				auto temp = increase_station_dist[min_dists_initial(increase_station_dist)];
				route.path.insert(temp.index + route.path.begin(), temp.node);
			}
			else
			{
				cout << "initial error!";
				system("pause");
			}
		}
		routes.push_back(route);
		set<int> route_set;
		vector<int> route_vec;
		for (auto &item : route.path)
		{
			route_set.insert(item);
		}
		set_difference(nodes.begin(), nodes.end(), route_set.begin(), route_set.end(), back_inserter(route_vec));
		nodes.clear();
		for (auto &item : route_vec)
		{
			nodes.insert(item);
		}
	}
	for (auto &item : routes) {
		if (item.path[item.path.size() - 1] != 0 || item.path[item.path.size() - 2] == 0 || !feasiable(item.path, item.veichle_type) || item.path.size() == 0)
		{
			cout << "error create routes!";
			system("pause");
		}
	}
	return routes;
}

void R_R::ci(int choice)//0:greedy,2:regret 2 version   3:regret 3 version
{
	Noise_Maker noise;
	if (choice == 0)//seems ok  with noise 
	{
		unordered_map <int, vector <Station_Remove>> node_dist_list;
		vector <Station_Remove> cus_dist_lists;
		for (auto node : nodes_unexplored)
		{
			for (size_t route_index = 0; route_index < solution.size(); route_index++)
			{
				int vtype = solution[route_index].veichle_type;
				vector <int> &route = solution[route_index].changed_path;
				if (!solution[route_index].remove_flag)
				{
					for (size_t index = 1; index < route.size(); index++)
					{
						vector <int> temp_route = route;
						int pre = temp_route[index - 1];
						int after = temp_route[index];
						if (data.demand_tw0[pre] + 30 > data.demand_tw1[node] || data.demand_tw0[node] + 30 > data.demand_tw1[after]) {
							continue;
						}
						temp_route.insert(temp_route.begin() + index, node);
						if (feasiable(temp_route, vtype))
							node_dist_list[node].push_back(Station_Remove{ int(route_index),int(index),node,(data.dists[temp_route[index - 1]][temp_route[index]]
								+ data.dists[temp_route[index]][temp_route[index + 1]] - data.dists[temp_route[index - 1]][temp_route[index + 1]])*(10 + 2 * vtype) });
					}
				}
			}
			if (node_dist_list[node].size() > 0) {
				sort(node_dist_list[node].begin(), node_dist_list[node].end(), [](Station_Remove &i1, Station_Remove &i2) {return i1.dist_cost < i2.dist_cost; });
				cus_dist_lists.push_back(Station_Remove{ node_dist_list[node][0].route_index,node_dist_list[node][0].node_index, node,node_dist_list[node][0].dist_cost });
			}
		}
		while (nodes_unexplored.size() > 0)
		{
			if (cus_dist_lists.size() > 0)
			{
				int index = min_dists(cus_dist_lists);
				Station_Remove item = cus_dist_lists[index];
				vector<int> &path = solution[item.route_index].changed_path;
				int vtype = solution[item.route_index].veichle_type;
				path.insert(item.node_index + path.begin(), item.node);
				solution[item.route_index].changed = true;
				nodes_unexplored.erase(nodes_unexplored.find(item.node));
				node_dist_list.erase(item.node);
				cus_dist_lists.clear();
				for (auto node : nodes_unexplored)
				{
					auto iter = node_dist_list[node].begin();
					while (iter != node_dist_list[node].end())
					{
						if ((*iter).route_index == item.route_index)
							iter = node_dist_list[node].erase(iter);
						else
							++iter;
					}
					for (size_t index = 1; index < path.size(); index++)
					{
						vector <int> temp_route = path;
						int pre = temp_route[index - 1];
						int after = temp_route[index];
						if (data.demand_tw0[pre] + 30 > data.demand_tw1[node] || data.demand_tw0[node] + 30 > data.demand_tw1[after]) {
							continue;
						}
						temp_route.insert(temp_route.begin() + index, node);
						if (feasiable(temp_route, vtype))
							node_dist_list[node].push_back(Station_Remove{ int(item.route_index),int(index),node,(data.dists[temp_route[index - 1]][temp_route[index]]
								+ data.dists[temp_route[index]][temp_route[index + 1]] - data.dists[temp_route[index - 1]][temp_route[index + 1]])*(10 + vtype * 2) });
						if (index > solution[item.route_index].changed_path.size())
							cout << "why";
					}
					if (node_dist_list[node].size() > 0) {
						sort(node_dist_list[node].begin(), node_dist_list[node].end(), [](Station_Remove &i1, Station_Remove &i2) {return i1.dist_cost < i2.dist_cost; });
						cus_dist_lists.push_back(Station_Remove{ node_dist_list[node][0].route_index,node_dist_list[node][0].node_index, node,node_dist_list[node][0].dist_cost });
					}
				}
				if (nodes_unexplored.size() == 0) {
					ci_post();
				}
			}
			else if (nodes_unexplored.size()>0)//routes must be created;
			{
				vector<ROUTE1>  routes = create_routes();
				double new_costs;
				int num2move = 0;
				for (auto &item : routes)
				{
					new_costs += compute_cost(item.path, item.veichle_type);
				}
				for (auto &item : solution)
				{
					if (item.remove_flag) {
						num2move++;
						continue;
					}
					new_costs += compute_cost(item.changed_path, item.veichle_type);
				}
				if (new_costs < temp_cost && num2move >= routes.size())
				{
					auto iter = solution.begin();
					while (iter != solution.end())
					{
						if ((*iter).remove_flag) {
							iter = solution.erase(iter);
						}
						else {
							if ((*iter).changed) {
								(*iter).path = (*iter).changed_path;
								(*iter).changed = false;
							}
							++iter;
						}
					}
					temp_cost = new_costs;
					for (auto &item : routes)
					{
						solution.push_back(ROUTE(item));
					}
					for (auto &item : solution)
					{
						item.changed = false;
						item.remove_flag = false;
					}
					cout << temp_cost << " " << nodes_unexplored.size() << endl;
				}
				else {
					reset_solution();
				}
			}
		}
	}
	else if (choice == 2)//regret 2
	{
		unordered_map <int, vector <Station_Remove>> node_dist_list;
		vector <Station_Remove> cus_dist_lists;
		for (auto node : nodes_unexplored)
		{
			for (size_t route_index = 0; route_index < solution.size(); route_index++)
			{
				vector <int> &route = solution[route_index].changed_path;
				int vtype = solution[route_index].veichle_type;

				/*only insert in the remaining routes of which remove_flag false*/
				if (!solution[route_index].remove_flag)
				{
					for (size_t index = 1; index < route.size(); index++)
					{
						vector <int> temp_route = route;
						int pre = temp_route[index - 1];
						int after = temp_route[index];
						if (data.demand_tw0[pre] + 30 > data.demand_tw1[node] || data.demand_tw0[node] + 30 > data.demand_tw1[after]) {
							continue;
						}
						temp_route.insert(temp_route.begin() + index, node);
						if (feasiable(temp_route, vtype))
							node_dist_list[node].push_back(Station_Remove{ int(route_index),int(index),node,(data.dists[temp_route[index - 1]][temp_route[index]]
								+ data.dists[temp_route[index]][temp_route[index + 1]] - data.dists[temp_route[index - 1]][temp_route[index + 1]])*(10 + vtype * 2) + noise.make_noise() * 13 });
					}
				}
			}
			if (node_dist_list[node].size() > 1) {
				sort(node_dist_list[node].begin(), node_dist_list[node].end(), [](Station_Remove &i1, Station_Remove &i2) {return i1.dist_cost < i2.dist_cost; });
				cus_dist_lists.push_back(Station_Remove{ node_dist_list[node][0].route_index,node_dist_list[node][0].node_index, node,node_dist_list[node][0].dist_cost - node_dist_list[node][1].dist_cost });
			}
		}
		while (nodes_unexplored.size() > 0)
		{
			if (cus_dist_lists.size() > 0)
			{
				int index = min_dists(cus_dist_lists);
				Station_Remove item = cus_dist_lists[index];
				vector<int> &path = solution[item.route_index].changed_path;
				int vtype = solution[item.route_index].veichle_type;
				path.insert(item.node_index + path.begin(), item.node);
				solution[item.route_index].changed = true;
				nodes_unexplored.erase(nodes_unexplored.find(item.node));
				node_dist_list.erase(item.node);
				cus_dist_lists.clear();
				for (auto node : nodes_unexplored)
				{
					auto iter = node_dist_list[node].begin();
					while (iter != node_dist_list[node].end())
					{
						if ((*iter).route_index == item.route_index)
							iter = node_dist_list[node].erase(iter);
						else
							++iter;
					}
					for (size_t index = 1; index < path.size(); index++)
					{
						vector <int> temp_route = path;
						int pre = temp_route[index - 1];
						int after = temp_route[index];
						if (data.demand_tw0[pre] + 30 > data.demand_tw1[node] || data.demand_tw0[node] + 30 > data.demand_tw1[after]) {
							continue;
						}
						temp_route.insert(temp_route.begin() + index, node);
						if (feasiable(temp_route, vtype))
							node_dist_list[node].push_back(Station_Remove{ int(item.route_index),int(index),node,(data.dists[temp_route[index - 1]][temp_route[index]]
								+ data.dists[temp_route[index]][temp_route[index + 1]] - data.dists[temp_route[index - 1]][temp_route[index + 1]])*(10 + vtype * 2) + noise.make_noise() * 13 });
					}
					if (node_dist_list[node].size() > 1) {
						sort(node_dist_list[node].begin(), node_dist_list[node].end(), [](Station_Remove &i1, Station_Remove &i2) {return i1.dist_cost < i2.dist_cost; });
						cus_dist_lists.push_back(Station_Remove{ node_dist_list[node][0].route_index,node_dist_list[node][0].node_index, node,node_dist_list[node][0].dist_cost - node_dist_list[node][1].dist_cost });
					}
				}
				if (nodes_unexplored.size() == 0) {
					ci_post();
				}
			}
			else if (nodes_unexplored.size()>0) //routes must be created;
			{
				vector<ROUTE1>   routes = create_routes();
				double new_costs=0;
				int num2move = 0;
				for (auto &item : routes)
				{
					new_costs += compute_cost(item.path, item.veichle_type);
				}
				for (auto &item : solution)
				{
					if (item.remove_flag) {
						num2move++;
						continue;
					}
					new_costs += compute_cost(item.changed_path, item.veichle_type);
				}
				if (new_costs < temp_cost && num2move >= routes.size())
				{
					auto iter = solution.begin();
					while (iter != solution.end())
					{
						if ((*iter).remove_flag) {
							iter = solution.erase(iter);
						}
						else {
							if ((*iter).changed) {
								(*iter).path = (*iter).changed_path;
								(*iter).changed = false;
							}
							++iter;
						}
					}
					temp_cost = new_costs;
					//updata_best_solu();
					for (auto &item : routes)
					{
						solution.push_back(ROUTE(item));
					}
					for (auto &item : solution)
					{
						item.changed = false;
						item.remove_flag = false;
					}
				}
				else {
					reset_solution();
				}
			}
		}
	}
	else if (choice == 3)//regret 3
	{
		unordered_map <int, vector <Station_Remove>> node_dist_list;
		vector <Station_Remove> cus_dist_lists;
		for (auto node : nodes_unexplored)
		{
			for (size_t route_index = 0; route_index < solution.size(); route_index++)
			{
				vector <int> &route = solution[route_index].changed_path;
				int vtype = solution[route_index].veichle_type;
				if (!solution[route_index].remove_flag)
				{
					for (size_t index = 1; index < route.size(); index++)
					{
						vector <int> temp_route = route;
						int pre = temp_route[index - 1];
						int after = temp_route[index];
						if (data.demand_tw0[pre] + 30 > data.demand_tw1[node] || data.demand_tw0[node] + 30 > data.demand_tw1[after]) {
							continue;
						}
						temp_route.insert(temp_route.begin() + index, node);
						if (feasiable(temp_route, vtype))
							node_dist_list[node].push_back(Station_Remove{ int(route_index),int(index),node,(data.dists[temp_route[index - 1]][temp_route[index]]
								+ data.dists[temp_route[index]][temp_route[index + 1]] - data.dists[temp_route[index - 1]][temp_route[index + 1]])*(10 + vtype * 2) + noise.make_noise() * 13 });
					}
				}
			}
			if (node_dist_list[node].size() > 2) {
				sort(node_dist_list[node].begin(), node_dist_list[node].end(), [](Station_Remove &i1, Station_Remove &i2) {return i1.dist_cost < i2.dist_cost; });
				cus_dist_lists.push_back(Station_Remove{ node_dist_list[node][0].route_index,node_dist_list[node][0].node_index, node,node_dist_list[node][0].dist_cost - node_dist_list[node][2].dist_cost });
			}
		}
		while (nodes_unexplored.size() > 0)
		{
			if (cus_dist_lists.size() > 0)
			{
				int index = min_dists(cus_dist_lists);
				Station_Remove item = cus_dist_lists[index];
				vector<int> &path = solution[item.route_index].changed_path;
				int vtype = solution[item.route_index].veichle_type;
				path.insert(item.node_index + path.begin(), item.node);
				solution[item.route_index].changed = true;
				nodes_unexplored.erase(nodes_unexplored.find(item.node));
				node_dist_list.erase(item.node);
				cus_dist_lists.clear();
				for (auto node : nodes_unexplored)
				{
					auto iter = node_dist_list[node].begin();
					while (iter != node_dist_list[node].end())
					{
						if ((*iter).route_index == item.route_index)
							iter = node_dist_list[node].erase(iter);
						else
							++iter;
					}
					for (size_t index = 1; index < path.size(); index++)
					{
						vector <int> temp_route = path;
						int pre = temp_route[index - 1];
						int after = temp_route[index];
						if (data.demand_tw0[pre] + 30 > data.demand_tw1[node] || data.demand_tw0[node] + 30 > data.demand_tw1[after]) {
							continue;
						}
						temp_route.insert(temp_route.begin() + index, node);
						if (feasiable(temp_route, vtype))
							node_dist_list[node].push_back(Station_Remove{ int(item.route_index),int(index),node,(data.dists[temp_route[index - 1]][temp_route[index]]
								+ data.dists[temp_route[index]][temp_route[index + 1]] - data.dists[temp_route[index - 1]][temp_route[index + 1]])*(10 + vtype * 2) + noise.make_noise() * 13 });
						if (index > solution[item.route_index].changed_path.size())
							cout << "why";
					}
					if (node_dist_list[node].size() > 2) {
						sort(node_dist_list[node].begin(), node_dist_list[node].end(), [](Station_Remove &i1, Station_Remove &i2) {return i1.dist_cost < i2.dist_cost; });
						cus_dist_lists.push_back(Station_Remove{ node_dist_list[node][0].route_index,node_dist_list[node][0].node_index, node,node_dist_list[node][0].dist_cost - node_dist_list[node][2].dist_cost });
					}
				}
				if (nodes_unexplored.size() == 0) {
					ci_post();
				}
			}
			else if (nodes_unexplored.size()>0) //routes must be created;
			{
				vector<ROUTE1>   routes = create_routes();
				print_set(nodes_unexplored);
				double new_costs;
				int num2move = 0;
				for (auto &item : routes)
				{
					new_costs += compute_cost(item.path, item.veichle_type);
				}
				for (auto &item : solution)
				{
					if (item.remove_flag) {
						num2move++;
						continue;
					}
					new_costs += compute_cost(item.changed_path, item.veichle_type);
				}
				if (new_costs < temp_cost && num2move >= routes.size())
				{
					auto iter = solution.begin();
					while (iter != solution.end())
					{
						if ((*iter).remove_flag) {
							iter = solution.erase(iter);
						}
						else {
							if ((*iter).changed) {
								(*iter).path = (*iter).changed_path;
								(*iter).changed = false;
							}
							++iter;
						}
					}
					temp_cost = new_costs;
					for (auto &item : routes)
					{
						solution.push_back(ROUTE(item));
					}
					for (auto &item : solution)
					{
						item.changed = false;
						item.remove_flag = false;
					}
				}
				else {
					reset_solution();
				}
			}
		}
	}
	else {
		std::cout << "ci choice error!" << endl;
		system("pause");
	}
}

void R_R::ci_post()
{
	double new_costs = 0;
	for (auto &item : solution)
	{
		if (item.remove_flag) {
			continue;
		}
		new_costs += compute_cost(item.changed_path, item.veichle_type);
	}
	if (new_costs < temp_cost)
	{
		auto iter = solution.begin();
		while (iter != solution.end())
		{
			if ((*iter).remove_flag) {
				iter = solution.erase(iter);
			}
			else {
				if ((*iter).changed) {
					(*iter).path = (*iter).changed_path;
					(*iter).changed = false;
				}
				++iter;
			}
		}
		temp_cost = new_costs;
		//updata_best_solu();
		for (auto &item : solution)
		{
			item.changed = false;
			item.remove_flag = false;
		}
	}
	else {
		reset_solution();
	}
}

/* exchange one point randomly from two routes 
if possible and improved ,then change the result in the operation*/
void R_R::inter_tour_exchange(int iterations,int penalty)
{
	int count = 0;
	int num = solution.size();
	for (int i = 0; i < iterations; i++)
	{
		//srand(time(NULL));
		int route_a_index = rand() % num;
		int route_b_index = rand() % num;
		if (route_a_index == route_b_index)
			continue;
		vector<int>route_a = solution[route_a_index].path;
		vector<int>route_b = solution[route_b_index].path;
		int a = solution[route_a_index].veichle_type;
		int b = solution[route_b_index].veichle_type;
		int a_coef = a * 2 + 10;
		int b_coef = b * 2 + 10;
		int node_a_index = rand() % (route_a.size() - 2) + 1;
		int node_b_index = rand() % (route_b.size() - 2) + 1;
		int node_a = route_a[node_a_index];
		int node_b = route_b[node_b_index];
		int dists_orig = (data.dists[route_a[node_a_index - 1]][node_a] + data.dists[node_a][route_a[node_a_index + 1]])*a_coef
			+ (data.dists[route_b[node_b_index - 1]][node_b] + data.dists[node_b][route_b[node_b_index + 1]])*b_coef;
		int dists_after = (data.dists[route_a[node_a_index - 1]][node_b] + data.dists[node_b][route_a[node_a_index + 1]])*a_coef
			+ (data.dists[route_b[node_b_index - 1]][node_a] + data.dists[node_a][route_b[node_b_index + 1]])*b_coef;
		route_a[node_a_index] = node_b;
		route_b[node_b_index] = node_a;
		if (dists_orig > dists_after - penalty * 13 && feasiable(route_a, a) && feasiable(route_b, b))
		{
			count++;
			solution[route_a_index].path[node_a_index] = node_b;
			solution[route_b_index].path[node_b_index] = node_a;
			solution[route_a_index].changed_path[node_a_index] = node_b;
			solution[route_b_index].changed_path[node_b_index] = node_a;
			break;
		}
	}
	//std::cout << "inter_tour_exchanged_num:--------------------------------" << count << endl;
}

/*exchange one point randomly from two routes,
first it transverse all the routes ,then randomly choose another route*/
void R_R::inter_tour_exchange1(int iterations, int penalty)
{
	//int count = 0;
	int num = solution.size();
	for (auto route_index = 0; route_index < solution.size(); route_index++)
	{
		int route_a_index = route_index;
		for (int k = 0; k < iterations; k++)
		{
			int route_b_index = rand() % (num-route_a_index)+route_a_index;
			if (route_b_index == route_a_index)continue;
			vector<int>route_a = solution[route_a_index].path;
			vector<int>route_b = solution[route_b_index].path;
			int a = solution[route_a_index].veichle_type;
			int b = solution[route_b_index].veichle_type;
			int a_coef = a * 2 + 10;
			int b_coef = b * 2 + 10;
			int node_a_index = rand() % (route_a.size() - 2) + 1;
			int node_b_index = rand() % (route_b.size() - 2) + 1;
			int node_a = route_a[node_a_index];
			int node_b = route_b[node_b_index];
			int dists_orig = (data.dists[route_a[node_a_index - 1]][node_a] + data.dists[node_a][route_a[node_a_index + 1]])*a_coef
				+ (data.dists[route_b[node_b_index - 1]][node_b] + data.dists[node_b][route_b[node_b_index + 1]])*b_coef;
			int dists_after = (data.dists[route_a[node_a_index - 1]][node_b] + data.dists[node_b][route_a[node_a_index + 1]])*a_coef
				+ (data.dists[route_b[node_b_index - 1]][node_a] + data.dists[node_a][route_b[node_b_index + 1]])*b_coef;
			route_a[node_a_index] = node_b;
			route_b[node_b_index] = node_a;
			if (dists_orig > dists_after - penalty * 13 && feasiable(route_a, a) && feasiable(route_b, b))
			{
				//count++;
				solution[route_a_index].path[node_a_index] = node_b;
				solution[route_b_index].path[node_b_index] = node_a;
				solution[route_a_index].changed_path[node_a_index] = node_b;
				solution[route_b_index].changed_path[node_b_index] = node_a;
				break;
			}
		}
	}	
	//std::cout << "inter_tour_exchanged_num:--------------------------------" << count << endl;
}

/*same as inter_tour_exchange1 except for exchange two successful nodes */
void R_R::inter_tour_exchange_two_points(int iterations, int penalty)
{
	//int count = 0;
	int num = solution.size();
	for (auto route_index = 0; route_index < solution.size(); route_index++)
	{
		int route_a_index = route_index;
		for (int k = 0; k < iterations; k++)
		{
			int route_b_index = rand() % (num - route_a_index) + route_a_index;
			if (route_b_index == route_a_index)continue;
			vector<int>route_a = solution[route_a_index].path;
			vector<int>route_b = solution[route_b_index].path;
			int a = solution[route_a_index].veichle_type;
			int b = solution[route_b_index].veichle_type;
			int a_coef = a * 2 + 10;
			int b_coef = b * 2 + 10;
			int node_a_index = rand() % (route_a.size() - 2) + 1;
			int node_b_index = rand() % (route_b.size() - 2) + 1;
			int node_a = route_a[node_a_index];
			int node_b = route_b[node_b_index];
			int node_b_next = route_b[node_b_index + 1];
			if (node_b_next == 0)continue;
			int dists_orig = (data.dists[route_a[node_a_index - 1]][node_a] + data.dists[node_a][route_a[node_a_index + 1]])*a_coef
				+ (data.dists[route_b[node_b_index - 1]][node_b] + data.dists[node_b][node_b_next]+ data.dists[node_b_next][route_b[node_b_index + 2]])*b_coef;
			int dists_after = (data.dists[route_a[node_a_index - 1]][node_b] + data.dists[node_b][node_b_next]+ data.dists[node_b_next][route_a[node_a_index + 1]])*a_coef
				+ (data.dists[route_b[node_b_index - 1]][node_a] + data.dists[node_a][route_b[node_b_index + 2]])*b_coef;
			route_a[node_a_index] = node_b;
			route_a.insert(route_a.begin() + node_a_index + 1, node_b_next);
			route_b[node_b_index] = node_a;
			route_b.erase(route_b.begin() + node_b_index + 1);
			if (dists_orig > dists_after - penalty * 13 && feasiable(route_a, a) && feasiable(route_b, b))
			{
				//count++;
				vector<int> &routea = solution[route_a_index].path;
				vector<int> & routeb = solution[route_b_index].path;
				vector<int> &routea_changed = solution[route_a_index].changed_path;
				vector<int> &routeb_changed = solution[route_b_index].changed_path;
				routea[node_a_index] = node_b;
				routea.insert(routea.begin() + node_a_index + 1, node_b_next);
				routeb[node_b_index] = node_a;
				routeb.erase(routeb.begin() + node_b_index + 1);
				routea_changed[node_a_index] = node_b;
				routea_changed.insert(routea_changed.begin() + node_a_index + 1, node_b_next);
				routeb_changed[node_b_index] = node_a;
				routeb_changed.erase(routeb_changed.begin() + node_b_index + 1);
				break;
			}
		}
	}
	//std::cout << "inter_tour_exchanged_num_two-points:--------------------------------" << count << endl;
}

/*exchange one point with the 500 nearest points of a randomly chosen point*/
void R_R::inter_tour_exchange_nearest100(int iterations, int penalty)
{
	vector<int> targets_lists;
	bool flag = false;
	unordered_map<int, LS> route_index_lists;
	for (size_t i = 0; i < solution.size(); i++)
	{
		for (size_t k = 1; k<solution[i].path.size() - 1; k++)
		{
			int item = solution[i].changed_path[k];
			route_index_lists[item] = LS{ int(i),int(k),item };
		}
	}
	//int count = 0;
	for (int i = 0; i < iterations; i++)
	{
		if (flag)
			break;
		int target = rand() % data.cus_num + 1;
		if (item_in_vectors(target, targets_lists))
		{
			i--;
			continue;
		}
		int route_index_target = route_index_lists[target].route_index;
		int node_index_target = route_index_lists[target].node_index;
		int neighbors_num = 500;
		for (size_t index = 0; index < neighbors_num; index++)
		{
			int obj = neighbours[target][index];
			int route_index_obj = route_index_lists[obj].route_index;
			int node_index_obj = route_index_lists[obj].node_index;
			if (route_index_obj == route_index_target || obj>data.cus_num || obj == 0) {
				continue;
			}

			vector<int>route_a = solution[route_index_target].path;
			vector<int>route_b = solution[route_index_obj].path;
			int a = solution[route_index_target].veichle_type;
			int b = solution[route_index_obj].veichle_type;
			int a_coef = a * 2 + 10;
			int b_coef = b * 2 + 10;
			if (node_index_obj == 0)
			{
				cout << "node index object equals to zeros";
				reset_best_solu();
				flag = true;
				system("pause");
				break;
			}

			int dists_orig = (data.dists[route_a[node_index_target - 1]][target] + data.dists[target][route_a[node_index_target + 1]])*a_coef
				+ (data.dists[route_b[node_index_obj - 1]][obj] + data.dists[obj][route_b[node_index_obj + 1]])*b_coef;
			int dists_after = (data.dists[route_a[node_index_target - 1]][obj] + data.dists[obj][route_a[node_index_target + 1]])*a_coef
				+ (data.dists[route_b[node_index_obj - 1]][target] + data.dists[target][route_b[node_index_obj + 1]])*b_coef;
			route_a[node_index_target] = obj;
			route_b[node_index_obj] = target;
			if (dists_orig > dists_after - penalty * 13 && feasiable(route_a, a) && feasiable(route_b, b))
			{
				//count++;
				solution[route_index_obj].path[node_index_obj] = target;
				solution[route_index_target].path[node_index_target] = obj;
				solution[route_index_obj].changed_path[node_index_obj] = target;
				solution[route_index_target].changed_path[node_index_target] = obj;
				route_index_lists[target] = LS{ int(route_index_obj),int(node_index_obj),target };
				route_index_lists[obj] = LS{ int(route_index_target),int(node_index_target),obj };
				targets_lists.push_back(target);
				break;
			}
		}
	}
	//std::cout << "inter_tour_exchanged_num_nearest100:----------------------------------------" << count << endl;
}

/*exchange two points of a randomly chosen route,the distance is non_exact
the purpose of this operation was to diverse the solution as well as improve
to some extend*/
void R_R::within_tour_exchange(int iterations,int penalty)
{
	int count = 0;
	int num = solution.size();
	for (int i = 0; i < iterations; i++)
	{
		//srand(time(NULL));
		int route_index = rand() % num;
		vector<int>route = solution[route_index].path;
		int a = solution[route_index].veichle_type;
		int node_a_index = rand() % (route.size() - 2) + 1;
		int node_b_index = rand() % (route.size() - 2) + 1;
		int node_a = route[node_a_index];
		int node_b = route[node_b_index];
		int dists_orig = data.dists[route[node_a_index - 1]][node_a] + data.dists[node_a][route[node_a_index + 1]]
			+ data.dists[route[node_b_index - 1]][node_b] + data.dists[node_b][route[node_b_index + 1]];
		int dists_after = data.dists[route[node_a_index - 1]][node_b] + data.dists[node_b][route[node_a_index + 1]]
			+ data.dists[route[node_b_index - 1]][node_a] + data.dists[node_a][route[node_b_index + 1]];
		route[node_a_index] = node_b;
		route[node_b_index] = node_a;
		if (dists_orig > dists_after-penalty && feasiable(route, a))
		{
			count++;
			solution[route_index].path[node_a_index] = node_b;
			solution[route_index].path[node_b_index] = node_a;
			solution[route_index].changed_path[node_a_index] = node_b;
			solution[route_index].changed_path[node_b_index] = node_a;
			break;
		}
	}
	//cout << "within_tour_exchanged_num:------------------------------------------" << count << endl;
}

/*same as within_tour_exchange except for first transverse all the routes,
the distance is non_exact */
void R_R::within_tour_exchange1(int penalty)//the same alog as within_tour_change except for routes chosen
{
	int count = 0;
	int num = solution.size();
	for (size_t route_index = 0; route_index < solution.size(); route_index++)
	{
		int route_flag = 10;
		while (route_flag)
		{
			vector<int>route = solution[route_index].path;
			int a = solution[route_index].veichle_type;
			int node_a_index = rand() % (route.size() - 2) + 1;
			int node_b_index = rand() % (route.size() - 2) + 1;
			int node_a = route[node_a_index];
			int node_b = route[node_b_index];
			int dists_orig = data.dists[route[node_a_index - 1]][node_a] + data.dists[node_a][route[node_a_index + 1]]
				+ data.dists[route[node_b_index - 1]][node_b] + data.dists[node_b][route[node_b_index + 1]];
			int dists_after = data.dists[route[node_a_index - 1]][node_b] + data.dists[node_b][route[node_a_index + 1]]
				+ data.dists[route[node_b_index - 1]][node_a] + data.dists[node_a][route[node_b_index + 1]];
			route[node_a_index] = node_b;
			route[node_b_index] = node_a;
			if (dists_orig > dists_after - penalty && feasiable(route, a))
			{
				count++;
				solution[route_index].path[node_a_index] = node_b;
				solution[route_index].path[node_b_index] = node_a;
				solution[route_index].changed_path[node_a_index] = node_b;
				solution[route_index].changed_path[node_b_index] = node_a;
				break;
			}
			route_flag--;
		}
	}
	//cout << "within_tour_exchanged_num:------------------------------------------" << count << endl;
}

/* actual distance used*/
void R_R::within_tour_exchange2(int penalty)
{
	int count = 0;
	int num = solution.size();
	for (size_t route_index = 0; route_index < solution.size(); route_index++)
	{
		int route_flag = 20;
		while (route_flag)
		{
			vector<int>route = solution[route_index].path;
			int a = solution[route_index].veichle_type;
			int node_a_index = rand() % (route.size() - 2) + 1;
			int node_b_index = rand() % (route.size() - 2) + 1;
			if (node_a_index >= node_b_index)
			{
				route_flag--;
				continue;
			}
			int node_a = route[node_a_index];
			int node_b = route[node_b_index];
			int dists_orig, dists_after;
			if (node_b_index - node_a_index == 1)
			{
				dists_orig = data.dists[route[node_a_index - 1]][node_a] + data.dists[node_a][node_b]+ data.dists[node_b][route[node_b_index + 1]];
				dists_after = data.dists[route[node_a_index - 1]][node_b] + data.dists[node_b][node_a] + data.dists[node_a][route[node_b_index + 1]];
			}
			else {
				dists_orig = data.dists[route[node_a_index - 1]][node_a] + data.dists[node_a][route[node_a_index + 1]]
					+ data.dists[route[node_b_index - 1]][node_b] + data.dists[node_b][route[node_b_index + 1]];
				dists_after = data.dists[route[node_a_index - 1]][node_b] + data.dists[node_b][route[node_a_index + 1]]
					+ data.dists[route[node_b_index - 1]][node_a] + data.dists[node_a][route[node_b_index + 1]];
			}
			route[node_a_index] = node_b;
			route[node_b_index] = node_a;
			if (dists_orig > dists_after - penalty && feasiable(route, a))
			{
				count++;
				solution[route_index].path[node_a_index] = node_b;
				solution[route_index].path[node_b_index] = node_a;
				solution[route_index].changed_path[node_a_index] = node_b;
				solution[route_index].changed_path[node_b_index] = node_a;
				break;
			}
			route_flag--;
		}
	}
	//cout << "within_tour_exchanged_num2:------------------------------------------" << count << endl;
}

