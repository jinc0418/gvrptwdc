/************************************************************************************************
Copyright:
Author:jincheng
Date:2018-8-30
Descriptions:
the project is designed to solve the large scale G-VRPTWDC using R&R combined with LS.
This file composite of some construct fuctions including the import of origin data,
and some tiny R_R fuctions as well.
************************************************************************************************/
#include"header.h"
#include"template.cpp"

/* used only for data read to split a string using a certain symbol returning a vector of characters */
std::vector<std::string> split(const  std::string& s, const std::string& delim)
{
	std::vector<std::string> elems;
	size_t pos = 0;
	size_t len = s.length();
	size_t delim_len = delim.length();
	if (delim_len == 0) return elems;
	while (pos < len)
	{
		int find_pos = s.find(delim, pos);
		if (find_pos < 0)
		{
			elems.push_back(s.substr(pos, len - pos));
			break;
		}
		elems.push_back(s.substr(pos, find_pos - pos));
		pos = find_pos + delim_len;
	}
	return elems;
}

/* default constructor of Data */
Data::Data(){}

/* constructor of Data in which origin data was loaded */
Data::Data(int cluster_num)
{
	string input_dist_name;
	string input_node_name;
	string line;
	ifstream input_dists,input_nodes;
	switch (cluster_num)
	{
		case 1:
		{
			input_dist_name = "inputdistancetime_1_1601.txt";
			input_node_name = "inputnode_1_1601.csv";
			total_num = 1601; cus_num = 1500;
			break;
		}	
		case 2:
		{
			input_dist_name = "inputdistancetime_2_1501.txt";
			input_node_name = "inputnode_2_1501.csv";
			total_num = 1501; cus_num = 1400;
			break;
		}
		case 3:
		{
			input_dist_name = "inputdistancetime_3_1401.txt";
			input_node_name = "inputnode_3_1401.csv";
			total_num = 1401; cus_num = 1300;
			break;
		}
		case 4:
		{
			input_dist_name = "inputdistancetime_4_1301.txt";
			input_node_name = "inputnode_4_1301.csv";
			total_num = 1301; cus_num = 1200;
			break;
		}
		case 5:
		{
			input_dist_name = "inputdistancetime_5_1201.txt";
			input_node_name = "inputnode_5_1201.csv";
			total_num = 1201; cus_num = 1100;
			break;
		}
		default:
			break;
	}

	/* new areas must be allocated using dynamic two dimensional arrays.
	there is a RAM leak here that I did not free the area in the process*/
	dists = new int*[total_num];
	for (size_t i = 0; i < total_num; i++)
		dists[i] = new int[total_num];
	dists_time = new int*[total_num];
	for (size_t i = 0; i < total_num; i++)
		dists_time[i] = new int[total_num];

	input_dists.open(input_dist_name, ios::in);
	std::getline(input_dists, line);//read the first line and abandon
	while (std::getline(input_dists, line)) //read every line for loops
	{
		stringstream ss;
		int index, from, to;
		char c;
		ss << line; //assign line to ss
		ss >> index >> c >> from >> c >> to >> c; //read every character of the line
		if(from!=0)
			from -= cluster_num * 10000;
		if(to!=0)
			to -= cluster_num * 10000;
		ss >> dists[from][to] >> c;
		ss >> dists_time[from][to];
	}
	input_nodes.open(input_node_name, ios::in);//open the file
	getline(input_nodes, line);//read first line
	getline(input_nodes, line);//read second line
	
	while (true)
	{
		/* the idea of stringstream to transform data was fantastic */
		stringstream ss;
		int id, type;
		double ing, lat;
		char c;
		ss << line;
		ss >> id >> c >> type >> c >> ing >> c >> lat;
		demand_m[0] = 0; demand_v[0] = 0; demand_tw0[0] = 480; demand_tw1[0] = 1440;
		pointx[0] = ing;
		pointy[0] = lat;
		break;
	}
	for (size_t i = 1; i <= cus_num; i++)
	{
		getline(input_nodes, line);
		stringstream ss;
		int id, type, hour, min,hour1,min1;
		double ing, lat, m, v;
		char c;
		ss << line;
		ss >> id >> c >> type >> c >> ing >> c >> lat >> c >> m >> c >> v >> c >> 
			hour >> c >> min >> c >> hour1 >> c >> min1 ;
		//cout <<m<<"  "<<v << "  " <<hour << "  " <<min << "  " <<hour1 << "  " <<min1 << "  " <<endl;
		demand_m[i] = m;
		demand_v[i] = v;
		demand_tw0[i] = hour * 60 + min;
		demand_tw1[i] = hour1 * 60 + min1;
		pointx[i] = ing;
		pointy[i] = lat;
	}
	for (size_t i = cus_num+1; i <total_num; i++)
	{
		getline(input_nodes, line);
		stringstream ss;
		int id, type;
		double ing, lat;
		char c;
		ss << line;
		ss >> id >> c >> type >> c >> ing >> c >> lat;
		demand_m[i] = 0;
		demand_v[i] = 0;
		demand_tw0[i] = 480;
		demand_tw1[i] = 1440;
		pointx[i] = ing;
		pointy[i] = lat;
	}
}

/* bugs that I cannot free the Two-dimensional array here */
Data::~Data()
{
	/*cout << total_num << cus_num;
	for (size_t i = 0; i < total_num; i++)
	{
		delete[] dists[i];
		delete[] dists_time[i];
	}
	delete[] dists;
	delete[] dists_time;*/
}

/* constructor of ROUTE with the parameter of vector acting as the interface with inital solutions*/
ROUTE::ROUTE(vector<int>route)
{
	if (route[route.size() - 1] == 10000) {
		veichle_type = 1;
	}
	else
	{
		veichle_type = 2;
	}	
	route.pop_back();
	path = route;
	changed_path = path;
}

/* constructor of ROUTE with ROUTE1 type which only contains veichle_type and a path */
ROUTE::ROUTE(ROUTE1 &route)
{
	if (route.veichle_type==1) {
		veichle_type = 1;
	}
	else
	{
		veichle_type = 2;
	}
	path = route.path;
	changed_path = route.path;
}

/* constructor of ROUTE with anther ROUTE and a flag i in order to distinguish */
ROUTE::ROUTE(ROUTE &route,int i)
{
	veichle_type = route.veichle_type;
	changed_path = route.changed_path;
}

/* constructor of ROUTE with anther ROUTE and flags i and j in order to distinguish */
ROUTE::ROUTE(ROUTE &route, int i, int j)
{
	veichle_type = route.veichle_type;
	changed_path = route.changed_path;
	path = changed_path;
}

/* default constructor of ROUTE doing nothing but kept in order to default copy*/
ROUTE::ROUTE()
{

}

/* constructor of ROUTE1 */
ROUTE1::ROUTE1(vector<int>&route, int type)
{
	path = route;
	veichle_type = type;
}

/* a brilliant operator copied from TSPTW-LKH but no help here */
void R_R::within_tour_opt()
{
	best_cost = compute_all_costs();
	int count = 0;
	bool reset = true;
	for (auto &item : solution)
	{
		vector<int>&route = item.changed_path;
		int vtype = item.veichle_type;
		if (vtype == 1)
		{
			capacity_m = 2.0; capacity_v = 12.0; length = 100000;
		}
		else if (vtype == 2)
		{
			capacity_m = 2.5; capacity_v = 16.0; length = 120000;
		}
		int numNodes = item.path.size();
		bool improvement = false;
		int setSize = numNodes - 2;
		int *set = new int[setSize];
		for (int i = 0; i < setSize; i++)
		{
			set[i] = i + 1;
		}
		int pos = 0; int npos = 0; int n1 = 0; int n2 = 0; int n3 = 0; int n4 = 0; int aux = 0;
		int e1 = 0; int e2 = 0; int e3 = 0; int e4 = 0; int s1 = 0; int s2 = 0; int isf = 0; int n1_1 = 0; int n1_2 = 0;
		while (setSize > 0)
		{
			improvement = false;
			pos = rand() % setSize;
			npos = set[pos];
			n1 = route[npos];
			n2 = route[npos + 1];
			e1 = data.dists[n1][n2];
			for (int j = npos + 2; j < numNodes-1; j++)
			{
				n1_1 = route[npos + 2];
				n3 = route[j];
				n1_2 = route[j-1];
				if (data.demand_tw0[n2] + 30 > data.demand_tw1[n3])
				{
					break;
				}
				//cout << n1_1 << " " << n1_2 << " " << n2 << " " << n3 << " " << endl;
				n4 = route[j + 1];
				e2 = data.dists[n3][n4];
				s1 = e1 + e2+data.dists[n2][n1_1]+ data.dists[n1_2][n3];
				e4 = data.dists[n2][n4];
				e3 = data.dists[n1][n3];
				s2 = e3 + e4 + data.dists[n3][n1_1] + data.dists[n1_2][n2];
				if (s1 > s2)//the origin distance larger than changed
				{
					vector<int> temp_route = route;
					int pre = route[npos]; int after = route[j];
					temp_route[npos] = after; temp_route[j] = pre;
					if (time_window_feasiable(temp_route) && distance_feasiable(temp_route))
					{
						route[npos] = after; route[j] = pre;
						improvement = true;
						count++;
						break;
					}
				}
			}
			{
				aux = set[setSize - 1];
				set[setSize - 1] = npos;
				set[pos] = aux;
				setSize--;
			}
		}
		delete[] set;
	}
	if (compute_all_costs() < best_cost+500) {
		//system("pause");
		for (auto &item : solution)
		{
			item.path = item.changed_path;
		}
		reset = false;
	}
	if (reset)
	{
		cout << "---------------------------reset------------------------------"<<endl;
		for (auto &item : solution)
		{
			item.changed_path = item.path;
		}
	}
	else {
		cout << "within_tour_opt:------------------------------------------" << count << endl;
	}
}

/* for result print purpose*/
bool R_R::print_feasiable(vector<int>&path, int temp_time)
{
	int temp = temp_time-30;
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
string R_R::get_time_back(int time)
{
	string back;
	int hour = time / 60;
	if (hour < 10)
	{
		back = back + "0" + numTostr<int>(hour);
	}
	else {
		back = back+ numTostr<int>(hour);
	}
	int min = time - 60 * hour;
	if (min < 10) {
		back = back +":"+ "0" + numTostr<int>(min);
	}
	else {
		back = back + ":" + numTostr<int>(min);
	}
	return back;
}

//if the temperate solutoin is better,then assign it to the best solution
void R_R::updata_best_solu()
{
	best_solution.clear();
	for (auto &item : solution)
	{
		best_solution.push_back(ROUTE(item,1));
	}
}

/*this function was used at the end which assign the best solution
to the current solution for output convenience */
void R_R::reset_best_solution()
{
	solution.clear();
	for (auto &item : best_solution)
	{
		solution.push_back(ROUTE(item, 1, 2));
	}
}

/* used in LS ,the difference between reset_best_solu is the route format */
void R_R::reset_best_solu()
{
	solution.clear();
	for (auto &item : best_solution)
	{
		solution.push_back(ROUTE(item, 1));
	}
}

void R_R::charge_dispose()
{
	for (auto &item : solution)
	{
		for (int index = 1; index < item.path.size()-1; index++)
		{
			if (item.path[index] > data.cus_num && item.path[index + 1] == item.path[index])
			{
				item.path.erase(item.path.begin() + index);
				index--;
			}
		}
	}
}

void R_R::inter_tour_exchange_nearest_all(int penalty)
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
	for (int target = 1; target <= data.cus_num; target++)
	{
		if (flag)
			break;
		if (item_in_vectors(target, targets_lists))
		{
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
				//targets_lists.push_back(target);
				targets_lists.push_back(obj);
				break;
			}
		}
	}
	//std::cout << "inter_tour_exchanged_num_nearest100:----------------------------------------" << count << endl;
}

void R_R::inter_tour_exchange_time_related(int penalty)
{
	for (auto &item : solution)
	{
		int temp = 450;
		int a, b;
		for (int index = 0; index < item.path.size(); index++)
		{
			if (index == 0 || index == item.path.size() - 1) {
				item.path_time.push_back(480);
				continue;
			}
			a = temp + service + data.dists_time[item.path[index]][item.path[index + 1]];
			b = data.demand_tw0[item.path[index + 1]];
			temp = (a > b ? a : b);
			item.path_time.push_back(temp);
		}
	}
	vector<int> targets_lists;
	unordered_map<int, LS_TIME> time_index_lists;
	for (size_t i = 0; i < solution.size(); i++)
	{
		for (size_t k = 1; k<solution[i].path.size() - 1; k++)
		{
			int item = solution[i].path[k];
			time_index_lists.insert(pair<int,LS_TIME>(item, LS_TIME{ int(i),int(k),item,solution[i].path_time[k] }));
		}
	}
	bool flag = false;
	int allow_diff = 30;
	int count = 0;
	for (int target = 1; target <= data.cus_num; target++)
	{
		if (flag)
			break;
		if (item_in_vectors(target, targets_lists))
		{
			continue;
		}
		int target_time = time_index_lists[target].arr_time;
		int route_index_target = time_index_lists[target].route_index;
		int node_index_target = time_index_lists[target].node_index;
		for (int node = 1; node < data.cus_num; node++)
		{
			if (abs(time_index_lists[node].arr_time - target_time) <= allow_diff)
			{
				int obj = node;
				int route_index_obj = time_index_lists[obj].route_index;
				int node_index_obj = time_index_lists[obj].node_index;
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
					ROUTE & route_obj = solution[route_index_obj];
					ROUTE & route_target = solution[route_index_target];
					route_obj.path[node_index_obj] = target;
					route_target.path[node_index_target] = obj;
					route_obj.changed_path[node_index_obj] = target;
					route_target.changed_path[node_index_target] = obj;

					route_target.path_time.clear();
					int temp = 450;
					for (int index = 0; index < route_target.path.size(); index++)
					{
						if (index == 0 || index == route_target.path.size() - 1) {
							route_target.path_time.push_back(480);
							continue;
						}
						a = temp + service + data.dists_time[route_target.path[index]][route_target.path[index + 1]];
						b = data.demand_tw0[route_target.path[index + 1]];
						temp = (a > b ? a : b);
						route_target.path_time.push_back(temp);
					}

					route_obj.path_time.clear();
					temp = 450;
					for (int index = 0; index < route_obj.path.size(); index++)
					{
						if (index == 0 || index == route_obj.path.size() - 1) {
							route_obj.path_time.push_back(480);
							continue;
						}
						a = temp + service + data.dists_time[route_obj.path[index]][route_obj.path[index + 1]];
						b = data.demand_tw0[route_obj.path[index + 1]];
						temp = (a > b ? a : b);
						route_obj.path_time.push_back(temp);
					}

					time_index_lists[target] = LS_TIME{ int(route_index_obj),int(node_index_obj),target,route_obj.path_time[node_index_obj] };
					time_index_lists[obj] = LS_TIME{ int(route_index_target),int(node_index_target),obj,route_target.path_time[node_index_target] };
					//targets_lists.push_back(target);
					targets_lists.push_back(obj);
					count++;
					break;
				}
			}
		}
	}
	std::cout << "inter_tour_exchanged_num_nearest100:----------------------------------------" << count << endl;
}