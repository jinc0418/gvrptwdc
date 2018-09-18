/************************************************************************************************
Copyright:
Author:jincheng
Date:2018-8-30
Descriptions:
The project is designed to solve the large scale G-VRPTWDC using R&R combined with LS.
This file is the R_R entry which includes all the parameters to be tuned ,besides it output
the demanding csv result for the solution
************************************************************************************************/
#include"header.h"
#include"template.cpp"

void R_R::run_r_r()
{
	clock_t start = clock();
	best_cost = compute_all_costs();
	temp_cost = best_cost;
	std::cout << best_cost << endl;
	vector<double> costs_per_thousand;
	int Max_iterate=12000; 
	int Penalty=5000;
	int inter_two_para = 100;
	int inter_point=30;
	int ls_para=4000;

	switch (cluster_num)
	{
	case 1:
		break;
	case 2:
		ls_para = 6000;
		break;
	case 3:
		inter_two_para = 200;
		break;
	case 4:
		inter_two_para = 250;
		break;
	case 5:
		Max_iterate = 20000;
		inter_point = 20;
		ls_para = 8000;
		Penalty = 6000;
		break;

	default:
		break;
	}
	int penalty = Penalty;
	//Max_iterate = 20000;
	int penalty_retrieve_num = 0;
	for (int i = 0; i < Max_iterate; i++)
	{
		if (i == 15000)break;
		if (i % 1000 == 0)
		{
			costs_per_thousand.push_back(compute_all_costs());
			cout << "----------------------------" << i << "----------------------------" << endl;
		}
		if (i % inter_point == 0)
		{
			penalty_retrieve_num++;
			clock_t start1 = clock();
			if (i < ls_para)
			{
				inter_tour_exchange_nearest100(999, Penalty);
				inter_tour_exchange_nearest_all(Penalty);
				inter_tour_exchange1(inter_two_para, Penalty);
				inter_tour_exchange_two_points(inter_two_para, Penalty);
			}
			else
			{
				if (penalty_retrieve_num == 40)
				{
					penalty = Penalty;
					penalty_retrieve_num = 0;
				}
				else
				{
					penalty = Penalty - Penalty * (i / double(Max_iterate));
				}

				inter_tour_exchange_nearest100(999, penalty);
				inter_tour_exchange_nearest_all(penalty);
				inter_tour_exchange1(inter_two_para, penalty);
				inter_tour_exchange_two_points(inter_two_para, penalty);
			}

			if (i < 6000)
			{
				within_tour_exchange1();
				inter_tour_exchange_nearest_all(0);
			}
			else {
				within_tour_exchange2(penalty);
			}
			auto iter = solution.begin();
			while (iter != solution.end())
			{
				if ((*iter).path.size() == 2 || ((*iter).path.size() == 3 && (*iter).path[1]>data.cus_num)) {
					iter = solution.erase(iter);
				}
				else {
					++iter;
				}
			}
			double tempcost1 = compute_all_costs();
			if (tempcost1 < best_cost) {
				best_cost = tempcost1;
				updata_best_solu();
			}
			temp_cost = tempcost1;
			std::cout << "after_points exchange cost:" << temp_cost << endl;
		}
		if (i % 200 == 0)
		{
			clock_t start2 = clock();
			for (auto k = 0; k < 300; k++)
			{
				//double temp_cost = best_cost;
				vector <int> temp_list;
				solution[k%solution.size()].remove_flag = true;
				extend_vec<int>(temp_list, solution[k%solution.size()].changed_path);
				for (auto &item : temp_list)
				{
					if (item > 0 && item <= data.cus_num)
						nodes_unexplored.insert(item);
				}
				ci(2);
			}
			double tempcost = compute_all_costs();
			if (tempcost < best_cost) {
				best_cost = tempcost;
				updata_best_solu();
			}
			std::cout << "after route removal  cost:" << temp_cost << endl;
		}
		if (i % 35 == 0 && i>6000)
		{
			clock_t start3 = clock();

			cr_point(0);

			double tempcost = compute_all_costs();
			if (tempcost < best_cost) {
				best_cost = tempcost;
				updata_best_solu();
			}
			inter_tour_exchange_nearest_all(penalty);
			std::cout << "after_cr_points exchange cost:" << temp_cost << endl;
		}
		cr(2);
		ci(2);
	}

	for (int k = 0; k < 100; k++) {
		cr(2);
		ci(2);
	}

	clock_t start2 = clock();
	for (auto k = 0; k < 300; k++)
	{
		//double temp_cost = best_cost;
		vector <int> temp_list;
		solution[k%solution.size()].remove_flag = true;
		extend_vec<int>(temp_list, solution[k%solution.size()].changed_path);
		for (auto &item : temp_list)
		{
			if (item > 0 && item <= data.cus_num)
				nodes_unexplored.insert(item);
		}
		ci(2);
	}
	double tempcost = compute_all_costs();
	if (tempcost < best_cost) {
		best_cost = tempcost;
		updata_best_solu();
	}

	sr_all();
	si_all();
	std::cout << "best cost:" << best_cost << endl;
	std::cout << "final cost:" << compute_all_costs() << endl;
	tempcost = compute_all_costs();
	if (tempcost > best_cost) {
		reset_best_solution();
	}
	charge_dispose();
	/* data output */
	{
		clock_t end = clock();
		std::cout << "compute_time:" << end - start << endl;
		std::cout << compute_all_costs();
		print_vec(costs_per_thousand);
		double time = (end - start) / double(1000);
		time = floor((time - int(time)) * 10) / 10 + int(time);
		string filename1 = "Result_" + numTostr<int>(cluster_num) + "_" + numTostr<double>(time) + ".csv";
		ofstream outfile(filename1.c_str(), ios::out);
		outfile << "trans_code" << ',' << "vehicle_type" << ',' << "dist_seq" << ',' << "distribute_lea_tm" << ',' << "distribute_arr_tm" << ',' << "distance" << ','
			<< "trans_cost" << ',' << "charge_cost" << ',' << "wait_cost" << ',' << "fixed_use_cost" << ',' << "total_cost" << ',' << "charge_cnt" << endl;
		int assgin_veichle_num = 0;
		for (auto &item : solution)
		{
			assgin_veichle_num++;
			int veichle_type = item.veichle_type;
			if (veichle_type == 1)
			{
				capacity_m = 2.0; capacity_v = 12.0; length = 100000; fixed = 200; cost_per_meter = 0.012;
			}
			else if (veichle_type == 2)
			{
				capacity_m = 2.5; capacity_v = 16.0; length = 120000; fixed = 300; cost_per_meter = 0.014;
			}
			int temp_dist = 0;
			int temp_time = 480;
			while (1)
			{
				if (print_feasiable(item.path, temp_time)) {
					temp_time++;
				}
				else {
					temp_time--;
					break;
				}
			}
			int stime = temp_time;
			int wait_time = 0;
			int charge_count = 0;
			temp_time -= 30;
			for (auto i = 0; i < item.path.size() - 1; i++)
			{
				if (item.path[i] > data.cus_num) {
					charge_count++;
				}
				temp_dist += data.dists[item.path[i]][item.path[i + 1]];
				int temp = data.dists_time[item.path[i]][item.path[i + 1]] + 30 + temp_time;
				if (temp < data.demand_tw0[item.path[i + 1]])
				{
					temp_time = data.demand_tw0[item.path[i + 1]];
					wait_time += data.demand_tw0[item.path[i + 1]] - temp;
				}
				else
				{
					temp_time = temp;
				}
			}
			double total_cost = temp_dist * cost_per_meter + fixed + charge_count * charge_cost_per + wait_time * wait_per_min;
			for (auto k = 1; k<item.path.size() - 1; k++)
			{
				item.path[k] += 10000 * cluster_num;
			}
			string route_print;
			for (auto k = 0; k<item.path.size(); k++)
			{
				if (k > 0) {
					route_print = route_print + ";" + numTostr<int>(item.path[k]);
				}
				else {
					route_print = route_print + numTostr<int>(item.path[k]);
				}
			}
			string assgined_num;
			if (assgin_veichle_num < 10)
			{
				assgined_num = "DP000" + numTostr<int>(assgin_veichle_num);
			}
			else if (assgin_veichle_num < 100)
			{
				assgined_num = "DP00" + numTostr<int>(assgin_veichle_num);
			}
			else {
				assgined_num = "DP0" + numTostr<int>(assgin_veichle_num);
			}
			outfile << setiosflags(ios::fixed) << setprecision(2);
			outfile << assgined_num << ',' << veichle_type << ',' << route_print << ',' << get_time_back(stime) << ',' << get_time_back(temp_time) << ',' << temp_dist << ','
				<< temp_dist * cost_per_meter << ',' << charge_count * charge_cost_per << ',' << wait_time * wait_per_min << ',' << fixed << ',' << total_cost << ',' << charge_count << endl;
		}
		outfile.close();
	}

	for (size_t i = 0; i < data.total_num; i++)
	{
		delete[] data.dists[i];
		delete[] data.dists_time[i];
	}
	delete[] data.dists;
	delete[] data.dists_time;
	//std::system("pause");
}