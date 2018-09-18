/************************************************************************************************
Copyright:
Author:jincheng
Date:2018-8-30
Descriptions:
the project is designed to solve the large scale G-VRPTWDC using R&R combined with LS.
this file is the main entry to the project 
************************************************************************************************/

#include"header.h"

int Noise_Maker::max_dist = 0;

int main()
{
	vector <Data> datas;
	vector <Initial> init_solus;
	for (int i{1}; i < 6; i++)
	{
		if (i == 1) {
			srand(75);
		}
		else if (i == 2) {
			srand(50);
		}
		else if (i == 3)
		{
			srand(50);
		}
		else if (i == 4)
		{
			srand(100);
		}
		else if (i == 5)
		{
			srand(115);
		}
		Data data(i);
		//cluster 5: 115 212792
		//cluster 4: 100 
		//cluster 3: 50 316792
		//cluster2:50 311245
		//cluster 1: 95  317236
		/*  get the parameter of noise maker:
		max_dist means the largest distance of two points in a cluster */
		int max_dist1 = 0;
		for (size_t p = 0; p<data.total_num; p++)
			for (size_t q = 0; q < data.total_num; q++)
			{
				if (max_dist1 < data.dists[p][q])
					max_dist1 = data.dists[p][q];
			}
		Noise_Maker::max_dist = max_dist1;

		Initial initial(data,i);
		initial.initial_routes();
		R_R r_r(data, i, initial);
		r_r.run_r_r();
		
	}
	system("pause");
}