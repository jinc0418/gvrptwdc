/************************************************************************************************
Copyright:
Author:jincheng
Date:2018-8-30
Descriptions:
The project is designed to solve the large scale G-VRPTWDC using R&R combined with LS.
This file includes some function templates which helps in other programs
************************************************************************************************/
#include"header.h"

/* transfer string to num or num to string using stringstream class */
template <class Type>
Type stringToNum(const string& str)
{
	istringstream iss(str);
	Type num;
	iss >> num;
	return num;
}
template <class Type>
string numTostr(Type i)
{
	stringstream ss;
	ss << i;
	return ss.str();
}

/* the argsort function return the ascending order of a vector */
template<typename T>
vector<int> argsort(const vector<T>& a) {

	int Len = a.size();

	vector<int> idx(Len, 0);

	for (int i = 0; i < Len; i++) {

		idx[i] = i;

	}

	sort(idx.begin(), idx.end(), [&a](int i1, int i2) {return a[i1] < a[i2]; });

	return idx;

}

/* judge whether a item is in a vectgor or not 
 using find()!=vector.end() is an substitute*/
template <class Type>
bool item_in_vectors(Type v, vector<Type> & vs)
{
	for (size_t i = 0; i < vs.size(); i++)
	{
		if (v == vs[i])
		{
			return true;
		}
	}
	return false;
}

/* count how many times an item occurred in a vector */
template <class Type>
int num_vec_in_vecs(Type v, vector<Type> vs)
{
	int count = 0;
	for (size_t i = 0; i < vs.size(); i++)
	{
		if (v == vs[i])
		{
			count++;
		}
	}
	return count;
}

/* judge whether a set is included in another set */
template <class Type>
int set_in_sets(vector <Type> set, vector <Type> sets)
{
	for (size_t i = 0; i < set.size(); i++)
	{
		int flag1 = 0;
		for (size_t j = 0; j < sets.size(); j++)
		{
			if (set[i] == sets[j])
			{
				flag1 = 1;
				break;
			}
		}
		if (flag1 == 0)
		{
			return 0;
		}
	}
	return 1;
}

/* updata a set with items in a vector */
template <class Type>
void add_vec2set(vector<Type> &vec, set<Type> &aset)
{
	for (size_t i = 0; i < vec.size(); i++)
	{
		aset.insert(vec[i]);
	}
}

/* return the index of a certain item in a vector */
template <class Type>
int vec_index_in_vecs(Type &v, vector<Type> &vec)
{
	for (size_t i = 0; i < vec.size(); i++)
	{
		if (v == vec[i])
			return i;
	}
}

/* print the items of a vector to the std */
template<class Type>
void print_vec(Type &vec)
{
	for (auto item : vec)
	{
		cout << item<<"->";
	}
	cout << endl;
}

/* print the item of a set to the std */
template<class Type>
void print_set(Type &vec)
{
	for (auto item : vec)
	{
		cout << item << "->";
	}
	cout << endl;
}

/* the same as python extend */
template<class Type>
void extend_vec(vector<Type>&a, vector<Type> &b)//put b to the end of a
{
	for (auto item : b)
		a.push_back(item);
}
