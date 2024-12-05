#pragma once
#pragma GCC optimize(3, "Ofast", "inline")
#include <unordered_map>
#include <vector>
#include "geo_hash.h"
#include "map_model.h"

using namespace std;

struct HashFunc
{
	template<typename T, typename U>
	size_t operator()(const std::pair<T, U>& p) const {
		size_t seed = std::hash<T>()(p.first);
		return std::hash<U>()(p.second) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
	}
};

// 键值比较，哈希碰撞的比较定义，需要直到两个自定义对象是否相等
struct EqualKey {
	template<typename T, typename U>
	bool operator ()(const std::pair<T, U>& p1, const std::pair<T, U>& p2) const {
		return p1.first == p2.first && p1.second == p2.second;
	}
};


class Graph_model{

public:
    Graph_model(const string& mode, Map_model& map);
	void Build(vector<Map_model::Road>& data);

	void addpoint(GeoHashTire::Tire* cur, unordered_set<int>& res);
	void search_near_points(float lon, float lat, unordered_set<int>& res, int depth);
	int find_closest_node(float lon, float lat);

    unordered_map<int, vector<int>> node_neighbors_list{};
	unordered_multimap<string, int> hashstr_nodeids_map{};
    unordered_map<pair<int, int>, double, HashFunc, EqualKey> node_neighbors_distance_list{};
	vector<int> node_with_neighbor{};
	Map_model& map;
	GeoHashTire tire_of_neighbor_nodes_roads;
};


// 分别计算出内置类型的 Hash Value 然后对它们进行 Combine 得到一个哈希值
// 一般直接采用移位加异或（XOR）得到哈希值

