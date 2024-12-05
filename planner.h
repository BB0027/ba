#pragma once
#pragma GCC optimize(3,"Ofast","inline")
#include <iostream>
#include "graph_model.h"
#include <iomanip>
#include <unordered_set>
#include <queue>
using namespace std;

class Route_planner{
    public:
        Route_planner(Graph_model& model);
        float get_distance() const {return distance_;}
        float get_traveltime() const {return traveltime_;}
        void A_star_search();
        void display_path(string& send_txt){
            int count = 0;
            while(!path.empty()){
                int cur = path.front();
                path.pop_front();
                count++;
                send_txt += to_string(self_model_.map.Nodes()[cur].Lon) + " " + to_string(self_model_.map.Nodes()[cur].Lat) + " ";
            }
            cout<<"count: "<<count<<endl;
        }
        void reset(float start_Lon, float start_Lat, float end_Lon, float end_Lat);

        class cmp{
            private:
                unordered_map<int, pair<float, float> >& node_infomation;
            public:
                cmp(Route_planner& model) : node_infomation(model.node_infomation){}
                bool operator()(int node1, int node2){
                    return node_infomation[node1].first + node_infomation[node1].second > node_infomation[node2].first + node_infomation[node2].second;
                }
        };


    private:
        void add_neighbors_forward(int current_node_forward);
        void add_neighbors_backward(int current_node_backward);
        void construct_final_path(int current_node);
        unordered_map<int, pair<float, float> > node_infomation{};   
        unordered_map<int, int> parent_node_forward{};
        unordered_map<int, int> parent_node_backward{};
        priority_queue<int, vector<int>, cmp> openlist_start;
        priority_queue<int, vector<int>, cmp> openlist_end;
        unordered_set<int> is_visited_formward{};
        unordered_set<int> is_closed_formward{};
        unordered_set<int> is_visited_backward{};
        unordered_set<int> is_closed_backward{};
        Graph_model& self_model_;
        float distance_{};
        float traveltime_{};
        int start_node_{};
        int end_node_{};
        deque<int> path;
};

