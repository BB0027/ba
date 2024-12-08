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
            int prvcur = 0, cur = 0;
            string road_name;
            string road_message;
            while(!path.empty()){
                prvcur = cur;
                cur = path.front();
                path.pop_front();
                count++;
                //for(auto& name : self_model_.Nodes_road_list[cur]){
                //    cout<<name<<" ";
                //}
                //cout<<endl;
                send_txt += to_string(self_model_.map.Nodes()[cur].Lon) + " " + to_string(self_model_.map.Nodes()[cur].Lat) + " ";
                if(count == 1){
                    road_name = *(self_model_.Nodes_road_list[cur].begin());
                    road_message = getFirstThreeChinese(road_name) + " ";
                }
                else{
                    if(self_model_.Nodes_road_list[cur].find(road_name) == self_model_.Nodes_road_list[cur].end()){
                        //cout<<"not same!"<<road_name<<" ";
                        for(auto& name : self_model_.Nodes_road_list[cur]){
                            if(self_model_.Nodes_road_list[prvcur].find(name) != self_model_.Nodes_road_list[prvcur].end()){
                                //cout<<"hit!"<<name<<endl;
                                road_name = name;
                                road_message += getFirstThreeChinese(road_name) + " ";
                                break;
                            }
                        }
                    }
                    else{
                        //cout<<"same!"<<road_name<<endl;
                    }
                }
            }
            //cout<<road_message<<endl;
            send_txt = to_string(distance_) + " " + to_string(traveltime_)+ " " + to_string(count) + " " + send_txt;
            send_txt += road_message;
            //cout<<send_txt<<endl;
            cout<<"count: "<<count<<endl;
        }

        inline string getFirstThreeChinese(const std::string& str) {
            string result;
            size_t i = 0;
            std::string specific = "无名路"; // 指定的汉字序列
            size_t specificLength = 3; // 指定汉字序列的长度s
            // 检查前specificLength个汉字是否为特定字符串
            bool isSpecific = true;
            for (size_t j = 0; j < specificLength;) {
                if (i + 3 > str.size()) { // 确保不越界
                    isSpecific = false;
                    break;
                }
                // 提取当前位置的汉字
                std::string currentChar = str.substr(i, 3);
                // 比较提取的汉字和特定汉字序列是否一致
                if (currentChar != specific.substr(j, 3)) {
                    isSpecific = false;
                    break;
                }
                j += 3;
                i += 3;
            }

            // 如果是特定的汉字序列，则只保留该序列
            if (isSpecific) {
                result = specific;
            } 
            else{
                result = str;
            }
            return result;
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
        deque<string_view> road_names;
};

