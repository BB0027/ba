#pragma GCC optimize(2)
#include "planner.h"
#include <algorithm>

Route_planner::Route_planner(Graph_model& model) : self_model_(model), openlist_start(cmp(*this)), openlist_end(cmp(*this)){
    // Convert inputs to percentage:
    path.clear();
    distance_ = 0.0f;
}

void Route_planner::reset(float start_Lon, float start_Lat, float end_Lon, float end_Lat){
    path.clear();
    openlist_start.~priority_queue();
    openlist_end.~priority_queue();
    new (&openlist_start) priority_queue<int, vector<int>, cmp>(cmp(*this));
    new (&openlist_end) priority_queue<int, vector<int>, cmp>(cmp(*this));
    parent_node_backward.clear();
    parent_node_forward.clear();
    is_visited_formward.clear();
    is_visited_backward.clear();
    node_infomation.clear();
    distance_ = 0.0f;
    cout<<"reset point begin"<<endl;

    time_t start_time=clock();
    start_node_ = self_model_.find_closest_node(start_Lon, start_Lat);
    node_infomation[start_node_] = {0.0f, calculate_h_value(start_node_,end_node_, end_node_)};
    cout << std::fixed << std::setprecision(7)<<"start_node_lon:"<<self_model_.Nodes()[start_node_].Lon<<" start_node_Lat:"<<self_model_.Nodes()[start_node_].Lat<<endl;
    //std::cout << std::fixed << std::setprecision(7)<<"Lon: "<<start_node_->node->Lon<<" Lat: "<<start_node_->node->Lat<<endl;
    end_node_ = self_model_.find_closest_node(end_Lon, end_Lat);
    node_infomation[end_node_] = {0.0f, calculate_h_value(end_node_,end_node_, end_node_)};
    cout << std::fixed << std::setprecision(7)<<"end_node_lon:"<<self_model_.Nodes()[end_node_].Lon<<" end_node_Lat:"<<self_model_.Nodes()[end_node_].Lat<<endl;
    //std::cout << std::fixed << std::setprecision(7)<<"Lon: "<<end_node_->node->Lon<<" Lat: "<<end_node_->node->Lat<<endl;
    time_t end_time=clock();
    cout << "The reset time is: " <<(double)(end_time - start_time) / CLOCKS_PER_SEC << "s" << endl;
    cout<<"reset point end"<<endl;
}
 

void Route_planner::A_star_search() {
    cout<<"searching..."<<endl;
    openlist_start.push(start_node_);
    openlist_end.push(end_node_);
    is_visited_formward.insert(start_node_);
    is_visited_backward.insert(end_node_);
    clock_t sum_time_of_add_neighbors=0;
    
    while(!openlist_start.empty() && !openlist_end.empty()){
        int current_node_backward = openlist_end.top();
        int current_node_forward = openlist_start.top();

        //cout<<"current_node_forward: "<<current_node_forward<<" current_node_backward: "<<current_node_backward<<endl;
        openlist_start.pop();
        if(is_visited_backward.find(current_node_forward)!= is_visited_backward.end()){
            clock_t start_time_of_construct=clock();
            construct_final_path(current_node_forward);
            clock_t end_time_of_construct=clock();
            cout << "The construct time is: " <<(double)(end_time_of_construct - start_time_of_construct) / CLOCKS_PER_SEC << "s" << endl;
            cout << "The add_neighbors time is: " <<(double)(sum_time_of_add_neighbors) / CLOCKS_PER_SEC << "s" << endl;
            return;
        }
        clock_t start_time=clock();
        add_neighbors_forward(current_node_forward, current_node_forward);
        clock_t end_time=clock();
        sum_time_of_add_neighbors += (end_time - start_time);

        openlist_end.pop();
        if(is_visited_formward.find(current_node_backward)!= is_visited_formward.end()){
            clock_t start_time_of_construct=clock();
            construct_final_path(current_node_backward);
            clock_t end_time_of_construct=clock();
            cout << "The construct time is: " <<(double)(end_time_of_construct - start_time_of_construct) / CLOCKS_PER_SEC << "s" << endl;
            cout << "The add_neighbors time is: " <<(double)(sum_time_of_add_neighbors) / CLOCKS_PER_SEC << "s" << endl;
            return;
        }
        start_time=clock();
        add_neighbors_backward(current_node_forward, current_node_backward);
        end_time=clock();
        sum_time_of_add_neighbors += (end_time - start_time);
    }
    cout<<"error"<<endl;
}

inline void Route_planner::add_neighbors_forward(int current_node_forward, int current_node_backward) {
    //cout<<"add_neighbors begin"<<endl;
    //cout<<"neibors size: "<<self_model_.node_neighbors_list[current_node_forward].size()<<endl;
    for(int neighbor : self_model_.node_neighbors_list[current_node_forward]){
        //cout<<"loop begin"<<endl;
        if(is_visited_formward.find(neighbor) != is_visited_formward.end()){
            //cout<<"hit"<<endl;
            continue;
        }
        parent_node_forward[neighbor] = current_node_forward;
        //cout<<"father_info finished"<<endl;
        int g_value = node_infomation[current_node_forward].first + self_model_.node_neighbors_distance_list[make_pair(current_node_forward, neighbor)];
        //cout<<"g_value finished"<<endl;
        int h_value = calculate_h_value(neighbor, current_node_backward, end_node_); //可以复用
        //cout<<"h_value finished"<<endl;
        node_infomation[neighbor] = {g_value, h_value};
        openlist_start.push(neighbor);
        is_visited_formward.insert(neighbor);        
    }
    //cout<<"add_neighbors end"<<endl;
}

inline void Route_planner::add_neighbors_backward(int current_node_forward, int current_node_backward) {
    //cout<<"add_neighbors begin"<<endl;
    //cout<<"neibors size: "<<self_model_.node_neighbors_list[current_node_backward].size()<<endl;
    for(int neighbor : self_model_.node_neighbors_list[current_node_backward]){
        if(is_visited_backward.find(neighbor)!= is_visited_backward.end()){
            continue;
        }
        parent_node_backward[neighbor] = current_node_backward;
        int g_value = node_infomation[current_node_backward].first + self_model_.node_neighbors_distance_list[make_pair(current_node_backward, neighbor)];
        int h_value = calculate_h_value(neighbor, current_node_forward, start_node_);
        node_infomation[neighbor] = {g_value, h_value};
        openlist_end.push(neighbor);
        is_visited_backward.insert(neighbor);
    }
}

inline float Route_planner::calculate_h_value(int node, int to_node, int goal_node) const {
    //cout<<"calculate_h_value begin"<<endl;
    //cout<<"node Lon: "<<node_info->node->Lon<<" Lat: "<<node_info->node->Lat<<endl;
    //cout<<"end_node Lon: "<<end_node_->node->Lon<<" Lat: "<<end_node_->node->Lat<<endl;
    return self_model_.distance(node, to_node) + self_model_.distance(node, goal_node);
}

void Route_planner::construct_final_path(int current_node){
    distance_ = 0.0f;
    int node_iter = current_node;
    while (node_iter != start_node_)
    {
        path.push_front(node_iter);
        node_iter = parent_node_forward[node_iter];
        //cout<<"node_iter: "<<node_iter<<" "<<"path.front(): "<<path.front()<<endl;
        distance_ += self_model_.node_neighbors_distance_list[make_pair(node_iter, path.front())];     
    }
    path.push_front(start_node_);
    distance_ += self_model_.node_neighbors_distance_list[make_pair(node_iter, path.front())];
    node_iter = current_node;
    while (node_iter != end_node_)
    {
        node_iter = parent_node_backward[node_iter];
        //cout<<"node_iter: "<<node_iter<<" "<<"path.back(): "<<path.back()<<endl;
        distance_ += self_model_.node_neighbors_distance_list[make_pair(node_iter, path.back())];
        path.push_back(node_iter);
    }
}