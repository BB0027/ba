#pragma GCC optimize(2)
#include "graph_model.h"

Graph_model::Graph_model(const char* map_path) : Map_model(map_path){
    cout<<"Graph model init begin"<<endl;
    //int loop = 0;
    for(auto& road : Roads()){
        //cout<<"road id:"<<road.Way_id<<endl;
        //cout<<"road type:"<<road.Way_type<<endl;
        //cout<<"road node size:"<<Ways()[road.Way_id].Node_ids.size()<<endl;
        //cout<<Ways()[road.Way_id].Node_ids[0]<<" ";
        //cout<<Ways()[road.Way_id].Node_ids[Ways()[road.Way_id].Node_ids.size() - 1]<<endl;
        if(road.is_oneway == false){
            for(size_t i = 0; i < Ways()[road.Way_id].Node_ids.size(); i++){
                //loop++;
                //if(loop % 10000 == 0){
                    //cout<<"loop:"<<loop<<endl;
                    //cout<<"bucket size:"<<node_neighbors_distance_list.bucket_count()<<endl;
                    //cout<<"load factor:"<<node_neighbors_distance_list.load_factor()<<endl;
                //}
                auto& node_id = Ways()[road.Way_id].Node_ids[i];
                //cout<<"node id:"<<node_id<<" ";
                //cout<<"neighbor size:"<<node_neighbors_list[node_id].size()<<" ";
                if(i != 0){  
                    node_neighbors_list[node_id].push_back(Ways()[road.Way_id].Node_ids[i - 1]);
                    node_neighbors_distance_list[make_pair(node_id, Ways()[road.Way_id].Node_ids[i - 1])] = distance(node_id,Ways()[road.Way_id].Node_ids[i - 1]);
                    //cout<<"hit 1"<<" ";
                }
                if(i != Ways()[road.Way_id].Node_ids.size() - 1){
                    node_neighbors_list[node_id].push_back(Ways()[road.Way_id].Node_ids[i + 1]);
                    node_neighbors_distance_list[make_pair(node_id, Ways()[road.Way_id].Node_ids[i + 1])] = distance(node_id,Ways()[road.Way_id].Node_ids[i + 1]);
                    //cout<<"node id:"<<node_id<<" ";
                    //cout<<"neighbor id:"<<Ways()[road.Way_id].Node_ids[i + 1]<<" ";
                    //cout<<"node id Lon:"<<Nodes()[node_id].Lon<<" ";
                    //cout<<"node id Lat:"<<Nodes()[node_id].Lat<<" ";
                    //cout<<"neighbor id Lon:"<<Nodes()[Ways()[road.Way_id].Node_ids[i + 1]].Lon<<" ";
                    //cout<<"neighbor id Lat:"<<Nodes()[Ways()[road.Way_id].Node_ids[i + 1]].Lat<<" ";
                    //cout<<"distance:";
                    //cout<<distance(node_id,Ways()[road.Way_id].Node_ids[i + 1])<<" ";
                    //cout<<node_neighbors_distance_list[make_pair(node_id, Ways()[road.Way_id].Node_ids[i + 1])]<<endl;
                    //cout<<"hit 2"<<" ";
                }
                //cout<<"neighbor size:"<<node_neighbors_list[node_id].size()<<" ";
                //cout<<endl;
            }
        }
        else{
            for(size_t i = 0; i < Ways()[road.Way_id].Node_ids.size(); i++){
                auto& node_id = Ways()[road.Way_id].Node_ids[i];
                if(i != Ways()[road.Way_id].Node_ids.size() - 1){
                    node_neighbors_list[node_id].push_back(Ways()[road.Way_id].Node_ids[i + 1]);
                    node_neighbors_distance_list[make_pair(node_id, Ways()[road.Way_id].Node_ids[i + 1])] = distance(node_id,Ways()[road.Way_id].Node_ids[i + 1]);
                }
            }
        }
    }
    //cout<<"loop:"<<loop<<endl;
    for(size_t node = 0; node < Nodes().size(); node++){
        if(node_neighbors_list.find(node) != node_neighbors_list.end()){
            node_with_neighbor.push_back(node);
        }
    }
    cout<<"Graph model init end"<<endl;
    cout<<"node_neighbors_list size:"<<node_neighbors_list.size()<<endl;  
}

int Graph_model::find_closest_node(float lon, float lat){
    int closest_node_id = 0;
    float min_distance = 1000000000.f;
    for(size_t i : node_with_neighbor){
        float distance = this->distance(i, lon, lat);
        if(distance < min_distance){
            //cout<<i<<" ";
            min_distance = distance;
            closest_node_id = i;
        }
    }
    return closest_node_id;
}