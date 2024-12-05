#pragma GCC optimize(3, "Ofast", "inline")
#include "graph_model.h"

Graph_model::Graph_model(const string& mode, Map_model& map) : map(map){
    cout<<"------------Graph model init begin------------"<<endl;
    cout<<"mode:  "<<mode<<endl;
    if(mode == "drive"){
        average_speed_ = 15;
        Build(map.Roads());  
    }
    else if(mode == "footway"){
        average_speed_ = 5;
        Build(map.Foot_Ways());  
    }
    else{
        average_speed_ = 15;
        Build(map.Cycle_Ways());
    }
}

void Graph_model::Build(vector<Map_model::Road>& Roads){
    cout<<"build begin"<<endl;
    auto& Ways = map.Ways();
    for(auto& road : Roads){
        //cout<<"road id:"<<road.Way_id<<endl;
        //cout<<"road type:"<<road.Way_type<<endl;
        //cout<<"road node size:"<<Ways()[road.Way_id].Node_ids.size()<<endl;
        //cout<<Ways()[road.Way_id].Node_ids[0]<<" ";
        //cout<<Ways()[road.Way_id].Node_ids[Ways()[road.Way_id].Node_ids.size() - 1]<<endl;
        if(road.is_oneway == false){
            for(size_t i = 0; i < Ways[road.Way_id].Node_ids.size(); i++){
                //loop++;
                //if(loop % 10000 == 0){
                    //cout<<"loop:"<<loop<<endl;
                    //cout<<"bucket size:"<<node_neighbors_distance_list.bucket_count()<<endl;
                    //cout<<"load factor:"<<node_neighbors_distance_list.load_factor()<<endl;
                //}
                auto& node_id = Ways[road.Way_id].Node_ids[i];
                //cout<<"node id:"<<node_id<<" ";
                //cout<<"neighbor size:"<<node_neighbors_list[node_id].size()<<" ";
                if(i != 0){  
                    node_neighbors_list[node_id].push_back(Ways[road.Way_id].Node_ids[i - 1]);
                    node_neighbors_distance_list[make_pair(node_id, Ways[road.Way_id].Node_ids[i - 1])] = map.distance(node_id,Ways[road.Way_id].Node_ids[i - 1]);
                    node_neighbors_traveltime_list_[make_pair(node_id, Ways[road.Way_id].Node_ids[i - 1])] = 60*map.distance(node_id,Ways[road.Way_id].Node_ids[i - 1])/(road.max_speed*1000);
                    //cout<<"hit 1"<<" ";
                }
                if(i != Ways[road.Way_id].Node_ids.size() - 1){
                    node_neighbors_list[node_id].push_back(Ways[road.Way_id].Node_ids[i + 1]);
                    node_neighbors_distance_list[make_pair(node_id, Ways[road.Way_id].Node_ids[i + 1])] = map.distance(node_id,Ways[road.Way_id].Node_ids[i + 1]);
                    node_neighbors_traveltime_list_[make_pair(node_id, Ways[road.Way_id].Node_ids[i + 1])] = 60*map.distance(node_id,Ways[road.Way_id].Node_ids[i + 1])/(road.max_speed*1000);
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
            for(size_t i = 0; i < Ways[road.Way_id].Node_ids.size(); i++){
                auto& node_id = Ways[road.Way_id].Node_ids[i];
                if(i != Ways[road.Way_id].Node_ids.size() - 1){
                    node_neighbors_list[node_id].push_back(Ways[road.Way_id].Node_ids[i + 1]);
                    node_neighbors_distance_list[make_pair(node_id, Ways[road.Way_id].Node_ids[i + 1])] = map.distance(node_id,Ways[road.Way_id].Node_ids[i + 1]);
                    node_neighbors_traveltime_list_[make_pair(node_id, Ways[road.Way_id].Node_ids[i + 1])] = 60*map.distance(node_id,Ways[road.Way_id].Node_ids[i + 1])/(road.max_speed*1000);
                }
            }
        }
    }
    //cout<<"loop:"<<loop<<endl;
    auto& Nodes = map.Nodes();
    for(size_t node = 0; node < Nodes.size(); node++){
        if(node_neighbors_list.find(node) != node_neighbors_list.end()){
            node_with_neighbor.push_back(node);
            //cout<<"insert..."<<endl;
            string hashstr = tire_of_neighbor_nodes_roads.geohash.encode(Nodes[node].Lat, Nodes[node].Lon);
            tire_of_neighbor_nodes_roads.insert(hashstr);
            hashstr_nodeids_map.insert(make_pair(hashstr, node));
        }
    }
    cout<<"node_neighbors_list size:"<<node_neighbors_list.size()<<endl;  
}

void Graph_model::addpoint(GeoHashTire::Tire* cur, unordered_set<int>& res){
    if(cur == NULL){
        return;
    }
    if(cur->is_end){
        auto range = hashstr_nodeids_map.equal_range(cur->nodestr);
        for(auto it = range.first; it!= range.second; it++){
            //cout<<it->first<<" "<<it->second<<endl;
            res.insert(it->second);
        }
    }
    else{
        for(int i = 0; i < 32; i++){
            addpoint(cur->child[i], res);
        }
    }
}

void Graph_model::search_near_points(float lon, float lat, unordered_set<int>& res, int depth){
    vector<string> neighbors = tire_of_neighbor_nodes_roads.geohash.get_neighbors(lat, lon);
    //cout<<neighbors.size()<<endl;
    for(auto& neighborstr : neighbors){
        GeoHashTire::Tire* cur = tire_of_neighbor_nodes_roads.search(neighborstr, depth);
        //cout<<cur<<endl;
        addpoint(cur, res);
    }
}

int Graph_model::find_closest_node(float lon, float lat){
    time_t start_time=clock();
    unordered_set<int> nearpoint;
    search_near_points(lon, lat, nearpoint, 6);
    //cout<<nearpoint.size()<<endl;
    int closet_node_id_1 = 0;
    float min_distance_1 = 1000000000.f;
    for(size_t i : nearpoint){
        float distance = map.distance(i, lon, lat);
        if(distance < min_distance_1){
            //cout<<i<<" ";
            min_distance_1 = distance;
            closet_node_id_1 = i;
        }
    }
    time_t end_time=clock();
    cout<<"find_closest_node time:"<<(end_time-start_time)/CLOCKS_PER_SEC<<"s"<<endl;
    /*
    start_time=clock();
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
    cout<<closest_node_id<<" "<<closet_node_id_1<<endl;
    end_time=clock();
    cout<<"find_closest_node time:"<<(end_time-start_time)<<endl;
    */
    return closet_node_id_1;
}