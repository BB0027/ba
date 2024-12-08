#pragma GCC optimize(3, "Ofast", "inline")
#include"map_model.h"
#include<iostream>
#include"tinyxml.h"
#include<cmath>
#include<algorithm>
#include<string_view>
#include<assert.h>
using namespace std;

Map_model::Map_model(const char* map_path){
    cout<<"--------------Map_model_construct----------------"<<endl;
    Load_data(map_path);
    cout<<"success..."<<endl;
}

static Map_model::Road::Type Get_type(string_view v){
    if( v == "motorway" || v == "motorway_link")        return Map_model::Road::Motorway;
    if( v == "trunk"    || v == "trunk_link")           return Map_model::Road::Trunk;
    if( v == "primary"  || v == "primary_link")         return Map_model::Road::Primary;
    if( v == "secondary"|| v == "secondary_link")       return Map_model::Road::Secondary;    
    if( v == "tertiary" || v == "tertiary_link")        return Map_model::Road::Tertiary;
    if( v == "residential" )     return Map_model::Road::Residential;
    if( v == "living_street" )   return Map_model::Road::Residential;    
    if( v == "service" )         return Map_model::Road::Service;
    if( v == "unclassified" )    return Map_model::Road::Unclassified;
    if( v == "footway" )         return Map_model::Road::Footway;
    if( v == "bridleway" )       return Map_model::Road::Footway;
    if( v == "steps" )           return Map_model::Road::Footway;
    if( v == "path" )            return Map_model::Road::Footway;
    if( v == "pedestrian" )      return Map_model::Road::Footway;    
    if( v == "corridor")         return Map_model::Road::Footway;
    if( v == "sidewalk" )        return Map_model::Road::Footway;
    if( v == "crossing")         return Map_model::Road::Residential;
    if( v  == "cycleway")        return Map_model::Road::Cycleway;
    return Map_model::Road::Invalid;       
}

void Map_model::Load_data(const char* map_path){
    cout<<"Load_File Begin..."<<endl;
    TiXmlDocument doc(map_path);
    bool loadOkay = doc.LoadFile();
    if(!loadOkay){
        throw logic_error("load file fail");
    }
    cout<<"Load_File Success"<<endl;
    cout<<"bounds read begin ..."<<endl;
    TiXmlElement *root = doc.RootElement();
    if(root == NULL){
        throw logic_error("root is null");
    }
    TiXmlElement *node = root->FirstChildElement();
    if(node == NULL){
        throw logic_error("map's bounds is not defined");
    }
    Min_lat = stod(node->Attribute("minlat"));
    Min_lon = stod(node->Attribute("minlon"));
    Max_lat = stod(node->Attribute("maxlat"));
    Max_lon = stod(node->Attribute("maxlon"));
    cout<<"bounds read success"<<endl;
    cout<<"node read begin..."<<endl;
    unordered_map<unsigned long long,unsigned int> Node_id_map;
    for(auto node_nd = node->NextSiblingElement("node"); node_nd;node_nd = node_nd->NextSiblingElement("node")){
        Node_id_map[stoull(node_nd->Attribute("id"))] = Nodes_list.size();
        Nodes_list.emplace_back();  
        Nodes_list.back().Lat = stod(node_nd->Attribute("lat"));
        Nodes_list.back().Lon = stod(node_nd->Attribute("lon"));
        for(const TiXmlElement* child_node = node_nd->FirstChildElement(); child_node; child_node = child_node->NextSiblingElement()){
            //cout<<"hit"<<endl;
            int id = Nodes_list.size();
            if(child_node->Value() == string("tag")){
                string_view k = child_node->Attribute("k");
                string_view v = child_node->Attribute("v");
                if(k == "amenity"){
                    if(v == "bar" || v == "cafe" || v == "fast_food" || v == "food_court" || v == "ice_cream" || v == "pub" || v == "restaurant"){
                        string hashstr = Tire_of_Resturant.geohash.encode(Nodes_list.back().Lat,Nodes_list.back().Lon);
                        Tire_of_Resturant.insert(hashstr);
                        Resturant_id_list.insert({hashstr, id});
                    }
                    else if(v == "parking" || v == "parking_entrance" || v == "parking_space"){
                        string hashstr = Tire_of_Parking.geohash.encode(Nodes_list.back().Lat,Nodes_list.back().Lon);
                        Tire_of_Parking.insert(hashstr);
                        Parking_id_list.insert({hashstr,id});
                    }
                    else if(v == "fuel" || v == "charge_station"){
                        string hashstr = Tire_of_fuel.geohash.encode(Nodes_list.back().Lat,Nodes_list.back().Lon);
                        Tire_of_fuel.insert(hashstr);
                        fuel_id_list.insert({hashstr,id});
                    }
                    else if(v == "bus_staion"){
                        string hashstr = Tire_of_Bus_stop.geohash.encode(Nodes_list.back().Lat,Nodes_list.back().Lon);
                        Tire_of_Bus_stop.insert(hashstr);
                        Bus_stop_id_list.insert({hashstr,id});
                        
                    }
                }
                if(k == "highway"){
                    if(v == "bus_stop"){
                        string hashstr = Tire_of_Bus_stop.geohash.encode(Nodes_list.back().Lat,Nodes_list.back().Lon);
                        Tire_of_Bus_stop.insert(hashstr);
                        Bus_stop_id_list.insert({hashstr,id});
                    }
                    if(v == "service"){
                        string hashstr = Tire_of_fuel.geohash.encode(Nodes_list.back().Lat,Nodes_list.back().Lon);
                        Tire_of_fuel.insert(hashstr);
                        fuel_id_list.insert({hashstr,id});
                    }
                }
                if(k == "highway"){
                    if(v == "stop_postion" || v == "staion"){
                        string hashstr = Tire_of_Bus_stop.geohash.encode(Nodes_list.back().Lat,Nodes_list.back().Lon);
                        Tire_of_Bus_stop.insert(hashstr);
                        Bus_stop_id_list.insert({hashstr,id});
                    }
                }
            }
        }
    }
    cout<<"node read success...size is: "<<Nodes_list.size()<<endl;
    int hit_time_highway = 0;
    unordered_map<unsigned long long,unsigned int> Way_id_map;
    int none_index = 0;
    for(auto node_w = node->NextSiblingElement("way"); node_w;node_w = node_w->NextSiblingElement("way")){
        Way_id_map[stoll(node_w->Attribute("id"))] = Ways_list.size();
        //cout<<"Way_id: "<<node_w->Attribute("id")<<endl;
        Ways_list.emplace_back();
        auto &new_way = Ways_list.back();
        bool isroad = false, isfootway = false, iscycleway = false;
        bool road_is_oneway = false, cycleway_is_oneway = false;
        int road_max_speed = 0;
        string road_name = "";
        enum Map_model::Road::Type way_type = Map_model::Road::Unclassified;
        //cout<<"Way read begin..."<<endl;
        for(const TiXmlElement* child_node = node_w->FirstChildElement(); child_node; child_node = child_node->NextSiblingElement()){
            //cout<<"hit"<<endl;
            if(child_node->Value() == string("nd")){
                new_way.Node_ids.emplace_back(Node_id_map[stoll(child_node->Attribute("ref"))]);
                //cout<<"Node_id: "<<Node_id_map[stoll(child_node->Attribute("ref"))]<<"  ";
                continue;
            }
            else if(child_node->Value() == string("tag")){
                string_view k = child_node->Attribute("k");
                string_view v = child_node->Attribute("v");
                if(k == "name"){
                    road_name = child_node->Attribute("v");
                    //cout<<"Name: "<<v<<"  ";
                }
                if(k == "highway"){
                    hit_time_highway++;
                    way_type = Get_type(v);
                    //cout<<"way_type: "<<(int)way_type<<"  "; 
                    if(way_type != Road::Invalid){
                        if(way_type == Road::Primary  || way_type == Road::Secondary 
                        || way_type == Road::Tertiary || way_type == Road::Residential
                        || way_type == Road::Service  || way_type == Road::Unclassified){
                            isfootway = true;
                            iscycleway = true;
                            isroad = true;
                        }
                        else if (way_type == Road::Trunk || way_type == Road::Motorway){
                            isfootway = false;
                            iscycleway = false;
                            isroad = true;
                        }
                        else if (way_type == Road::Footway){
                            isfootway = true;
                        }
                        else if (way_type == Road::Cycleway){
                            iscycleway = true;
                        }
                    }
                }
                else if(k == "oneway"){
                    if(v == "yes"){
                        road_is_oneway = true;
                    }
                }
                else if(k == "oneway:bicycle"){
                    iscycleway = true;
                    if(v == "yes"){
                        cycleway_is_oneway = true;
                    }
                }
                else if(k == "motorroad"){
                    if(v == "yes"){
                        isroad = true;
                        iscycleway = false;
                        isfootway = false;
                    }
                }
                else if(k == "bicycle_road"){
                    if(v == "yes"){
                        iscycleway = true;
                    }
                }
                else if(k == "sidewalk"){
                    if(v != "no" && v!= "none"){
                        isfootway = true;
                    }
                }
                else if(k == "cycleway"){
                    iscycleway = true;
                }
                else if(k == "maxspeed"){
                        road_max_speed = atoi(child_node->Attribute("v"));
                }
            }
        }
        if(road_name == ""){
            road_name = "无名路"+to_string(none_index++);
        }
        if(isroad){
                Roads_list.emplace_back();
                Roads_list.back().Way_name = road_name;
                Roads_list.back().Way_type = way_type;
                Roads_list.back().Way_id = Way_id_map[stoll(node_w->Attribute("id"))];
                Roads_list.back().is_oneway = road_is_oneway;
                if(road_max_speed == 0){
                    if(way_type == Road::Motorway){
                        Roads_list.back().max_speed = 120;
                    }
                    else if(way_type == Road::Trunk){
                        Roads_list.back().max_speed = 80;
                    }
                    else if(way_type == Road::Primary){
                        Roads_list.back().max_speed = 40;
                    }
                    else if(way_type == Road::Secondary){
                        Roads_list.back().max_speed = 30;
                    }
                    else if(way_type == Road::Tertiary){
                        Roads_list.back().max_speed = 20;
                    }
                    else if(way_type == Road::Residential){
                        Roads_list.back().max_speed = 5;
                    }
                    else if(way_type == Road::Service){
                        Roads_list.back().max_speed = 5;
                    }
                    else if(way_type == Road::Unclassified){
                        Roads_list.back().max_speed = 5;
                    }
                    else{
                        Roads_list.back().max_speed = 5;
                    }
                }
                else{
                    Roads_list.back().max_speed = road_max_speed * 0.7;
                }
                //cout<<"way_name: "<<Roads_list.back().Way_name<<" ";
                //cout<<"max_speed: "<<Roads_list.back().max_speed<<" ";
                //cout<<"way_type: "<<(int)Roads_list.back().Way_type<<endl;
        }
        if(isfootway){
                Foot_Way_list.emplace_back();
                Foot_Way_list.back().Way_name = road_name;
                Foot_Way_list.back().Way_type = Map_model::Road::Footway;
                Foot_Way_list.back().Way_id = Way_id_map[stoll(node_w->Attribute("id"))];
                Foot_Way_list.back().is_oneway = false;
                Foot_Way_list.back().max_speed = 5;
        }
        if(iscycleway){
                Cycle_Way_list.emplace_back();
                Cycle_Way_list.back().Way_name = road_name;
                Cycle_Way_list.back().Way_type = Map_model::Road::Cycleway;
                Cycle_Way_list.back().Way_id = Way_id_map[stoll(node_w->Attribute("id"))];
                Cycle_Way_list.back().is_oneway = cycleway_is_oneway || road_is_oneway;
                Cycle_Way_list.back().max_speed = 15;
        }
    }
    //cout<<endl;
    cout<<"hit time of highways: "<<hit_time_highway<<"   ";     
    cout<<"Nodes size: "<<Nodes().size()<<endl;
    cout<<"Ways size: "<<Ways().size()<<"   ";  
    cout<<"Roads size: "<<Roads().size()<<endl;
    cout<<"Foot_Way size: "<<Foot_Way_list.size()<<"   ";
    cout<<"Cycle_Way size: "<<Cycle_Way_list.size()<<endl;
    cout<<"Resturant size: "<<Resturant_id_list.size()<<" ";
    cout<<"Parking size: "<<Parking_id_list.size()<<endl;
    cout<<"fuel size: "<<fuel_id_list.size()<<" ";
    cout<<"Bus_stop size: "<<Bus_stop_id_list.size()<<endl;
}

void Map_model::search_near(int mode, double lat, double lon){
    buffer.clear();
    time_t start = clock();
    cout<<"search_near begin..."<<endl;
    buffer.clear();
    if(mode == 1){
        search_near(Tire_of_Resturant, Resturant_id_list, lat, lon);
    }
    else if(mode == 2){
        search_near(Tire_of_Parking, Parking_id_list, lat, lon);
    }
    else if(mode == 3){
        search_near(Tire_of_fuel, fuel_id_list, lat, lon);
    }
    else if(mode == 4){
        search_near(Tire_of_Bus_stop, Bus_stop_id_list, lat, lon);
    }
    time_t end = clock();
    cout<<"search_near time: "<<(double)(end - start) / CLOCKS_PER_SEC << "s" << endl;
    cout<<"search_near size: "<<buffer.size()<<endl;
}

void Map_model::search_near(GeoHashTire& Tire, unordered_multimap<string, int>& id_list, double lat, double lon){
    vector<string> hashstr_list = Tire.geohash.get_neighbors(lat, lon);
    for(auto& hashstr : hashstr_list){
        GeoHashTire::Tire* cur = Tire.search(hashstr, 5);
        addpoint(cur, id_list);
    }
}

void Map_model::addpoint(GeoHashTire::Tire* cur, unordered_multimap<string, int>& id_list){
    if(cur == nullptr){
        return;
    }
    if(cur->is_end){
        auto range = id_list.equal_range(cur->nodestr);
        for(auto it = range.first; it!= range.second; ++it){
            buffer.emplace(it->second);
        }
    }
    else{
        for(int i = 0; i < 32; ++i){
            addpoint(cur->child[i], id_list);
        }
    }
}