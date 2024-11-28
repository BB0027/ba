#pragma GCC optimize(2)
#include"map_model.h"
#include<iostream>
#include"tinyxml.h"
#include<cmath>
#include<algorithm>
#include<string_view>
#include<assert.h>
using namespace std;

Map_model::Map_model(const char* map_path){
    cout<<"Map_model_construct"<<endl;
    Load_data(map_path);
    cout<<"Load_data_success"<<endl;
}

static Map_model::Road::Type Get_type(string_view v){
    if( v == "motorway" )        return Map_model::Road::Motorway;
    if( v == "trunk" )           return Map_model::Road::Trunk;
    if( v == "primary" )         return Map_model::Road::Primary;
    if( v == "secondary" )       return Map_model::Road::Secondary;    
    if( v == "tertiary" )        return Map_model::Road::Tertiary;
    if( v == "residential" )     return Map_model::Road::Residential;
    if( v == "living_street" )   return Map_model::Road::Residential;    
    if( v == "service" )         return Map_model::Road::Service;
    if( v == "unclassified" )    return Map_model::Road::Unclassified;
    //if( v == "footway" )         return Map_model::Road::Footway;
    //if( v == "bridleway" )       return Map_model::Road::Footway;
    //if( v == "steps" )           return Map_model::Road::Footway;
    //if( v == "path" )            return Map_model::Road::Footway;
    //if( v == "pedestrian" )      return Map_model::Road::Footway;    
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
    }
    cout<<"node read success"<<endl;
    //cout<<Nodes_list.size()<<endl;
    int hit_time_highway = 0;
    unordered_map<unsigned long long,unsigned int> Way_id_map;
    for(auto node_w = node->NextSiblingElement("way"); node_w;node_w = node_w->NextSiblingElement("way")){
        Way_id_map[stoll(node_w->Attribute("id"))] = Ways_list.size();
        Ways_list.emplace_back();
        auto &new_way = Ways_list.back();
        for(const TiXmlElement* child_node = node_w->FirstChildElement(); child_node; child_node = child_node->NextSiblingElement()){
            if(child_node->Value() == string("nd")){
                new_way.Node_ids.emplace_back(Node_id_map[stoll(child_node->Attribute("ref"))]);
            }
            else if(child_node->Value() == string("tag")){
                string_view k = child_node->Attribute("k");
                string_view v = child_node->Attribute("v");
                if(k == "highway"){
                    hit_time_highway++;
                    auto way_type = Get_type(v);
                    //cout<<"way_type: "<<(int)way_type<<"  "; 
                    if(way_type != Road::Invalid){
                        Roads_list.emplace_back();
                        Roads_list.back().Way_type = way_type;
                        Roads_list.back().Way_id = Way_id_map[stoll(node_w->Attribute("id"))];
                    }
                    /*
                    else{
                        cout<<"invalid way type"<<endl;
                    }
                    */
                }
                else if(k == "oneway"){
                    if(v == "yes"){
                        Roads_list.back().is_oneway = true;
                    }
                    else{
                        Roads_list.back().is_oneway = false;
                    }
                }
            }
        }
    }
    //cout<<endl;
    cout<<"hit time of highways: "<<hit_time_highway<<endl;     
    cout<<"Nodes size: "<<Nodes().size()<<endl;
    cout<<"Ways size: "<<Ways().size()<<endl;  
    cout<<"Roads size: "<<Roads().size()<<endl;
}