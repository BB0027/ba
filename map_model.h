#pragma once
#pragma GCC optimize(2)
#include <iostream>
#include <vector>
#include <unordered_map>
#include <cmath>
#include "tinyxml.h"
#define EARTH_RADIUS 6371.004

using namespace std;

class Map_model
{
public:
    struct Node{
        double Lon = 0.f;
        double Lat = 0.f;
    };

    struct Way{
        vector<int> Node_ids;
    };

    struct Road {
        enum Type {
            Invalid, Unclassified, Service, Residential, Railway,
            Tertiary, Secondary, Primary, Trunk, Motorway, Footway        
        };
        int Way_id;
        Type Way_type;
        bool is_oneway = false;
    };

    
    Map_model(const char* map_path);

    auto &Nodes() { return Nodes_list; }
    auto &Ways() { return Ways_list; }
    auto &Roads() { return Roads_list; }

    static inline float HaverSin(double theta) {
        double v = sin(theta / 2);
        return v * v;
    }

    static inline float ConvertToRadian(double deg) {
        return deg * M_PI / 180;
    }


    inline float distance(int node_index_1, int node_index_2){
        Node node1 = Nodes_list[node_index_1];
        Node node2 = Nodes_list[node_index_2];
        float from_Lon = ConvertToRadian(node1.Lon);
        float from_Lat = ConvertToRadian(node1.Lat);
        float to_Lon = ConvertToRadian(node2.Lon);
        float to_Lat = ConvertToRadian(node2.Lat);
        double vlon = abs(to_Lon - from_Lon);
        double vlat = abs(to_Lat - from_Lat);
        double h = HaverSin(vlat) + cos(from_Lat) * cos(to_Lat) * HaverSin(vlon);
        double distance = 2 * EARTH_RADIUS * asin(sqrt(h));
        distance = distance * 1000;
        return distance;
    }

    inline float distance(int node_index_1, float Lon, float Lat){
        Node node1 = Nodes_list[node_index_1];
        float from_Lon = ConvertToRadian(node1.Lon);
        float from_Lat = ConvertToRadian(node1.Lat);
        float to_Lon = ConvertToRadian(Lon);
        float to_Lat = ConvertToRadian(Lat);
        double vlon = abs(to_Lon - from_Lon);
        double vlat = abs(to_Lat - from_Lat);
        double h = HaverSin(vlat) + cos(from_Lat) * cos(to_Lat) * HaverSin(vlon);
        double distance = 2 * EARTH_RADIUS * asin(sqrt(h));
        distance = distance * 1000;
        return distance;
    }
    double Min_lon = 0.f;
    double Min_lat = 0.f;
    double Max_lon = 0.f;
    double Max_lat = 0.f;
    
private:
    void Load_data(const char* map_path);
    vector<Node> Nodes_list;
    vector<Way> Ways_list;
    vector<Road> Roads_list;
};

