#pragma once
#pragma GCC optimize(3, "Ofast", "inline")
#include <iostream>
#include <vector>
#include <unordered_map>
#include <cmath>
#include "geo_hash.h"
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
            Tertiary, Secondary, Primary, Trunk, Motorway, Footway,
            Cycleway        
        };
        int Way_id;
        int max_speed;
        Type Way_type;
        bool is_oneway = false;
    };

    
    Map_model(const char* map_path);

    auto &Nodes() { return Nodes_list; }
    auto &Ways() { return Ways_list; }
    auto &Roads() { return Roads_list; }
    auto &Foot_Ways() { return Foot_Way_list; }
    auto &Cycle_Ways() { return Cycle_Way_list; }
    auto &Resturant_id() { return Resturant_id_list; }
    auto &Fuel_id() { return fuel_id_list; }
    auto &Bus_stop_id() { return Bus_stop_id_list; }
    auto &Parking_id() { return Parking_id_list; }

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

    void search_near(int mode, double lat, double lon);
    void search_near(GeoHashTire& Tire, unordered_multimap<string, int>& map, double lat, double lon);
    void addpoint(GeoHashTire::Tire* tire, unordered_multimap<string, int>& map);

    void display_buffer(string &send_txt){
        for(auto i : buffer){
            send_txt+=to_string(Nodes_list[i].Lon) + " " + to_string(Nodes_list[i].Lat) + " ";
        }
        send_txt+=to_string(buffer.size());
    }

private:
    void Load_data(const char* map_path);

    vector<Node> Nodes_list;
    vector<Way> Ways_list;
    vector<Road> Roads_list;
    vector<Road> Foot_Way_list;
    vector<Road> Cycle_Way_list;

    unordered_multimap<string, int> Resturant_id_list;
    GeoHashTire Tire_of_Resturant;

    unordered_multimap<string, int> fuel_id_list;
    GeoHashTire Tire_of_fuel;

    unordered_multimap<string, int> Bus_stop_id_list;
    GeoHashTire Tire_of_Bus_stop;

    unordered_multimap<string, int> Parking_id_list;
    GeoHashTire Tire_of_Parking;

    unordered_set<int> buffer{};
};

