#pragma GCC optimize(3, "Ofast", "inline")
#include <iostream>
#include <istream>
#include <string>
#include "planner.h"
#include "graph_model.h"
#include <crow.h>
using namespace std;

vector<double> Parsenumber(const string& data){
    vector<double> result;
    istringstream iss(data);
    string word;
    while(getline(iss,word,' ')){
        //cout<<word<<endl;
        double number= stod(word);
        result.push_back(number);
    }
    return result;
}

void search(Route_planner& route_planner, float start_Lon, float start_Lat, float end_Lon, float end_Lat){
        route_planner.reset(start_Lon, start_Lat, end_Lon, end_Lat);
        time_t start_time=clock();
	    route_planner.A_star_search();
	    std::cout << "Distance: " << route_planner.get_distance() << " meters." << std::endl;
        time_t end_time=clock();
        cout << "The search time is: " <<(double)(end_time - start_time) / CLOCKS_PER_SEC << "s" << endl;
}

int main(){
    clock_t start_time=clock();
    //string str = "/mnt/e/Poject/guide/My_guide/testmap/backend/map.osm";
    //string str = "/mnt/e/Poject/guide/My_guide/testmap/backend/map_test";
    string str = "/mnt/e/Poject/guide/My_guide/testmap/frontend/data/map";
    Map_model map(str.c_str());
    cout<<"Min lon: "<<map.Min_lon<<"  ";
    cout<<"Min lat: "<<map.Min_lat<<endl;
    cout<<"Max lon: "<<map.Max_lon<<"  ";
    cout<<"Max lat: "<<map.Max_lat<<endl;
    clock_t end_time=clock();
    cout << "The load time is: " <<(double)(end_time - start_time) / CLOCKS_PER_SEC << "s" << endl;
    Graph_model road_model("drive", map);
    Graph_model footway_model("footway", map);
    Graph_model bicycle_model("bicycle", map);
    Route_planner route_planner(road_model);
    Route_planner footway_planner(footway_model);
    Route_planner bicycle_planner(bicycle_model);
    crow::SimpleApp app;
    //测试
    CROW_WEBSOCKET_ROUTE(app, "/ws")
        .onopen([](crow::websocket::connection& conn) {
           std::cout << "New route planner connection from " << conn.get_remote_ip() << std::endl;
           conn.send_text("Hello, world of route planner...");
       })
       .onclose([](crow::websocket::connection& conn, const std::string& reason, uint16_t with_status_code) {
           std::cout << "Connection closed from " << conn.get_remote_ip() << " with reason: " << reason << std::endl;
       })
       .onmessage([&](crow::websocket::connection& conn, const std::string& data, bool is_binary) {
          std::cout << "Received message from " << conn.get_remote_ip() << ": " << data << std::endl;
          vector<double> lonlatidude=Parsenumber(data);
          float start_Lon=lonlatidude[0];
          float start_Lat=lonlatidude[1];
          float end_Lon=lonlatidude[2];
          float end_Lat=lonlatidude[3];
          int mode=lonlatidude[4];
          cout<<"start_Lon: "<<start_Lon<<" ";
          cout<<"start_Lat: "<<start_Lat<<" ";
          cout<<"end_Lon: "<<end_Lon<<" ";
          cout<<"end_Lat: "<<end_Lat<<endl;
          cout<<"mode is:"<<mode<<endl;
          cout<<"planner init..."<<endl;
          if(mode==1){
              search(route_planner, start_Lon, start_Lat, end_Lon, end_Lat);
              string send_txt;
              route_planner.display_path(send_txt);
              conn.send_text(send_txt);
          }
           else if(mode==2){
              search(footway_planner, start_Lon, start_Lat, end_Lon, end_Lat);
              string send_txt;
              footway_planner.display_path(send_txt);
              conn.send_text(send_txt);
          }
           else if(mode==3){
              search(bicycle_planner, start_Lon, start_Lat, end_Lon, end_Lat);
              string send_txt;
              bicycle_planner.display_path(send_txt);
              conn.send_text(send_txt);
          }
           else{
              cout<<"error"<<endl;
              return;
           }
       });
    CROW_WEBSOCKET_ROUTE(app, "/near")
        .onopen([](crow::websocket::connection& conn) {
           std::cout << "New search near point connection from " << conn.get_remote_ip() << std::endl;
           conn.send_text("Hello, world from near point search...");
       })
       .onclose([](crow::websocket::connection& conn, const std::string& reason, uint16_t with_status_code) {
           std::cout << "Connection closed from " << conn.get_remote_ip() << " with reason: " << reason << std::endl;
       })
     .onmessage([&](crow::websocket::connection& conn, const std::string& data, bool is_binary) {
          std::cout << "Received message from " << conn.get_remote_ip() << ": " << data << std::endl;
          vector<double> lonlatidude=Parsenumber(data);
          float Lon=lonlatidude[0];
          float Lat=lonlatidude[1];
          int mode=lonlatidude[2];
          cout<<"Lon: "<<Lon<<" ";
          cout<<"Lat: "<<Lat<<" ";
          cout<<"mode is:"<<mode<<endl;
          cout<<"search init..."<<endl;
          map.search_near(mode, Lat, Lon);
          string send_txt;
          map.display_buffer(send_txt);
          conn.send_text(send_txt);
       });
    app.port(18080).multithreaded().run();
    return 0;
}