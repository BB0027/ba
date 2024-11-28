#pragma GCC optimize(2)
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
    std::cout<<"hello_tinyxml"<<endl;
    //string str = "E:\\Poject\\guide\\My_guide\\testmap\\my-app\\data\\map";
    string str = "/mnt/e/Poject/guide/My_guide/testmap/backend/map_test";
    //string str = "E:\\Poject\\guide\\My_guide\\testmap\\my-app\\backend\\map.osm";
    //Route_model map(str.c_str());
    cout<<"graph init"<<endl;
    Graph_model map(str.c_str());
    cout<<"Min lon: "<<map.Min_lon<<endl;
    cout<<"Min lat: "<<map.Min_lat<<endl;
    cout<<"Max lon: "<<map.Max_lon<<endl;
    cout<<"Max lat: "<<map.Max_lat<<endl;
    clock_t end_time=clock();
    cout << "The load time is: " <<(double)(end_time - start_time) / CLOCKS_PER_SEC << "s" << endl;
    Route_planner route_planner(map);

    crow::SimpleApp app;
    //测试
    CROW_WEBSOCKET_ROUTE(app, "/ws")
        .onopen([](crow::websocket::connection& conn) {
           std::cout << "New connection from " << conn.get_remote_ip() << std::endl;
           conn.send_text("Hello, world!");
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
          cout<<"start_Lon: "<<start_Lon<<" ";
          cout<<"start_Lat: "<<start_Lat<<" ";
          cout<<"end_Lon: "<<end_Lon<<" ";
          cout<<"end_Lat: "<<end_Lat<<endl;
          cout<<"planner init..."<<endl;
          search(route_planner, start_Lon, start_Lat, end_Lon, end_Lat);
          string send_txt;
          route_planner.display_path(send_txt);
          conn.send_text(send_txt);
       });

    app.port(18080).multithreaded().run();
    return 0;
}