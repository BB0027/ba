#pragma once
#pragma GCC optimize(3, "Ofast", "inline")
#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <cmath>
#include <bitset>
#include <unordered_set>
#include <iomanip>

using namespace std;

class GeoHashCoder
{
public:
	double MINLAT = -90;
	double MAXLAT = 90;
	double MINLNG = -180;
	double MAXLNG = 180;
	static const int bitlen = 5*5;
    string base_digit = "0123456789bcdefghjkmnpqrstuvwxyz";
	map<char, int> digit_to_int;

	double pixel_lon;
	double pixel_lat;

	GeoHashCoder(){
		int i = 0;
		for(char c : base_digit){
			digit_to_int[c] = i;
			//cout<<c<<" "<<base_digit[i]<<endl;
			i++;
		}
		pixel_lat = (MAXLAT - MINLAT)/pow(2,bitlen);
		pixel_lon = (MAXLNG - MINLNG)/pow(2,bitlen);
		//cout<<pixel_lat<<" "<<pixel_lon<<endl;
	}

	string encode(double lat, double lon){
		//cout<<setprecision(18)<<lat<<" "<<lon<<endl;
		vector<bool> latbits(bitlen);
		getbits(latbits,lat, -90, 90);
		vector<bool> lonbits(bitlen);
		getbits(lonbits,lon, -180, 180);
		vector<bool> res;
		//cout<<"get_bits"<<endl;
		for(int i = 0; i < bitlen; i++){
			//cout<<i<<" "<<latbits[i]<<" "<<lonbits[i]<<endl;
			res.push_back(lonbits[i]);
			res.push_back(latbits[i]);
		}
		string resstr = "";
		for(int i = 0; i < bitlen*2/5; i++){
			int tmp = 0;
			for(int j = 0; j < 5; j++){
				tmp = tmp*2 + res[i*5+j];
			}
			resstr += base_digit[tmp];
		}
		//cout<<setprecision(10)<<lat<<" "<<lon<<endl;
		return resstr;
	}

	void getbits(vector<bool>& res, double val, double min, double max){
		for(int i = 0; i < bitlen; i++){
			double mid = (min + max)/2;
			if(val >= mid){
				res[i] = 1;
				min = mid;
			}
			else{
				res[i] = 0;
				max = mid;
			}
			//cout<<setprecision(10)<<val<<" "<<"min: "<<min<<"max: "<<max<<"mid: "<<mid<<" "<<"i: "<<i<<" "<<res[i]<<endl;
		}
	}

	vector<string> get_neighbors(double lat, double lon){
		vector<string> res;
		double uplat = lat + pixel_lat;
		double downlat = lat - pixel_lat;
		double leftlon = lon - pixel_lon;
		double rightlon = lon + pixel_lon;
		res.push_back(encode(lat, lon));
		res.push_back(encode(lat, leftlon));
		res.push_back(encode(lat, rightlon));
		res.push_back(encode(uplat, lon));
		res.push_back(encode(downlat, lon));
		res.push_back(encode(uplat, leftlon));
		res.push_back(encode(uplat, rightlon));
		res.push_back(encode(downlat, leftlon));
		res.push_back(encode(downlat, rightlon));
		return res;
	}

	double decode(vector<bool> bits, double min, double max){
		double mid = 0.0;
		for(int i = 0; i < bitlen; i++){
			//cout<<bits[i];
			//cout<<setprecision(10)<<"min: "<<min<<"max: "<<max<<endl;
			mid = (min + max)/2.0;
			if(bits[i] == 1){
				min = mid;
			}
			else{
				max = mid;
			}
		}
		//cout<<endl;
		return mid;
	}	

	vector<double> decode(string hashstr){
		vector<bool> latbits(bitlen);
		vector<bool> lonbits(bitlen);
		//cout<<hashstr<<endl;
		for(int i = 0; i < bitlen/5; i++){
			int tmp = digit_to_int[hashstr[2*i]];
			lonbits[i*5+0] = (tmp>>4) & 1;
			latbits[i*5+0] = (tmp>>3) & 1;
			lonbits[i*5+1] = (tmp>>2) & 1;
			latbits[i*5+1] = (tmp>>1) & 1;
			lonbits[i*5+2] = (tmp) & 1;
			int tmp2 = digit_to_int[hashstr[2*i+1]];
			latbits[i*5+2] = (tmp2>>4) & 1;
			lonbits[i*5+3] = (tmp2>>3) & 1;
			latbits[i*5+3] = (tmp2>>2) & 1;
			lonbits[i*5+4] = (tmp2>>1) & 1;
			latbits[i*5+4] = (tmp2) & 1;
		}
		vector<double> res;
		res.push_back(decode(latbits, -90, 90));
		res.push_back(decode(lonbits, -180, 180));
		return res;
	}
};

class GeoHashTire{
	public:
	struct Tire{
		Tire* child[32];
		bool is_end;
		string nodestr;
		Tire(){
			for(int i = 0; i < 32; i++){
				child[i] = NULL;
			}
			is_end = false;
			nodestr = "";
		}
	};

	Tire* root;
	GeoHashCoder geohash;

	GeoHashTire(){
		root = new Tire();
		geohash = GeoHashCoder();
	}

	void insert(string hashstr){
		//cout<<" "<<hashstr<<" ";
		Tire* cur = root;
		for(char c : hashstr){
			//cout<<c;
			int index = geohash.digit_to_int[c];
			if(cur->child[index] == NULL){
				cur->child[index] = new Tire();
			}
			cur = cur->child[index];
		}
		cur->nodestr = hashstr;
		cur->is_end = true;
	}

	Tire* search(string hashstr, int depth){
		Tire* cur = root;
		int depth_s = 0;
		for(int i = 0; i < depth; i++){
			char c = hashstr[i];
			int index = geohash.digit_to_int[c];
			if(cur->child[index] == NULL){
				return cur;
			}
			cur = cur->child[index];
		}
		return cur;
	}	

};