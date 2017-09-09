//
// Created by hvrigazov on 31.08.17.
//
#include "Map.h"
#include "Constants.h"
#include "spline.h"

Map::Map(const string &map_file_){

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    maps_x.push_back(x);
    maps_y.push_back(y);
    maps_s.push_back(s);
    maps_dx.push_back(d_x);
    maps_dy.push_back(d_y);
  }

	spline_x.set_points(maps_s, maps_x);
	spline_y.set_points(maps_s, maps_y);
	spline_dx.set_points(maps_s, maps_dx);
	spline_dy.set_points(maps_s, maps_dy);
}

Map::~Map(){}


void Map::padSplines() {
	for(int i = 0; i < 10; i++) {
		maps_s.push_back(maps_s[i] + MAX_S);
		maps_x.push_back(maps_x[i]);
		maps_y.push_back(maps_y[i]);
		maps_dx.push_back(maps_dx[i]);
		maps_dy.push_back(maps_dy[i]);
	}

  spline_x.set_points(maps_s, maps_x);
  spline_y.set_points(maps_s, maps_y);
  spline_dx.set_points(maps_s, maps_dx);
  spline_dy.set_points(maps_s, maps_dy);
}

vector<double> Map::getXY(double s, double d)
{
	if(s < 30){
		s += MAX_S;
	}

	const double x = spline_x(s) + spline_dx(s) * (d - 0.2);
	const double y = spline_y(s) + spline_dy(s) * (d - 0.2);

	return {x,y};
}


