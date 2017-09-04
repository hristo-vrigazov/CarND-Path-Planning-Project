//
// Created by hvrigazov on 31.08.17.
//
#ifndef WORLD_H
#define WORLD_H

#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <math.h>

#include "Constants.h"
#include "Utils.h"
#include "spline.h"

using namespace std;

class Map {

   std::vector<double> maps_x;
   std::vector<double> maps_y;
   std::vector<double> maps_s;
   std::vector<double> maps_dx;
   std::vector<double> maps_dy;


   tk::spline spline_x;
   tk::spline spline_y;
   tk::spline spline_dx;
   tk::spline spline_dy;
   
 public:
  Map(const std::string &map_file_);
  ~Map();

  std::vector<double> getXY(double s, double d);

  void padSplines();


};
#endif /* WORLD_H */
