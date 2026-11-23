#pragma once
#include <string>
#include <vector>
#include <cmath>

struct Slot{std::string color; double x, y, z;};

// PYRAMID POS
inline const std::vector<Slot>& slots(){
    static const std::vector<Slot> s = {
        {"orange", 0.120, -0.360, 0.03}, 
        {"orange", 0.135, -0.285, 0.03},  
        {"orange", 0.150, -0.200, 0.03},  
        {"black",  0.127, -0.322, 0.09},  
        {"black",  0.142, -0.242, 0.09},   
        {"beige",  0.135, -0.285, 0.15},  
    };
    return s;
}