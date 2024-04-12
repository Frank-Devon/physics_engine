#ifndef SPACE_PARTITION_HPP
#define SPACE_PARTITION_HPP

//#include "vector.hpp"

//template<typedef T>
class SpacePartition {
public:
    SpacePartition(std::vector<Ball>& balls, var_type _x_max, var_type _y_max);
    void update();  // updates Ball* arrays 
    std::array<Ball*, 30 * 9>& get_nearby_balls(Vector2 pos);  // 30 pointers times 9 neighbors

public:  // make private later
    //using BallptrArray = std::array<std::array<std::array<Ball*, 30>, 20>, 20>; 
    std::pair<int, int> get_index(Vector2 _pos);
    void get_neighbor_indexes(int _x,  int _y);  // puts answers in neighbor_indexes
    void clear_grid_ballptrs();
    
    // negative indexes represent invalid neighbor indexes
    std::array<std::pair<int, int>, 9> neighbor_indexes;  
    std::vector<Ball>& balls; 
    var_type x_max;
    var_type y_max;
    //std::array<std::array<std::array<Ball*, 30>, X_MAX_INDEX>, Y_MAX_INDEX> grid_ballptrs;
    std::array<std::array<std::array<Ball*, 30>, 20>, 15> grid_ballptrs;
    int x_max_index;
    int y_max_index;
    std::array<Ball*, 30 * 9> nearby_balls;
};













#endif
