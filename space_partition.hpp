#ifndef SPACE_PARTITION_HPP
#define SPACE_PARTITION_HPP

//#include "vector.hpp"

//template<typedef T>
#define MAX_BALLS_PER_CELL 30

template <typename T = var_type>
class SpacePartition {
public:
    SpacePartition(std::vector<Ball<T>>& balls, T _x_max, T _y_max);
    void update();  // updates Ball* arrays 
    std::array<Ball<T>*, MAX_BALLS_PER_CELL * 9>& get_nearby_balls(Vector2<T> pos);  // 30 pointers times 9 neighbors
    int get_nearby_balls_count(Vector2<T> pos);  // number of valid balls in the array
    //std::pair<std::array<Ball*, MAX_BALLS_PER_CELL * 9>&, int> get_nearby_balls(Vector2 pos);

public:  // make private later
    //using BallptrArray = std::array<std::array<std::array<Ball*, 30>, 20>, 20>; 
    std::pair<int, int> get_index(Vector2<T> _pos);
    void get_neighbor_indexes(int _x,  int _y, std::array<std::pair<int, int>, 9>& neighbor_indexes);  // puts answers in neighbor_indexes
    void clear_grid_ballptrs();
    
    // negative indexes represent invalid neighbor indexes
    //std::array<std::pair<int, int>, 9> neighbor_indexes;  
    std::vector<Ball<T>>& balls; 
    T x_max;
    T y_max;
    std::array<std::array<std::array<Ball<T>*, MAX_BALLS_PER_CELL * 9>, 20>, 15> cached_ballptrs;
    std::array<std::array<std::array<std::pair<int, int>, 9>, 20>, 15> cached_neighbor_indexes;  

    std::array<std::array<std::size_t, 20>, 15> cached_num_ballptrs;

    int x_max_index;
    int y_max_index;
};


extern template class SpacePartition<int>;
extern template class SpacePartition<float>;
extern template class SpacePartition<double>;










#endif
