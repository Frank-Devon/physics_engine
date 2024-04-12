#include <vector>
#include <array>
#include <iostream>
#include "vector.hpp"
#include "physics.hpp"
#include "space_partition.hpp"

////template<typedef T>
//class SpacePartition {
//public:
//    SpacePartition(std::vector<Ball>& balls, var_type _x_max, var_type _y_max);
//    void update();  // updates Ball* arrays 
//
//private:
//    using BallptrArray = std::array<std::array<std::array<Ball*, 30>, 20>, 20>; 
//    BallptrArray grid_ballptrs;
//    std::vector<Ball>& balls; 
//    var_type x_max, y_max;
//    std::array<Ball*, 30>& get_nearby_balls(Vector2 pos);
//};

//template<typename T>
//bool in_range(T x, T low, T high) {
//    return (x >= low && x < high); 
//}

bool array_bound_check(int index, int size) {
    return (index >= 0 && index < size);
}

//template<typedef T>
SpacePartition::SpacePartition(std::vector<Ball>& _balls, var_type _x_max, var_type _y_max) : 
        neighbor_indexes{std::pair<unsigned int, unsigned int>(-1, -1)}, balls(_balls), x_max(_x_max), y_max(_y_max), grid_ballptrs{} 
{
    // make sure to fill multidimensional array with NULLs 
    clear_grid_ballptrs();
    y_max_index = grid_ballptrs.size();
    x_max_index = grid_ballptrs[0].size();

    std::cout << "sp x_max_index, y_max_index = " << x_max_index << ", " <<  y_max_index << std::endl;
}

std::array<Ball*, 30 * 9>&
SpacePartition::get_nearby_balls(Vector2 _pos) {
    //nearby_balls.fill(nullptr);
    //nearby_balls[0] = nullptr;
    using std::pair;
    pair<int, int> index = get_index(_pos);
    get_neighbor_indexes(index.first, index.second);
    int i = 0;
    for (auto index : neighbor_indexes) {
        if (index.first == -1 || index.second == -1) break;
        for (Ball* ball : grid_ballptrs[index.second][index.first]) {
            if (ball == nullptr) break;  // assume remaining elements are nullptr as well
            nearby_balls[i] = ball;
            i++;
        }
    }
    nearby_balls[i] = nullptr;  // this marks the end of the valid pointers
    return nearby_balls;
}

void
SpacePartition::update() {
    std::pair<int, int> index_grid(0, 0);
    clear_grid_ballptrs();
    for (Ball& ball : balls) {
        index_grid = get_index(ball.pos); 
        // now find the next nullptr element in the array, and write ball address to it
        for (size_t i = 0; i < grid_ballptrs[0][0].size(); ++i) { 
            if (grid_ballptrs[index_grid.second][index_grid.first][i] == nullptr) {
                grid_ballptrs[index_grid.second][index_grid.first][i] = &ball;
                break;
            }
        }
    }
}


std::pair<int, int> 
SpacePartition::get_index(Vector2 _pos) {
    std::pair<int, int> result;
    result.first = (_pos.x / x_max) * x_max_index; //grid_ballptrs.size();
    result.second = (_pos.y / y_max) * y_max_index; //grid_ballptrs[0].size();
    // bounds checking
    if (result.first < 0) result.first = 0;
    else if (result.first >= x_max_index) result.first = x_max_index - 1;
    if (result.second < 0) result.second = 0;
    else if (result.second >= y_max_index) result.second = y_max_index - 1;
    return result;
}

// puts answers in neighbor_indexes
void
SpacePartition::get_neighbor_indexes(int _x, int _y) {  
    int k = 0;
    int x_index = 0;
    int y_index = 0;
    //printf("g_neighbors_indexes\n");
    for (int y = -1; y <= 1; ++y) {
        for (int x = -1; x <= 1; ++x) {
            x_index = _x + (int)x;
            y_index = _y + (int)y;
            if (array_bound_check(x_index, x_max_index) 
                    && array_bound_check(y_index, y_max_index)) {

                neighbor_indexes[k] = std::pair<int, int>(x_index, y_index);
                k++;
            }
            //else printf("out of bounds\n");
        }
    }
    // fill in invalid indexes with an obviously invalid index
    for (size_t i = k; i < 9; ++i) {
        neighbor_indexes[i] = std::pair<int, int>(-1, -1);    
    }
}

void
SpacePartition::clear_grid_ballptrs() {
    for (auto& row : grid_ballptrs) {
        for (auto& ballptr_array : row) {
            ballptr_array.fill(nullptr);
        }
    }
}










