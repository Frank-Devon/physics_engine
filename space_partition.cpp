#include <vector>
#include <array>
#include <iostream>
#include "vector.hpp"
#include "physics.hpp"
#include "space_partition.hpp"

bool array_bound_check(int index, int size) {
    return (index >= 0 && index < size);
}

template <typename T>
SpacePartition<T>::SpacePartition(std::vector<Ball<T>>& _balls, T _x_max, T _y_max) : 
        balls(_balls), x_max(_x_max), y_max(_y_max),
        cached_ballptrs{}, cached_neighbor_indexes{}
{
    clear_grid_ballptrs();
    y_max_index = cached_ballptrs.size();
    x_max_index = cached_ballptrs[0].size();

    //cache indexes of neighbors of each cell
    for (int y = 0; y < y_max_index; y++) {
        for (int x = 0; x < x_max_index; x++) {
            get_neighbor_indexes(x, y, cached_neighbor_indexes[y][x]); 
        }
    }

    std::cout << "sp x_max_index, y_max_index = " << x_max_index << ", " <<  y_max_index << std::endl;
}

template <typename T>
std::array<Ball<T>*, MAX_BALLS_PER_CELL * 9>&
SpacePartition<T>::get_nearby_balls(Vector2<T> _pos) {
    std::pair<int, int> index = get_index(_pos);
    return cached_ballptrs[index.second][index.first];
}

template <typename T>
int SpacePartition<T>::get_nearby_balls_count(Vector2<T> _pos) {  // number of valid balls in the array
    std::pair<int, int> index = get_index(_pos);
    return cached_num_ballptrs[index.second][index.first];
}

template <typename T>
void SpacePartition<T>::update() {
    std::pair<int, int> index_ball(0, 0);
    clear_grid_ballptrs();
    for (Ball<T>& ball : balls) {
        index_ball = get_index(ball.pos); //index of ball 
        //get_neighbor_indexes(index_ball.first, index_ball.second);    
        for (auto index_neighbor : cached_neighbor_indexes[index_ball.second][index_ball.first]) {
            if (index_neighbor.first == -1 || index_neighbor.second == -1) {
                continue; // probably can break;
            }
            std::size_t& count_cached = cached_num_ballptrs[index_neighbor.second][index_neighbor.first];
            if (count_cached + 1 
                    >= cached_ballptrs[index_neighbor.second][index_neighbor.first].size()) {
                std::cout << "ERROR, ball location cannot be cached" << std::endl;
                continue;
            }
            cached_ballptrs[index_neighbor.second][index_neighbor.first][count_cached] = &ball;
            count_cached++;
            cached_ballptrs[index_neighbor.second][index_neighbor.first][count_cached] = nullptr;
        }
    }
}

template <typename T>
std::pair<int, int> 
SpacePartition<T>::get_index(Vector2<T> _pos) {
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
template <typename T>
void SpacePartition<T>::get_neighbor_indexes(int _x, int _y, std::array<std::pair<int, int>, 9>& neighbor_indexes ) {  
    int k = 0;
    int x_index = 0;
    int y_index = 0;
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

template <typename T>
void SpacePartition<T>::clear_grid_ballptrs() {
    for(auto& array_nums : cached_num_ballptrs) {
        array_nums.fill(0);
    }
}



template class SpacePartition<int>;
template class SpacePartition<float>;
template class SpacePartition<double>;







