#pragma once

#include <vector>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief Multi-slice 2D map logic for 3D exploration
 * 
 * Generates multiple 2D occupancy grids at different height slices
 * based on the drone's frame of reference and height layers.
 */
class MultiSliceLogic {
public:
    struct HeightSlice {
        double min_height;
        double max_height;
        nav_msgs::msg::OccupancyGrid map_2d;
        std::vector<geometry_msgs::msg::Point> frontier_points_3d;
    };

    /**
     * @brief Generate height slices from 3D map data
     * 
     * @param occupancy_3d_data 3D occupancy grid data
     * @param width Width of the map
     * @param height Height of the map  
     * @param depth Depth (z-dimension) of the map
     * @param resolution Resolution of the grid (meters per cell)
     * @param origin_x X origin of the map
     * @param origin_y Y origin of the map
     * @param origin_z Z origin of the map
     * @param drone_z Current drone height
     * @param height_margin Margin above and below drone height to consider (meters)
     * @param slice_thickness Thickness of each height slice (meters)
     * @return Vector of height slices with 2D maps
     */
    static std::vector<HeightSlice> generateHeightSlices(
        const std::vector<int8_t>& occupancy_3d_data,
        uint32_t width,
        uint32_t height,
        uint32_t depth,
        float resolution,
        float origin_x,
        float origin_y,
        float origin_z,
        float drone_z,
        float height_margin = 3.0f,
        float slice_thickness = 0.5f);

    /**
     * @brief Convert 2D frontier points to 3D points at specified height
     * 
     * @param frontier_2d_points 2D frontier points in map coordinates
     * @param slice_height Z-height for the 3D frontier points
     * @param resolution Grid resolution
     * @param origin_x Map origin X
     * @param origin_y Map origin Y
     * @param origin_z Map origin Z
     * @return Vector of 3D frontier points
     */
    static std::vector<geometry_msgs::msg::Point> convertTo3DFrontiers(
        const std::vector<geometry_msgs::msg::Point>& frontier_2d_points,
        double slice_height,
        float resolution,
        float origin_x,
        float origin_y,
        float origin_z);

    /**
     * @brief Extract occupied cells in a height range from 3D grid
     * 
     * @param occupancy_3d_data Raw 3D occupancy data
     * @param width Width of map
     * @param height Height of map
     * @param depth Depth of map
     * @param min_z Minimum Z coordinate (world frame)
     * @param max_z Maximum Z coordinate (world frame)
     * @param origin_z Origin Z coordinate
     * @param resolution Grid resolution
     * @return 2D occupancy grid for the height range
     */
    static nav_msgs::msg::OccupancyGrid extract2DSlice(
        const std::vector<int8_t>& occupancy_3d_data,
        uint32_t width,
        uint32_t height,
        uint32_t depth,
        float min_z,
        float max_z,
        float origin_z,
        float resolution);

private:
    /**
     * @brief Get index in 3D occupancy grid
     */
    static inline int getIndex3D(int x, int y, int z, uint32_t width, uint32_t height) {
        return x + y * width + z * width * height;
    }

    /**
     * @brief Get index in 2D occupancy grid
     */
    static inline int getIndex2D(int x, int y, uint32_t width) {
        return x + y * width;
    }
};
