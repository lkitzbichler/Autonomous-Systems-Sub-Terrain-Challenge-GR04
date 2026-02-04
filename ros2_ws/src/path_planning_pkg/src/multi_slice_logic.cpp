#include "multi_slice_logic.hpp"
#include <algorithm>
#include <cmath>

std::vector<MultiSliceLogic::HeightSlice> MultiSliceLogic::generateHeightSlices(
    const std::vector<int8_t>& occupancy_3d_data,
    uint32_t width,
    uint32_t height,
    uint32_t depth,
    float resolution,
    float origin_x,
    float origin_y,
    float origin_z,
    float drone_z,
    float height_margin,
    float slice_thickness)
{
    std::vector<HeightSlice> slices;

    // Calculate height range around drone
    float min_height = drone_z - height_margin;
    float max_height = drone_z + height_margin;

    // Generate slices
    for (float z = min_height; z < max_height; z += slice_thickness) {
        HeightSlice slice;
        slice.min_height = z;
        slice.max_height = z + slice_thickness;

        // Extract 2D map from 3D data for this height range
        slice.map_2d = extract2DSlice(
            occupancy_3d_data, width, height, depth,
            z, z + slice_thickness,
            origin_z, resolution);

        slices.push_back(slice);
    }

    return slices;
}

std::vector<geometry_msgs::msg::Point> MultiSliceLogic::convertTo3DFrontiers(
    const std::vector<geometry_msgs::msg::Point>& frontier_2d_points,
    double slice_height,
    float resolution,
    float origin_x,
    float origin_y,
    float origin_z)
{
    std::vector<geometry_msgs::msg::Point> frontiers_3d;

    for (const auto& point_2d : frontier_2d_points) {
        geometry_msgs::msg::Point point_3d;

        // Convert from map coordinates to world coordinates
        point_3d.x = origin_x + (point_2d.x * resolution);
        point_3d.y = origin_y + (point_2d.y * resolution);
        point_3d.z = slice_height;  // Use the slice height

        frontiers_3d.push_back(point_3d);
    }

    return frontiers_3d;
}

nav_msgs::msg::OccupancyGrid MultiSliceLogic::extract2DSlice(
    const std::vector<int8_t>& occupancy_3d_data,
    uint32_t width,
    uint32_t height,
    uint32_t depth,
    float min_z,
    float max_z,
    float origin_z,
    float resolution)
{
    nav_msgs::msg::OccupancyGrid grid_2d;
    grid_2d.info.width = width;
    grid_2d.info.height = height;
    grid_2d.info.resolution = resolution;
    grid_2d.info.origin.position.x = 0.0f;  // Relative to map origin
    grid_2d.info.origin.position.y = 0.0f;
    grid_2d.header.frame_id = "map";

    // Initialize 2D grid with all unknown cells
    grid_2d.data.assign(width * height, -1);

    // Calculate depth indices for the height range
    int min_depth_idx = std::max(0.0f, (min_z - origin_z) / resolution);
    int max_depth_idx = std::min((float)depth, (max_z - origin_z) / resolution);

    // Aggregate 3D cells into 2D slice
    for (uint32_t x = 0; x < width; ++x) {
        for (uint32_t y = 0; y < height; ++y) {
            int8_t max_occupancy = -1;  // Start with unknown

            // Check all depth layers in the height range
            for (int z = min_depth_idx; z < max_depth_idx; ++z) {
                if (z >= 0 && z < (int)depth) {
                    int idx_3d = getIndex3D(x, y, z, width, height);
                    if (idx_3d >= 0 && idx_3d < (int)occupancy_3d_data.size()) {
                        int8_t occupancy = occupancy_3d_data[idx_3d];

                        // Keep maximum occupancy for this cell
                        if (occupancy > max_occupancy) {
                            max_occupancy = occupancy;
                        }
                    }
                }
            }

            // Set the 2D grid cell
            int idx_2d = getIndex2D(x, y, width);
            grid_2d.data[idx_2d] = max_occupancy;
        }
    }

    return grid_2d;
}
