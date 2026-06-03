#pragma once

#include <array>
#include <vector>
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <memory>

#include <vamp/collision/math.hh>
#include <vamp/vector.hh>

namespace vamp::collision
{
    struct Voxel {
        Point stored_point;
        Point voxel_center;
        float stored_point_dist_sq = 0.0f;
        bool occupied = false;
        
        void set_voxel_center(uint16_t vx, uint16_t vy, uint16_t vz, float voxel_size, Point workspace_min) {
            voxel_center[0] = workspace_min[0] + (vx + 0.5f) * voxel_size;
            voxel_center[1] = workspace_min[1] + (vy + 0.5f) * voxel_size;
            voxel_center[2] = workspace_min[2] + (vz + 0.5f) * voxel_size;
        }
        
        bool try_insert(const Point& point) {
            const float dx = point[0] - voxel_center[0];
            const float dy = point[1] - voxel_center[1];
            const float dz = point[2] - voxel_center[2];
            const float new_dist_sq = dx*dx + dy*dy + dz*dz;

            if (!occupied || new_dist_sq < stored_point_dist_sq) {
                stored_point = point;
                stored_point_dist_sq = new_dist_sq;
                occupied = true;
                return true;
            }
            
            return false;
        }
    };

    struct CenterSelectiveVoxelFilter {
        using TableOffset = uint32_t;
        static constexpr TableOffset NULL_OFFSET = std::numeric_limits<TableOffset>::max();
        
        // ====================================================================
        // MEMBER VARIABLES
        // ====================================================================

        std::unique_ptr<uint8_t[], decltype(&std::free)> hierarchy_pool{nullptr, &std::free};
        size_t hierarchy_pool_size = 0;
        size_t hierarchy_pool_used = 0;
        TableOffset x_level_table_offset = NULL_OFFSET;

        std::unique_ptr<Voxel[], decltype(&std::free)> voxel_pool{nullptr, &std::free};
        size_t voxel_pool_size = 0;
        size_t allocated_voxel_count = 0;
        
        Point workspace_aabb_min;
        Point origin_point;
        float inverse_scale_factor;
        float voxel_size;
        float max_range_sq;
        uint16_t grid_width;
        
        // ====================================================================
        // CONSTRUCTOR
        // ====================================================================
        
        CenterSelectiveVoxelFilter(float voxel_sz, float max_range, Point origin, 
                                  Point workspace_min, Point workspace_max)
            : workspace_aabb_min(workspace_min),
              voxel_size(voxel_sz), max_range_sq(max_range * max_range), 
              origin_point(origin)
        {
            const float workspace_width = std::max({
                workspace_max[0] - workspace_min[0],
                workspace_max[1] - workspace_min[1], 
                workspace_max[2] - workspace_min[2]
            });
            
            grid_width = static_cast<uint16_t>(std::ceil(workspace_width / voxel_size));
            inverse_scale_factor = 1.0 / voxel_size;
            
            // Estimated hierarchy pool size
            size_t table_size = grid_width * sizeof(TableOffset);
            size_t estimated_tables = 1 + grid_width + static_cast<size_t>(grid_width * grid_width * 0.3);
            hierarchy_pool_size = estimated_tables * table_size;
            // Allocate hierarchy pool
            void* h_ptr = nullptr;
            int result = posix_memalign(&h_ptr, 64, hierarchy_pool_size);
            if (result != 0 || h_ptr == nullptr) {
                throw std::bad_alloc();
            }
            hierarchy_pool.reset(static_cast<uint8_t*>(h_ptr));
            std::memset(hierarchy_pool.get(), 0xFF, hierarchy_pool_size); // Init to NULL_OFFSET
            
            // Estimated voxel pool size
            voxel_pool_size = static_cast<size_t>(std::pow(grid_width, 3.0f) * 0.05f);
            
            // Allocate voxel pool
            void* v_ptr = nullptr;
            result = posix_memalign(&v_ptr, 64, sizeof(Voxel) * voxel_pool_size); // sizeof(Voxel): 32 bytes
            if (result != 0 || v_ptr == nullptr) {
                throw std::bad_alloc();
            }
            voxel_pool.reset(static_cast<Voxel*>(v_ptr));

            allocate_table(x_level_table_offset);
        }

        // ====================================================================
        // CORE LOGIC
        // ====================================================================
        
        TableOffset* get_table_ptr(TableOffset offset) const {
            return reinterpret_cast<TableOffset*>(hierarchy_pool.get() + offset);
        }

        bool allocate_table(TableOffset& out_offset) {
            size_t size = grid_width * sizeof(TableOffset);
            if (hierarchy_pool_used + size > hierarchy_pool_size) {
                std::cerr << "Table Allocation Failed!" << std::endl;
                return false;
            }
            out_offset = static_cast<TableOffset>(hierarchy_pool_used);
            std::memset(hierarchy_pool.get() + out_offset, 0xFF, size); // Set to NULL_OFFSET
            hierarchy_pool_used += size;
            return true;
        }

        bool try_insert_point(const Point& point) {
            // Range and bounds culling
            const float dx = point[0] - origin_point[0];
            const float dy = point[1] - origin_point[1];
            const float dz = point[2] - origin_point[2];
            if (dx*dx + dy*dy + dz*dz >= max_range_sq) {
                // std::cout << "Culled by distance" << std::endl;
                return false;
            }
            int ivx = static_cast<int>((point[0] - workspace_aabb_min[0]) * inverse_scale_factor);
            int ivy = static_cast<int>((point[1] - workspace_aabb_min[1]) * inverse_scale_factor);
            int ivz = static_cast<int>((point[2] - workspace_aabb_min[2]) * inverse_scale_factor);
            if (static_cast<uint32_t>(ivx) >= static_cast<uint32_t>(grid_width) ||
                static_cast<uint32_t>(ivy) >= static_cast<uint32_t>(grid_width) ||
                static_cast<uint32_t>(ivz) >= static_cast<uint32_t>(grid_width)) {
                // std::cout << "Outside of grid\n" << std::endl;
                return false;
            }
            const uint16_t vx = static_cast<uint16_t>(ivx);
            const uint16_t vy = static_cast<uint16_t>(ivy);
            const uint16_t vz = static_cast<uint16_t>(ivz);
            // Traverse three-level tables
            TableOffset* x_table = get_table_ptr(x_level_table_offset);
        
            if (x_table[vx] == NULL_OFFSET) {
                if (!allocate_table(x_table[vx])) return false;
            }
            
            TableOffset* y_table = get_table_ptr(x_table[vx]);
            if (y_table[vy] == NULL_OFFSET) {
                if (!allocate_table(y_table[vy])) return false;
            }
    
            TableOffset* z_table = get_table_ptr(y_table[vy]);
            
            TableOffset& voxel_idx = z_table[vz];
            if (voxel_idx == NULL_OFFSET) {
                if (allocated_voxel_count >= voxel_pool_size) {
                    std::cerr << "Voxel Pool Full!" << std::endl;
                    return false;
                }
                
                voxel_idx = static_cast<TableOffset>(allocated_voxel_count++);
                Voxel* v = &voxel_pool[voxel_idx];
                new(v) Voxel(); // Placement new
                v->set_voxel_center(vx, vy, vz, voxel_size, workspace_aabb_min);
            }

            return voxel_pool[voxel_idx].try_insert(point);
        }

        std::vector<Point> extract_points() const {
            std::vector<Point> result;
            result.reserve(allocated_voxel_count);
        
            for (size_t i = 0; i < allocated_voxel_count; ++i) {
                if (voxel_pool[i].occupied) {
                    result.push_back(voxel_pool[i].stored_point);
                }
            }
            
            return result;
        }
    };
    
    // CenterVox: Voxel-based point cloud downsampling filter
    //
    // Subdivides workspace into 3D grid of cubic voxels, keeping at most one point per voxel.
    // The representative point is chosen as the one closest to the voxel's geometric center.
    //
    // Parameters:
    // - voxel_size: Edge length of cubic voxels (controls downsampling resolution)
    // - max_range: Maximum distance from origin for point retention  
    // - origin: Origin point for range filtering
    // - workspace_min/max: Workspace bounding box
    //
    // Features:
    // - Adaptive memory allocation based on workspace volume
    // - Append-only voxel pool for optimal performance
    // - Three-level hierarchical lookup (X->Y->Z) for sparse storage

    template <typename PointCloud>
    auto filter_pointcloud_centervox(
        const PointCloud &pc,
        float voxel_size,
        float max_range,
        Point origin,
        Point workspace_min,
        Point workspace_max) -> std::vector<Point>
    {
        if (pc.shape(0) == 0) {
            return std::vector<Point>();
        }

        CenterSelectiveVoxelFilter filter(voxel_size, max_range, origin, workspace_min, workspace_max);
        
        for (uint32_t i = 0; i < pc.shape(0); ++i) {
            Point point{pc(i, 0), pc(i, 1), pc(i, 2)};
            filter.try_insert_point(point);
        }

        // filter.test_point_roundtrip(pc.shape(0));
        // filter.visualize_summary();

        return filter.extract_points();
    }

    template <>
    inline auto filter_pointcloud_centervox(
        const std::vector<Point> &pc,
        float voxel_size,
        float max_range,
        Point origin,
        Point workspace_min,
        Point workspace_max) -> std::vector<Point>
    {
        struct PointcloudWrapper {
            const std::vector<Point> &pc;
            
            auto shape(std::size_t dim) const noexcept -> std::size_t {
                return (dim == 0) ? pc.size() : (dim == 1) ? 3 : 0;
            }

            auto operator()(std::size_t i, std::size_t j) const noexcept -> typename Point::value_type {
                return pc[i][j];
            }
        };

        return filter_pointcloud_centervox(
            PointcloudWrapper{pc}, voxel_size, max_range,
            std::move(origin), std::move(workspace_min), std::move(workspace_max));
    }

}  // namespace vamp::collision