#pragma once

#include <unistd.h>
#include <iostream>
#include <fstream> 
#include <iomanip>
#include <algorithm>
#include <cstdint>
#include <new>
#include <numeric>
#include <limits>
#include <vector>
#include <cmath>
#include <cassert>
#include <cstring>

#include <vamp/collision/math.hh>
#include <vamp/vector.hh>

namespace vamp::collision
{
    /**
     * Multi-level Voxel Table (MVT) - A hierarchical spatial data structure
     * for efficient collision detection using a three-level sparse table.
     * 
     * Assumption: all points to be inserted to MVT is in the given workspace bounds 
     */
    struct MVT
    {
        // ====================================================================
        // TYPE DEFINITIONS
        // ====================================================================
        
        using FVectorT = FloatVector<>;
        using IVectorT = IntVector<>;
        using VoxelIndex = uint32_t;
        
        static constexpr VoxelIndex INVALID_VOXEL_INDEX = std::numeric_limits<VoxelIndex>::max();
        static constexpr uint16_t MAX_GRID_WIDTH = std::numeric_limits<uint16_t>::max();
        
        // Three-level table types
        using TableOffset = uint32_t;
        using ZLevelTable = VoxelIndex*;   
        using YLevelTable = TableOffset*;  
        using XLevelTable = TableOffset*;

        // ====================================================================
        // VOXEL STRUCTURE
        // ====================================================================
        
        struct alignas(32) Voxel {
            float* x_coords = nullptr;
            float* y_coords = nullptr;
            float* z_coords = nullptr;
            size_t point_count = 0;
            size_t capacity = 0;
            
            Point bbox_min = {0.0f, 0.0f, 0.0f};
            Point bbox_max = {0.0f, 0.0f, 0.0f};
            
            Voxel() = default;

            void add_point(const Point& point, const float& point_radius) {
                if (point_count >= capacity) {
                    std::cout << "Try to add " << point_count + 1 << "th point to a voxel" << std::endl;
                    throw std::runtime_error("Voxel capacity exceeded");
                }

                x_coords[point_count] = point[0];
                y_coords[point_count] = point[1];
                z_coords[point_count] = point[2];
                
                update_bounding_box(point, point_radius);
                ++point_count;
            }

        private:
            void update_bounding_box(const Point& point, const float& point_radius) {
                float px_min = point[0] - point_radius;
                float py_min = point[1] - point_radius;
                float pz_min = point[2] - point_radius;
                float px_max = point[0] + point_radius;
                float py_max = point[1] + point_radius;
                float pz_max = point[2] + point_radius;

                if (point_count == 0) {
                    bbox_min = {px_min, py_min, pz_min};
                    bbox_max = {px_max, py_max, pz_max};
                } else {
                    bbox_min[0] = std::min(bbox_min[0], px_min);
                    bbox_min[1] = std::min(bbox_min[1], py_min);
                    bbox_min[2] = std::min(bbox_min[2], pz_min);
                    bbox_max[0] = std::max(bbox_max[0], px_max);
                    bbox_max[1] = std::max(bbox_max[1], py_max);
                    bbox_max[2] = std::max(bbox_max[2], pz_max);
                }
            }
        };

        // ====================================================================
        // MEMBER VARIABLES
        // ====================================================================
        
        // Query parameters
        float max_query_radius;
        float point_radius;
        
        // Spatial bounds
        Point workspace_aabb_min;
        Point workspace_aabb_max;
        Point global_aabb_min;
        Point global_aabb_max;
        
        // Grid configuration
        float inverse_scale_factor;
        uint16_t grid_width;
        
        // Memory pools
        std::unique_ptr<float[], decltype(&std::free)> point_coord_pool{nullptr, &std::free};
        size_t point_coord_pool_size = 0;
        size_t point_coord_pool_used = 0;
        
        static constexpr TableOffset NULL_OFFSET = std::numeric_limits<TableOffset>::max();
        std::unique_ptr<uint8_t[], decltype(&std::free)> hierarchy_pool{nullptr, &std::free};
        size_t hierarchy_pool_size_bytes = 0;
        size_t hierarchy_pool_used_bytes = 0;
        
        // Voxel storage and hierarchy entry
        std::vector<Voxel> voxel_storage;
        XLevelTable x_level_table = nullptr;
        
        // SIMD-optimized bounds
        FVectorT simd_global_min_x, simd_global_min_y, simd_global_min_z;
        FVectorT simd_global_max_x, simd_global_max_y, simd_global_max_z;
        FVectorT simd_workspace_min_x, simd_workspace_min_y, simd_workspace_min_z;

        // ====================================================================
        // CONSTRUCTOR & DESTRUCTOR
        // ====================================================================
        
        MVT(const std::vector<Point>& points,
            const float max_radius,
            const Point &workspace_aabb_min, 
            const Point &workspace_aabb_max, 
            const float point_radius) noexcept
            : max_query_radius{max_radius},
              workspace_aabb_min{workspace_aabb_min},
              workspace_aabb_max{workspace_aabb_max},
              point_radius{point_radius}
        {
            if (points.empty()) {
                initialize_empty_bounds();
                return;
            }
            
            configure_grid();
            initialize_hierarchy_pool();
            initialize_voxel_storage();
            build_spatial_grid_two_phase(points);
            compute_global_bounds();
            setup_simd_vectors();
        }

        MVT(const MVT& other)
            : max_query_radius(other.max_query_radius),
              point_radius(other.point_radius),
              workspace_aabb_min(other.workspace_aabb_min),
              workspace_aabb_max(other.workspace_aabb_max),
              global_aabb_min(other.global_aabb_min),
              global_aabb_max(other.global_aabb_max),
              inverse_scale_factor(other.inverse_scale_factor),
              grid_width(other.grid_width),
              point_coord_pool_size(other.point_coord_pool_size),
              point_coord_pool_used(other.point_coord_pool_used),
              hierarchy_pool_size_bytes(other.hierarchy_pool_size_bytes),
              hierarchy_pool_used_bytes(other.hierarchy_pool_used_bytes),
              voxel_storage(other.voxel_storage)
        {
            copy_memory_pools(other);
            update_pointers_after_copy(other);
            setup_simd_vectors();
        }

        ~MVT() = default;

        // ====================================================================
        // COLLISION DETECTION
        // ====================================================================
        
        // Scalar collision detection
        [[nodiscard]] auto collides(const Point& center, float radius) const noexcept -> bool
        {
            const float query_radius = radius + point_radius;
            const float query_radius_squared = query_radius * query_radius;

            // Early exit: Global AABB check
            if (center[0] + query_radius < global_aabb_min[0] ||
                center[0] - query_radius > global_aabb_max[0] ||
                center[1] + query_radius < global_aabb_min[1] ||
                center[1] - query_radius > global_aabb_max[1] ||
                center[2] + query_radius < global_aabb_min[2] ||
                center[2] - query_radius > global_aabb_max[2]) {
                return false;
            }

            // Compute grid space coordinates and query bounds
            const float grid_query_radius = std::min(1.0f, query_radius * inverse_scale_factor);
            const float grid_center_x_float = (center[0] - workspace_aabb_min[0]) * inverse_scale_factor;
            const float grid_center_y_float = (center[1] - workspace_aabb_min[1]) * inverse_scale_factor;
            const float grid_center_z_float = (center[2] - workspace_aabb_min[2]) * inverse_scale_factor;
            
            //Calculate voxel iteration bounds
            const uint16_t min_x = static_cast<uint16_t>(std::max(0.0f, (grid_center_x_float - grid_query_radius)));
            const uint16_t max_x = static_cast<uint16_t>(std::min(static_cast<float>(grid_width - 1), (grid_center_x_float + grid_query_radius)));
            const uint16_t min_y = static_cast<uint16_t>(std::max(0.0f, (grid_center_y_float - grid_query_radius)));
            const uint16_t max_y = static_cast<uint16_t>(std::min(static_cast<float>(grid_width - 1), (grid_center_y_float + grid_query_radius)));
            const uint16_t min_z = static_cast<uint16_t>(std::max(0.0f, (grid_center_z_float - grid_query_radius)));
            const uint16_t max_z = static_cast<uint16_t>(std::min(static_cast<float>(grid_width - 1), (grid_center_z_float + grid_query_radius)));

            const uint8_t* hierarchy_base = hierarchy_pool.get();
            // Traverse three-level spatial hierarchy
            for (uint16_t voxel_x = min_x; voxel_x <= max_x; ++voxel_x) {
                if (x_level_table[voxel_x] == NULL_OFFSET) continue;
                const TableOffset* y_table = reinterpret_cast<const TableOffset*>(hierarchy_base + x_level_table[voxel_x]);

                for (uint16_t voxel_y = min_y; voxel_y <= max_y; ++voxel_y) {
                    if (y_table[voxel_y] == NULL_OFFSET) continue;
                    const VoxelIndex* z_table = reinterpret_cast<const VoxelIndex*>(hierarchy_base + y_table[voxel_y]);
                    
                    for (uint16_t voxel_z = min_z; voxel_z <= max_z; ++voxel_z) {
                        VoxelIndex voxel_index = z_table[voxel_z];
                        if (voxel_index == INVALID_VOXEL_INDEX) continue;
                        
                        const Voxel& voxel = voxel_storage[voxel_index];
                        
                        // Voxel-level AABB culling
                        if (center[0] + query_radius < voxel.bbox_min[0] ||
                            center[0] - query_radius > voxel.bbox_max[0] ||
                            center[1] + query_radius < voxel.bbox_min[1] ||
                            center[1] - query_radius > voxel.bbox_max[1] ||
                            center[2] + query_radius < voxel.bbox_min[2] ||
                            center[2] - query_radius > voxel.bbox_max[2]) {
                            continue;
                        }
                        
                        // Point-level collision detection
                        const size_t num_points = voxel.point_count;
                        for (size_t i = 0; i < num_points; ++i) {
                            const float dx = center[0] - voxel.x_coords[i];
                            const float dy = center[1] - voxel.y_coords[i];
                            const float dz = center[2] - voxel.z_coords[i];
                            const float distance_squared = dx * dx + dy * dy + dz * dz;
                            
                            if (distance_squared <= query_radius_squared) {
                                return true;
                            }
                        }
                    }
                }
            }
            
            return false;
        }

        // SIMD vectorized collision detection for multiple spheres
        auto inline collides_simd(const std::array<FVectorT, 3> &centers, 
                                FVectorT radii) const noexcept -> bool
        {
            constexpr size_t SIMD_WIDTH = FVectorT::num_scalars;

            // Compute query radii for all spheres
            const FVectorT point_radius_vec = FVectorT::fill(point_radius);
            const FVectorT query_radii = radii + point_radius_vec;

            // SIMD global AABB check - cull entire lanes that are completely outside
            const auto outside_x_low = (centers[0]) < (simd_global_min_x - query_radii);
            const auto outside_x_high = (simd_global_max_x + query_radii) < (centers[0]);
            const auto outside_y_low = (centers[1]) < (simd_global_min_y - query_radii);
            const auto outside_y_high = (simd_global_max_y + query_radii) < (centers[1]);
            const auto outside_z_low = (centers[2]) < (simd_global_min_z - query_radii);
            const auto outside_z_high = (simd_global_max_z + query_radii) < (centers[2]);
            
            const auto outside_mask = outside_x_low | outside_x_high | 
                                    outside_y_low | outside_y_high | 
                                    outside_z_low | outside_z_high;
            
            if (outside_mask.all()) {
                return false;  // All spheres are outside global bounds
            }
            
            // Transform centers to grid space
            const FVectorT inv_scale = FVectorT::fill(inverse_scale_factor);
            const FVectorT grid_center_x = (centers[0] - simd_workspace_min_x) * inv_scale;
            const FVectorT grid_center_y = (centers[1] - simd_workspace_min_y) * inv_scale;
            const FVectorT grid_center_z = (centers[2] - simd_workspace_min_z) * inv_scale;
            const auto query_radii_squared = query_radii * query_radii;

            // Extract scalar arrays for per-sphere processing
            const auto centers_x_array = centers[0].to_array();
            const auto centers_y_array = centers[1].to_array();
            const auto centers_z_array = centers[2].to_array();
            const auto query_radii_array = query_radii.to_array();
            const auto query_radii_squared_array = query_radii_squared.to_array();
            const auto outside_array = outside_mask.to_array();
            const auto grid_x_array = grid_center_x.to_array();
            const auto grid_y_array = grid_center_y.to_array();
            const auto grid_z_array = grid_center_z.to_array();
            
            const uint8_t* hierarchy_base = hierarchy_pool.get();
            // Process each sphere individually
            for (size_t sphere_idx = 0; sphere_idx < SIMD_WIDTH; ++sphere_idx) {
                // Skip spheres that failed global AABB test
                if (outside_array[sphere_idx] != 0) {
                    continue;
                }
                
                // Extract sphere parameters
                const Point center = {centers_x_array[sphere_idx], centers_y_array[sphere_idx], centers_z_array[sphere_idx]};
                const float query_radius = query_radii_array[sphere_idx];
                const float query_radius_squared = query_radii_squared_array[sphere_idx];
                const float grid_query_radius = query_radius * inverse_scale_factor;
                // const float grid_query_radius = std::min(1.0f, query_radius * inverse_scale_factor);
                const float grid_center_x_float = grid_x_array[sphere_idx];
                const float grid_center_y_float = grid_y_array[sphere_idx];
                const float grid_center_z_float = grid_z_array[sphere_idx];
                
                // Calculate voxel iteration bounds for this sphere
                const float max_grid_idx_float = static_cast<float>(grid_width - 1);
                const uint16_t min_x = static_cast<uint16_t>(std::max(0.0f, (grid_center_x_float - grid_query_radius)));
                const uint16_t max_x = static_cast<uint16_t>(std::min(max_grid_idx_float, (grid_center_x_float + grid_query_radius)));
                const uint16_t min_y = static_cast<uint16_t>(std::max(0.0f, (grid_center_y_float - grid_query_radius)));
                const uint16_t max_y = static_cast<uint16_t>(std::min(max_grid_idx_float, (grid_center_y_float + grid_query_radius)));
                const uint16_t min_z = static_cast<uint16_t>(std::max(0.0f, (grid_center_z_float - grid_query_radius)));
                const uint16_t max_z = static_cast<uint16_t>(std::min(max_grid_idx_float, (grid_center_z_float + grid_query_radius)));

                // Traverse spatial hierarchy for this sphere
                for (uint16_t voxel_x = min_x; voxel_x <= max_x; ++voxel_x) {
                    // Level 1: Get Y-level table offset from X-level table
                    TableOffset y_offset = x_level_table[voxel_x];
                    if (y_offset == NULL_OFFSET) continue;
                    // Resolve the actual pointer by adding the offset to the hierarchy base
                    const TableOffset* y_level_table = reinterpret_cast<const TableOffset*>(hierarchy_base + y_offset);
                    
                    for (uint16_t voxel_y = min_y; voxel_y <= max_y; ++voxel_y) {
                        // Level 2: Get Z-level (voxel index) table offset from Y-level table
                        TableOffset z_offset = y_level_table[voxel_y];
                        if (z_offset == NULL_OFFSET) continue;
                        
                        // Resolve the actual pointer for the Z-level table
                        const VoxelIndex* z_level_table = reinterpret_cast<const VoxelIndex*>(hierarchy_base + z_offset);
                        
                        for (uint16_t voxel_z = min_z; voxel_z <= max_z; ++voxel_z) {
                            VoxelIndex voxel_index = z_level_table[voxel_z];
                            if (voxel_index == INVALID_VOXEL_INDEX) continue;
                            
                            const Voxel& voxel = voxel_storage[voxel_index];
                            
                            // Voxel-level AABB culling
                            if (center[0] + query_radius < voxel.bbox_min[0] ||
                                center[0] - query_radius > voxel.bbox_max[0] ||
                                center[1] + query_radius < voxel.bbox_min[1] ||
                                center[1] - query_radius > voxel.bbox_max[1] ||
                                center[2] + query_radius < voxel.bbox_min[2] ||
                                center[2] - query_radius > voxel.bbox_max[2]) {
                                continue;
                            }
                            
                            // SIMD point-level collision detection
                            const size_t num_points = voxel.point_count;
                            const auto* x_coords = voxel.x_coords;
                            const auto* y_coords = voxel.y_coords;
                            const auto* z_coords = voxel.z_coords;

                            const FVectorT sphere_x = FVectorT::fill(center[0]);
                            const FVectorT sphere_y = FVectorT::fill(center[1]);
                            const FVectorT sphere_z = FVectorT::fill(center[2]);
                            const FVectorT sphere_radius_sq = FVectorT::fill(query_radius_squared);

                            // Process points in SIMD chunks
                            for (size_t point_idx = 0; point_idx < num_points; point_idx += SIMD_WIDTH) {
                                const FVectorT point_x(x_coords + point_idx);
                                const FVectorT point_y(y_coords + point_idx);
                                const FVectorT point_z(z_coords + point_idx);
                                
                                const FVectorT dx = sphere_x - point_x;
                                const FVectorT dy = sphere_y - point_y;
                                const FVectorT dz = sphere_z - point_z;
                                const FVectorT dist_sq = dx * dx + dy * dy + dz * dz;
                                
                                const auto collision_mask = dist_sq <= sphere_radius_sq;
                                if (collision_mask.any()) {
                                    return true;
                                }
                            }
                        }
                    }
                }
            }
            
            return false;
        }

    private:
        // ====================================================================
        // INITIALIZATION HELPERS
        // ====================================================================
        
        void initialize_empty_bounds() {
            constexpr float max_val = std::numeric_limits<float>::max();
            constexpr float min_val = std::numeric_limits<float>::lowest();
            
            global_aabb_min = Point{max_val, max_val, max_val};
            global_aabb_max = Point{min_val, min_val, min_val};
        }

        void configure_grid() {
            const float workspace_width = workspace_aabb_max[0] - workspace_aabb_min[0];
            float voxel_size = (max_query_radius) + point_radius;
            
            grid_width = static_cast<uint16_t>(std::min(
                static_cast<uint32_t>(std::floor(workspace_width / voxel_size)), // Empirically < 50 for manipulator robots
                static_cast<uint32_t>(MAX_GRID_WIDTH) // upper bound
            ));

            inverse_scale_factor = grid_width / workspace_width;
        }

        /**
         * Initializes a unified memory pool for the X, Y, and Z level tables.
         * Uses 32-bit offsets to save space and simplify memory relocation.
         */
        void initialize_hierarchy_pool() {
            // 1. Estimate Pointer Table requirements (X and Y levels)
            const size_t estimated_xy_tables = 1 + grid_width;
            const size_t pointer_pool_bytes = estimated_xy_tables * grid_width * sizeof(TableOffset);

            // 2. Estimate Voxel Index Table requirements (Z level)
            const size_t estimated_z_tables = static_cast<size_t>(grid_width) * grid_width * 0.8;
            const size_t z_table_pool_bytes = estimated_z_tables * grid_width * sizeof(VoxelIndex);

            // 3. Calculate Total Size
            hierarchy_pool_size_bytes = pointer_pool_bytes + z_table_pool_bytes;
            hierarchy_pool_used_bytes = 0;

            // 4. Allocate Aligned Memory
            void* raw_ptr = nullptr;
            if (posix_memalign(&raw_ptr, 64, hierarchy_pool_size_bytes) != 0) {
                throw std::runtime_error("Failed to allocate hierarchy pool");
            }
            
            hierarchy_pool.reset(static_cast<uint8_t*>(raw_ptr));
        }

        void initialize_voxel_storage() {
            const size_t estimated_voxel_count = 
                static_cast<size_t>(grid_width) * grid_width * grid_width * 0.1;
            voxel_storage.reserve(estimated_voxel_count);
        }

        void setup_simd_vectors() {
            simd_global_min_x = FVectorT::fill(global_aabb_min[0]);
            simd_global_min_y = FVectorT::fill(global_aabb_min[1]);
            simd_global_min_z = FVectorT::fill(global_aabb_min[2]);
            simd_global_max_x = FVectorT::fill(global_aabb_max[0]);
            simd_global_max_y = FVectorT::fill(global_aabb_max[1]);
            simd_global_max_z = FVectorT::fill(global_aabb_max[2]);
            
            simd_workspace_min_x = FVectorT::fill(workspace_aabb_min[0]);
            simd_workspace_min_y = FVectorT::fill(workspace_aabb_min[1]);
            simd_workspace_min_z = FVectorT::fill(workspace_aabb_min[2]);
        }

        // ====================================================================
        // SPATIAL GRID CONSTRUCTION
        // ====================================================================
        
        void build_spatial_grid_two_phase(const std::vector<Point>& points) {
            // --- PHASE 1: INIT HIERARCHY & COUNT VOXEL POINTS ---
            
            // Initialize root of three-level table hierarchy
            TableOffset x_table_offset;
            x_level_table = allocate_table<XLevelTable>(x_table_offset);

            // Keep track of which voxel each point belongs to so we don't re-calculate in Phase 2
            std::vector<VoxelIndex> point_to_voxel(points.size());
            
            // Insert each point into the corresponding voxel
            for (size_t i = 0; i < points.size(); ++i) {
                const auto& point = points[i];
                // Transform point coordinates to grid space
                const float voxel_x_float = (point[0] - workspace_aabb_min[0]) * inverse_scale_factor;
                const float voxel_y_float = (point[1] - workspace_aabb_min[1]) * inverse_scale_factor;
                const float voxel_z_float = (point[2] - workspace_aabb_min[2]) * inverse_scale_factor;

                // Clamp to valid grid indices just in case
                const uint16_t voxel_x = static_cast<uint16_t>(std::clamp(voxel_x_float, 0.0f, static_cast<float>(grid_width - 1)));
                const uint16_t voxel_y = static_cast<uint16_t>(std::clamp(voxel_y_float, 0.0f, static_cast<float>(grid_width - 1)));
                const uint16_t voxel_z = static_cast<uint16_t>(std::clamp(voxel_z_float, 0.0f, static_cast<float>(grid_width - 1)));
                // Intervals are half-open [lower, upper) except for the last voxel

                // Level 1: Get Y-level
                TableOffset y_offset = x_level_table[voxel_x];
                if (y_offset == NULL_OFFSET) {
                    allocate_table<YLevelTable>(x_level_table[voxel_x]);
                }
                YLevelTable y_level_table = reinterpret_cast<YLevelTable>(hierarchy_pool.get() + x_level_table[voxel_x]);

                // Level 2: Get Z-level
                TableOffset z_offset = y_level_table[voxel_y];
                if (z_offset == NULL_OFFSET) {
                    allocate_table<ZLevelTable>(y_level_table[voxel_y]);
                }
                ZLevelTable z_level_table = reinterpret_cast<ZLevelTable>(hierarchy_pool.get() + y_level_table[voxel_y]);
                
                // Level 3: Get or create voxel
                VoxelIndex voxel_index = z_level_table[voxel_z];
                if (voxel_index == INVALID_VOXEL_INDEX) {
                    // Create new voxel and assign index
                    voxel_index = static_cast<VoxelIndex>(voxel_storage.size());
                    voxel_storage.emplace_back();
                    z_level_table[voxel_z] = voxel_index;
                }
                voxel_storage[voxel_index].point_count++; // Increment count only
                point_to_voxel[i] = voxel_index;          // Cache index
            }
                

            // --- PHASE 2: EXACT ALLOCATION & FILLING ---
            size_t total_required_floats = 0;
            constexpr size_t SIMD_WIDTH = FVectorT::num_scalars;

            for (auto& voxel : voxel_storage) {
                // Round each voxel's count up to SIMD width for alignment
                voxel.capacity = (voxel.point_count + SIMD_WIDTH - 1) & ~(SIMD_WIDTH - 1);
                total_required_floats += voxel.capacity * 3; // X, Y, Z
            }
            allocate_exact_point_pool(total_required_floats);

            // Assign pointers within the pool
            for (auto& voxel : voxel_storage) {
                voxel.x_coords = allocate_coords(voxel.capacity);
                voxel.y_coords = allocate_coords(voxel.capacity);
                voxel.z_coords = allocate_coords(voxel.capacity);
                // Reset count to 0 so add_point() can fill it correctly
                voxel.point_count = 0; 
            }

            for (size_t i = 0; i < points.size(); ++i) {
                VoxelIndex voxel_index = point_to_voxel[i];
                voxel_storage[voxel_index].add_point(points[i], point_radius);
            }
        }

        void compute_global_bounds() {
            initialize_empty_bounds();
            
            for (const auto& voxel : voxel_storage) {
                if (voxel.point_count > 0) {
                    global_aabb_min[0] = std::min(global_aabb_min[0], voxel.bbox_min[0]);
                    global_aabb_min[1] = std::min(global_aabb_min[1], voxel.bbox_min[1]);
                    global_aabb_min[2] = std::min(global_aabb_min[2], voxel.bbox_min[2]);
                    global_aabb_max[0] = std::max(global_aabb_max[0], voxel.bbox_max[0]);
                    global_aabb_max[1] = std::max(global_aabb_max[1], voxel.bbox_max[1]);
                    global_aabb_max[2] = std::max(global_aabb_max[2], voxel.bbox_max[2]);
                }
            }
        }

        // ====================================================================
        // MEMORY POOL ALLOCATION
        // ====================================================================
        
        void allocate_exact_point_pool(size_t total_floats) {
            point_coord_pool_size = total_floats;
            size_t total_bytes = point_coord_pool_size * sizeof(float);
            
            void* raw_ptr = nullptr;
            if (posix_memalign(&raw_ptr, 64, total_bytes) != 0) {
                throw std::runtime_error("Failed to allocate exact pool");
            }
            
            point_coord_pool.reset(static_cast<float*>(raw_ptr));
            point_coord_pool_used = 0;
            
            // Fill with Infinity for SIMD safety padding
            std::fill(point_coord_pool.get(), point_coord_pool.get() + point_coord_pool_size, 
                      std::numeric_limits<float>::infinity());
        }

        template<typename T>
        T allocate_table(TableOffset& out_offset) {
            const size_t element_size = std::is_same_v<T, ZLevelTable> ? sizeof(VoxelIndex) : sizeof(TableOffset);
            const size_t size_bytes = grid_width * element_size;
            
            if (hierarchy_pool_used_bytes + size_bytes > hierarchy_pool_size_bytes) {
                std::cout << "try to allocate " << hierarchy_pool_used_bytes + size_bytes << " bytes. Capacity: " << hierarchy_pool_size_bytes << "bytes" << std::endl;
                throw std::runtime_error("hierarchy pool exhausted");
            }
            
            out_offset = static_cast<TableOffset>(hierarchy_pool_used_bytes);
            T result = reinterpret_cast<T>(hierarchy_pool.get() + hierarchy_pool_used_bytes);
            
            if constexpr (std::is_same_v<T, ZLevelTable>) {
                std::fill(result, result + grid_width, INVALID_VOXEL_INDEX);
            } else {
                std::fill(result, result + grid_width, NULL_OFFSET);
            }
            
            hierarchy_pool_used_bytes += size_bytes;
            return result;
        }

        float* allocate_coords(size_t count) {
            if (point_coord_pool_used + count > point_coord_pool_size) {
                throw std::runtime_error("Point coordinate pool exhausted");
            }
            
            float* result = point_coord_pool.get() + point_coord_pool_used;
            point_coord_pool_used += count;
            return result;
        }

        // ====================================================================
        // COPY CONSTRUCTOR HELPERS
        // ====================================================================
        
        void copy_memory_pools(const MVT& other) {
            copy_point_coord_pool(other);
            copy_hierarchy_pool(other);
        }

        void copy_point_coord_pool(const MVT& other) {
            if (!other.point_coord_pool || other.point_coord_pool_size == 0) return;
            
            void* raw_ptr = nullptr;

            if (posix_memalign(&raw_ptr, 64, point_coord_pool_size * sizeof(float)) != 0) {
                throw std::runtime_error("Failed to allocate aligned memory pool");
            }
            
            point_coord_pool.reset(static_cast<float*>(raw_ptr));
            std::memcpy(point_coord_pool.get(), other.point_coord_pool.get(), 
                       point_coord_pool_used * sizeof(float));
        }

        void copy_hierarchy_pool(const MVT& other) {
            if (!other.hierarchy_pool || other.hierarchy_pool_size_bytes == 0) return;
            
            void* raw_ptr = nullptr;
            // Allocate the same amount of memory as the original pool
            if (posix_memalign(&raw_ptr, 64, other.hierarchy_pool_size_bytes) != 0) {
                throw std::runtime_error("Failed to allocate aligned hierarchy pool during copy");
            }
            
            hierarchy_pool.reset(static_cast<uint8_t*>(raw_ptr));
            
            // Copy the actual data (the X, Y, and Z table offsets)
            std::memcpy(hierarchy_pool.get(), other.hierarchy_pool.get(), 
                       other.hierarchy_pool_used_bytes);
        }

        void update_pointers_after_copy(const MVT& other) {
            relocate_table_hierarchy(other);
            relocate_voxel_coordinates(other);
        }

        void relocate_table_hierarchy(const MVT& other) {
            ptrdiff_t x_offset_in_pool = reinterpret_cast<uint8_t*>(other.x_level_table) - 
                                         other.hierarchy_pool.get();
            
            x_level_table = reinterpret_cast<XLevelTable>(hierarchy_pool.get() + x_offset_in_pool);
            // All internal offsets remain valid.
        }

        void relocate_voxel_coordinates(const MVT& other) {
            for (size_t idx = 0; idx < voxel_storage.size(); ++idx) {
                auto& voxel = voxel_storage[idx];
                const auto& other_voxel = other.voxel_storage[idx];
                
                if (other_voxel.x_coords != nullptr) {
                    voxel.x_coords = point_coord_pool.get() + (other_voxel.x_coords - other.point_coord_pool.get());
                    voxel.y_coords = point_coord_pool.get() + (other_voxel.y_coords - other.point_coord_pool.get());
                    voxel.z_coords = point_coord_pool.get() + (other_voxel.z_coords - other.point_coord_pool.get());
                }
            }
        }

        // ====================================================================
        // UTILITY FUNCTIONS
        // ====================================================================
        
        unsigned int next_power_of_two(unsigned int n) {
            if (n == 0) return 1;
            n--;
            n |= n >> 1;
            n |= n >> 2;
            n |= n >> 4;
            n |= n >> 8;
            n |= n >> 16;
            n++;
            return n;
        }
    };
}  // namespace vamp::collision