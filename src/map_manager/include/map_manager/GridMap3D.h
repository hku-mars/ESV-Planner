#ifndef __GRID_MAP_
#define __GRID_MAP_

#include <ros/ros.h>
#include <ros/package.h>
#include <Eigen/Eigen>
#include <string>
using namespace std;
using namespace Eigen;
class GridMap3D {
public:
    void clearGridMap();
    void releaseMemory();
    void createGridMap(const Vector3d& boundary_xyzmin, const Vector3d& boundary_xyzmax);
    bool isInMap(const Vector3d& pos_w) const;
    ////////////////////// test purpose ////////////////////////////
    bool isIndexOccupied3d_test(int ix, int iy, int iz);
    ////////////////////// test purpose ////////////////////////////

    ////////////////////// 2d occupancy map ////////////////////////
    bool isIndexOccupied2d(const Vector2i& index);
    bool isIndexOccupied2d(int ix, int iy);

    bool isCoordOccupied2d(const Vector2d& pos_w);
    bool isCoordOccupied2d(int ix, int iy);
    Vector2i getGridIndex2d(const Vector2d& pos_w);
    Vector2d getGridCubeCenter2d(const Vector2i& index);
    Vector2d getGridCubeCenter2d(int ix, int iy);
    bool isIndexOccupiedFlate2d(const Vector2i& index, const int flate_pix);
    ////////////////////// 2d occupancy map ////////////////////////

    ////////////////////// 3d occupancy map ////////////////////////
    bool isIndexValid3d(const Vector3i& index) const;
    bool isIndexValid3d(const int ix, const int iy, const int iz) const;

    bool isCoordOccupied3d(const Vector3d& pos_w);
    bool isIndexOccupied3d(int ix, int iy, int iz);
    bool isIndexOccupied3d(const Vector3i& index);

    bool isIndexOccupiedFlate3d(const Vector3i& index, const int flate_pix);

    Vector3i getGridIndex3d(const Vector3d& pos_w);

    Vector3d getGridCubeCenter3d(const Vector3i& index);
    Vector3d getGridCubeCenter3d(int ix, int iy, int iz);

    ////////////////////// 3d occupancy map ////////////////////////

    int getVoxelNum(int dim);
    bool isGridBoundOfObs(const Vector3i& index);  // return true while the grid is on the boundary of a obstacle.

    void generateESDF3d();

    // to generate ESDF map from occupancy map
    void clearGridESDF();
    template <typename F_get_val, typename F_set_val>
    void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);

    double getGridSDFValue(const Vector3i& index);
    double getGridSDFValue(int ix, int iy, int iz);

    inline double getSDFValue(const Vector3d& pos_w) {
        /* use trilinear interpolation */
        Vector3d pos_m = pos_w - 0.5 * grid_resolution * Vector3d::Ones();
        Vector3i idx = getGridIndex3d(pos_m);
        Vector3d idx_pos, diff;
        idx_pos = getGridCubeCenter3d(idx);
        diff = (pos_w - idx_pos) * (1.0 / grid_resolution);

        double values[2][2][2];
        Vector3i current_idx;
        for (int x = 0; x < 2; x++) {
            for (int y = 0; y < 2; y++) {
                for (int z = 0; z < 2; z++) {
                    current_idx = idx + Vector3i(x, y, z);
                    if (!isInMap(getGridCubeCenter3d(current_idx))) {
                        values[x][y][z] = 0;
                    }
                    values[x][y][z] = getGridSDFValue(current_idx);
                }
            }
        }

        double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
        double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
        double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
        double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];
        double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
        double v1 = (1 - diff[1]) * v01 + diff[1] * v11;
        double dist = (1 - diff[2]) * v0 + diff[2] * v1;

        return dist;
    }

    inline double getSDFValueWithGrad(const Vector3d& pos_w, Vector3d& grad) {
        /* use trilinear interpolation */
        Vector3d pos_m = pos_w - 0.5 * grid_resolution * Vector3d::Ones();
        Vector3i idx = getGridIndex3d(pos_m);
        Vector3d idx_pos, diff;
        idx_pos = getGridCubeCenter3d(idx);
        diff = (pos_w - idx_pos) * (1.0 / grid_resolution);

        double values[2][2][2];
        Vector3i current_idx;
        for (int x = 0; x < 2; x++) {
            for (int y = 0; y < 2; y++) {
                for (int z = 0; z < 2; z++) {
                    current_idx = idx + Vector3i(x, y, z);
                    if (!isInMap(getGridCubeCenter3d(current_idx))) {
                        values[x][y][z] = 0;
                    }
                    values[x][y][z] = getGridSDFValue(current_idx);
                }
            }
        }
        double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
        double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
        double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
        double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];
        double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
        double v1 = (1 - diff[1]) * v01 + diff[1] * v11;
        double dist = (1 - diff[2]) * v0 + diff[2] * v1;

        grad[2] = (v1 - v0) * (1.0 / grid_resolution);
        grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * (1.0 / grid_resolution);
        grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
        grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
        grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
        grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
        grad[0] *= (1.0 / grid_resolution);
        return dist;
    }
    // The toAddr function computes a unique 1D index for a given 3D coordinate in a 3D grid. This allows you to store and access 3D data using a simple 1D array.
    inline int toAddr(const int idx, const int idy, const int idz) { return idx * Y_size * Z_size + idy * Z_size + idz; }
    // inline int toAddr(const int idx, const int idy, const int idz = 0) { return (Z_size > 1) ? (idx * Y_size * Z_size + idy * Z_size + idz) : (idx * Y_size + idy); }

public:
    // max index size
    int X_size;
    int Y_size;
    int Z_size;
    int totalsize;
    // map size box
    Vector3d boundary_xyzmin;
    Vector3d boundary_xyzmax;

    // map resolution
    double grid_resolution;

    // debug switch
    bool debug_output;

    // occupancy map and its ESDF map

    double* grid_map{nullptr};
    double* grid_esdf{nullptr};

    // for esdf map generation, temp buffer
    double* grid_map_buffer_neg{nullptr};
    double* grid_map_buffer_all{nullptr};
    double* grid_esdf_buffer1{nullptr};
    double* grid_esdf_buffer2{nullptr};

    // a flag for custom usage.
    bool* grid_map_flags{nullptr};

public:
    typedef shared_ptr<GridMap3D> Ptr;
};

#endif