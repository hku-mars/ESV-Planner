#include "map_manager/GridMap3D.h"

void GridMap3D::clearGridMap() {
    for (int i = 0; i < totalsize; i++) {
        grid_map[i] = 0;
        grid_map_buffer_neg[i] = 0;
        grid_map_buffer_all[i] = 0;
        grid_map_flags[i] = 0;
    }
}

void GridMap3D::releaseMemory() {
    delete[] grid_map;
    delete[] grid_map_buffer_neg;
    delete[] grid_map_buffer_all;
    delete[] grid_map_flags;
    grid_map = nullptr;
    grid_map_buffer_neg = nullptr;
    grid_map_buffer_all = nullptr;
    grid_map_flags = nullptr;
}
// The origin of the grid map (i.e., the (0, 0, 0) index in the grid) corresponds to the boundary_xyzmin point in the 3D space. left-bottom corner
void GridMap3D::createGridMap(const Vector3d &boundary_xyzmin, const Vector3d &boundary_xyzmax) {
    cout << "createGridMap initial done" << endl;
    this->boundary_xyzmin = boundary_xyzmin;
    this->boundary_xyzmax = boundary_xyzmax;
    X_size = ceil((boundary_xyzmax(0) - boundary_xyzmin(0)) / grid_resolution);
    Y_size = ceil((boundary_xyzmax(1) - boundary_xyzmin(1)) / grid_resolution);
    Z_size = ceil((boundary_xyzmax(2) - boundary_xyzmin(2)) / grid_resolution);

    // cout << "in createGridMap, X_size: " << X_size << " Y_size: " << Y_size << " Z_size: " << Z_size << endl;

    totalsize = X_size * Y_size * Z_size;
    grid_map = new double[totalsize]();
    grid_map_buffer_neg = new double[totalsize]();
    grid_map_buffer_all = new double[totalsize]();
    grid_map_flags = new bool[totalsize]();

    // clearGridMap();
}

bool GridMap3D::isInMap(const Vector3d &pos_w) const {
    if (pos_w(0) < boundary_xyzmin(0)) {
        cout << "pos_x < boundary_xmin: " << pos_w << endl;
        return false;
    }
    if (pos_w(1) < boundary_xyzmin(1)) {
        cout << "pos_y < boundary_ymin: " << pos_w << endl;
        return false;
    }
    if (pos_w(2) < boundary_xyzmin(2)) {
        cout << "pos_z < boundary_zmin: " << pos_w << endl;
        return false;
    }

    if (pos_w(0) > boundary_xyzmax(0)) {
        cout << "pos_x > boundary_xmax: " << pos_w << endl;
        return false;
    }
    if (pos_w(1) > boundary_xyzmax(1)) {
        cout << "pos_y > boundary_ymax: " << pos_w << endl;
        return false;
    }
    if (pos_w(2) > boundary_xyzmax(2)) {
        cout << "pos_z > boundary_zmax: " << pos_w << endl;
        return false;
    }
    return true;
}

bool GridMap3D::isIndexValid3d(const Vector3i &index) const {
    if (index(0) < 0) {
        return false;
    }
    if (index(0) >= X_size) {
        return false;
    }

    if (index(1) < 0) {
        return false;
    }
    if (index(1) >= Y_size) {
        return false;
    }

    if (index(2) < 0) {
        return false;
    }
    if (index(2) >= Z_size) {
        return false;
    }

    return true;
}

bool GridMap3D::isIndexValid3d(const int ix, const int iy, const int iz) const {
    if (ix < 0) {
        return false;
    }
    if (ix >= X_size) {
        return false;
    }

    if (iy < 0) {
        return false;
    }
    if (iy >= Y_size) {
        return false;
    }

    if (iz < 0) {
        return false;
    }
    if (iz >= Z_size) {
        return false;
    }

    return true;
}

Vector3i GridMap3D::getGridIndex3d(const Vector3d &pos_w) {
    if (!isInMap(pos_w)) {
        cout << "pos_w is not in map: " << pos_w << endl;
        return Vector3i(0, 0, 0);
    }

    double dx = pos_w(0) - boundary_xyzmin(0);
    double dy = pos_w(1) - boundary_xyzmin(1);
    double dz = pos_w(2) - boundary_xyzmin(2);
    int ix = floor(dx / grid_resolution);
    int iy = floor(dy / grid_resolution);
    int iz = floor(dz / grid_resolution);

    if (ix < 0) {
        ix = 0;
    }
    if (ix >= X_size) {
        ix = X_size - 1;
    }
    if (iy < 0) {
        iy = 0;  // Corrected from ix to iy
    }
    if (iy >= Y_size) {
        iy = Y_size - 1;
    }
    if (iz < 0) {
        iz = 0;  // Corrected from ix to iz
    }
    if (iz >= Z_size) {
        iz = Z_size - 1;
    }

    return Vector3i(ix, iy, iz);
}

Vector2i GridMap3D::getGridIndex2d(const Vector2d &pos_w) {
    Vector3i gridIndex3D = getGridIndex3d(Vector3d(pos_w(0), pos_w(1), 0));
    return Vector2i(gridIndex3D(0), gridIndex3D(1));
}

// Overload to handle 2D indices passed as individual integers
Vector2d GridMap3D::getGridCubeCenter2d(int ix, int iy) {
    Vector3d center3d = getGridCubeCenter3d(Vector3i(ix, iy, 0));
    return center3d.head<2>();  // Return the first two components as a Vector2d
}

// Overload to handle 2D indices passed as a Vector2i
Vector2d GridMap3D::getGridCubeCenter2d(const Vector2i &index) {
    Vector3d center3d = getGridCubeCenter3d(Vector3i(index(0), index(1), 0));
    return center3d.head<2>();  // Return the first two components as a Vector2d
}

bool GridMap3D::isIndexOccupied2d(const Vector2i &index) {
    return isIndexOccupied3d(Vector3i(index(0), index(1), 0));
}

bool GridMap3D::isIndexOccupied2d(int ix, int iy) {
    return isIndexOccupied3d(Vector3i(ix, iy, 0));
}

Vector3d GridMap3D::getGridCubeCenter3d(int ix, int iy, int iz) {
    return getGridCubeCenter3d(Vector3i(ix, iy, iz));
}

Vector3d GridMap3D::getGridCubeCenter3d(const Vector3i &index) {
    if (!isIndexValid3d(index)) {
        return Vector3d(0, 0, 0);
    }
    double dx = (index(0) + 0.5) * grid_resolution;
    double dy = (index(1) + 0.5) * grid_resolution;
    // double dz = (index(2) + 0.5) * grid_resolution;
    double dz = 0;
    Vector3d dpos_w = Vector3d(dx, dy, dz);
    return dpos_w + boundary_xyzmin;
}

bool GridMap3D::isIndexOccupied3d(const Vector3i &index) {
    if (!isIndexValid3d(index)) {
        return true;
    }
    if (grid_map[toAddr(index(0), index(1), index(2))] == 0) {
        return false;
    } else {
        return true;
    }
}
bool GridMap3D::isIndexOccupiedFlate2d(const Vector2i &index, const int flate_pix) {
    Vector3i new_ind = Vector3i(index(0), index(1), 0);
    return isIndexOccupiedFlate3d(new_ind, flate_pix);
}

bool GridMap3D::isIndexOccupiedFlate3d(const Vector3i &index, const int flate_pix) {
    Eigen::Vector3i new_ind = index;
    for (int dx = -flate_pix; dx <= flate_pix; dx++) {
        for (int dy = -flate_pix; dy <= flate_pix; dy++) {
            for (int dz = -flate_pix; dz <= flate_pix; dz++) {
                new_ind(0) = index(0) + dx;
                new_ind(1) = index(1) + dy;
                new_ind(2) = index(2);
                if (!isIndexValid3d(new_ind)) {
                    continue;
                } else if (grid_map[toAddr(new_ind(0), new_ind(1), new_ind(2))] == 1) {
                    return true;
                }
            }
        }
    }
    return false;
}

bool GridMap3D::isIndexOccupied3d(int ix, int iy, int iz) {
    bool invalid = false;
    if (ix < 0) {
        return true;
    }
    if (ix >= X_size) {
        return true;
    }
    if (iy < 0) {
        return true;
    }
    if (iy >= Y_size) {
        return true;
    }
    if (iz < 0) {
        return true;
    }
    if (iz >= Z_size) {
        return true;
    }
    if (invalid) {
        return true;
    }
    if (grid_map[toAddr(ix, iy, iz)] == 0) {
        return false;
    } else {
        return true;
    }
}

bool GridMap3D::isIndexOccupied3d_test(int ix, int iy, int iz) {
    cout << "current check ix: " << ix << " iy: " << iy << " iz: " << iz << endl;
    bool invalid = false;
    if (ix < 0) {
        cout << "ix < 0, return true" << endl;
        return true;
    }
    if (ix >= X_size) {
        cout << "ix >= X_size, return true" << endl;
        return true;
    }
    if (iy < 0) {
        cout << "iy < 0, return true" << endl;
        return true;
    }
    if (iy >= Y_size) {
        cout << "iy >= Y_size, return true" << endl;
        return true;
    }
    if (iz < 0) {
        cout << "iz < 0, return true" << endl;
        return true;
    }
    if (iz >= Z_size) {
        cout << "iz >= Z_size, return true" << endl;
        return true;
    }
    if (invalid) {
        cout << "invalid, return true" << endl;
        return true;
    }
    if (grid_map[toAddr(ix, iy, iz)] == 0) {
        cout << "grid_map[toAddr(ix, iy, iz)] == 0, return false" << endl;
        return false;
    } else {
        cout << "grid_map[toAddr(ix, iy, iz)] == 1, return true" << endl;
        return true;
    }
}

bool GridMap3D::isCoordOccupied3d(const Vector3d &pos_w) {
    if (!isInMap(pos_w)) {
        // if (debug_output)
        // {
        //     ROS_WARN("[isCoordOccupied3d()] this point is out of map!");
        // }
        return true;
    }
    return isIndexOccupied3d(getGridIndex3d(pos_w));
}

bool GridMap3D::isCoordOccupied2d(const Vector2d &pos_w) {
    Vector3d pos_3d = Vector3d(pos_w(0), pos_w(1), 0);
    if (!isInMap(pos_3d)) {
        // if (debug_output)
        // {
        //     ROS_WARN("[isCoordOccupied3d()] this point is out of map!");
        // }
        return true;
    }
    return isIndexOccupied2d(getGridIndex2d(pos_w));
}

bool GridMap3D::isGridBoundOfObs(const Vector3i &index) {
    if (!isIndexOccupied3d(index)) {
        cout << "[isGridBoundOfObs] this function got a free index!" << endl;
        return false;
    }

    int idx, idy, idz;
    Vector3i index_scan;
    for (idx = 0; idx < X_size; idx++) {
        for (idy = 0; idy < Y_size; idy++) {
            for (idz = 0; idz < Z_size; idz++) {
                index_scan = index + Vector3i(idx, idy, idz);
                if (index_scan == index) {
                    continue;
                }
                if (!isIndexValid3d(index_scan)) {
                    continue;
                }
                if (!isIndexOccupied3d(index_scan)) {
                    return true;
                }
            }
        }
    }
    return false;
}

void GridMap3D::clearGridESDF() {
    for (int i = 0; i < totalsize; i++) {
        grid_esdf_buffer1[i] = 0;
        grid_esdf_buffer2[i] = 0;
    }
}

int GridMap3D::getVoxelNum(int dim) {
    if (dim == 0) {
        return X_size;
    }
    if (dim == 1) {
        return Y_size;
    }
    if (dim == 2) {
        return Z_size;
    }
    return 0;
}

void debugP(double num) {
    cout << num << endl;
}

void GridMap3D::generateESDF3d() {
    Vector3i min_esdf(0, 0, 0);
    Vector3i max_esdf(X_size - 1, Y_size - 1, Z_size - 1);

    grid_esdf = new double[totalsize]();
    grid_esdf_buffer1 = new double[totalsize]();
    grid_esdf_buffer2 = new double[totalsize]();
    // clearGridESDF();

    /* ========== compute positive DT ========== */

    for (int x = min_esdf(0); x <= max_esdf(0); x++) {
        for (int y = min_esdf(1); y <= max_esdf(1); y++) {
            fillESDF(
                [&](int z) { return isIndexOccupied3d(x, y, z) == 1 ? 0 : std::numeric_limits<double>::max(); }, [&](int z, double val) { grid_esdf_buffer1[toAddr(x, y, z)] = val; }, min_esdf(2),
                max_esdf(2), 2);
        }
    }

    for (int x = min_esdf(0); x <= max_esdf(0); x++) {
        for (int z = min_esdf(2); z <= max_esdf(2); z++) {
            fillESDF([&](int y) { return grid_esdf_buffer1[toAddr(x, y, z)]; }, [&](int y, double val) { grid_esdf_buffer2[toAddr(x, y, z)] = val; }, min_esdf(1), max_esdf(1), 1);
        }
    }

    for (int y = min_esdf(1); y <= max_esdf(1); y++) {
        for (int z = min_esdf(2); z <= max_esdf(2); z++) {
            fillESDF([&](int x) { return grid_esdf_buffer2[toAddr(x, y, z)]; }, [&](int x, double val) { grid_esdf[toAddr(x, y, z)] = grid_resolution * std::sqrt(val); }, min_esdf(0), max_esdf(0), 0);
        }
    }

    /* ========== compute negative distance ========== */
    for (int x = min_esdf(0); x <= max_esdf(0); ++x)
        for (int y = min_esdf(1); y <= max_esdf(1); ++y)
            for (int z = min_esdf(2); z <= max_esdf(2); ++z) {
                if (grid_map[toAddr(x, y, z)] == 0) {
                    grid_map_buffer_neg[toAddr(x, y, z)] = 1;
                } else if (grid_map[toAddr(x, y, z)] == 1) {
                    grid_map_buffer_neg[toAddr(x, y, z)] = 0;
                } else {
                    ROS_ERROR("what?");
                }
            }

    ros::Time t1, t2;

    for (int x = min_esdf(0); x <= max_esdf(0); x++) {
        for (int y = min_esdf(1); y <= max_esdf(1); y++) {
            fillESDF(
                [&](int z) { return grid_map_buffer_neg[toAddr(x, y, z)] == 1 ? 0 : std::numeric_limits<double>::max(); }, [&](int z, double val) { grid_esdf_buffer1[toAddr(x, y, z)] = val; },
                min_esdf(2), max_esdf(2), 2);
        }
    }

    for (int x = min_esdf(0); x <= max_esdf(0); x++) {
        for (int z = min_esdf(2); z <= max_esdf(2); z++) {
            fillESDF([&](int y) { return grid_esdf_buffer1[toAddr(x, y, z)]; }, [&](int y, double val) { grid_esdf_buffer2[toAddr(x, y, z)] = val; }, min_esdf(1), max_esdf(1), 1);
        }
    }

    for (int y = min_esdf(1); y <= max_esdf(1); y++) {
        for (int z = min_esdf(2); z <= max_esdf(2); z++) {
            fillESDF(
                [&](int x) { return grid_esdf_buffer2[toAddr(x, y, z)]; }, [&](int x, double val) { grid_map_buffer_neg[toAddr(x, y, z)] = grid_resolution * std::sqrt(val); }, min_esdf(0),
                max_esdf(0), 0);
        }
    }

    /* ========== combine pos and neg DT ========== */
    for (int x = min_esdf(0); x <= max_esdf(0); ++x)
        for (int y = min_esdf(1); y <= max_esdf(1); ++y)
            for (int z = min_esdf(2); z <= max_esdf(2); ++z) {
                grid_map_buffer_all[toAddr(x, y, z)] = grid_map[toAddr(x, y, z)];

                if (grid_map_buffer_neg[toAddr(x, y, z)] > 0.0)
                    grid_map_buffer_all[toAddr(x, y, z)] += (-grid_map_buffer_neg[toAddr(x, y, z)] + grid_resolution);
            }
}

template <typename F_get_val, typename F_set_val>
void GridMap3D::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim) {
    int v[getVoxelNum(dim)];
    double z[getVoxelNum(dim) + 1];

    int k = start;
    v[start] = start;
    z[start] = -std::numeric_limits<double>::max();
    z[start + 1] = std::numeric_limits<double>::max();

    for (int q = start + 1; q <= end; q++) {
        k++;
        double s;

        do {
            k--;
            s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
        } while (s <= z[k]);

        k++;

        v[k] = q;
        z[k] = s;
        z[k + 1] = std::numeric_limits<double>::max();
    }

    k = start;

    for (int q = start; q <= end; q++) {
        while (z[k + 1] < q)
            k++;
        double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
        f_set_val(q, val);
    }
}

double GridMap3D::getGridSDFValue(const Vector3i &index) {
    if (!isIndexValid3d(index)) {
        return 0;
    }
    return grid_esdf[toAddr(index(0), index(1), index(2))];
}

double GridMap3D::getGridSDFValue(int ix, int iy, int iz) {
    if (ix < 0) {
        ix = 0;
    }
    if (ix >= X_size) {
        ix = X_size - 1;
    }
    if (iy < 0) {
        iy = 0;
    }
    if (iy >= Y_size) {
        iy = Y_size - 1;
    }
    if (iz < 0) {
        iz = 0;
    }
    if (iz >= Z_size) {
        iz = Z_size - 1;
    }
    return grid_esdf[toAddr(ix, iy, iz)];
}
