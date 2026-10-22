#ifndef SHAPE_HPP
#define SHAPE_HPP
#define IGL_STATIC_LIBRARY 1
#include <iostream>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <vector>
#include <utils/config.hpp>
#include <utils/Visualization.hpp>
#include <utils/quickhull.hpp>
#include <utils/geo_utils.hpp>
#include <unordered_set>
#include <igl/read_triangle_mesh.h>
#include <igl/write_triangle_mesh.h>
#include <nav_msgs/OccupancyGrid.h>

#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>
#include <igl/signed_distance.h>
#include <igl/sparse_voxel_grid.h>
#include <igl/marching_cubes.h>
#include <igl/fast_winding_number.h>
#include <igl/winding_number.h>
#include <igl/writeDMAT.h>
#include <igl/random_points_on_mesh.h>
#include <igl/AABB.h>
#include <igl/doublearea.h>
#include <igl/per_face_normals.h>
#include <igl/readDMAT.h>
#define PI 3.14159265358979323846
using namespace vis;
using namespace std;
#define DEFINE_USEFUL_FUNCTION()                                                              \
    inline Eigen::Vector3d getonlyGrad1(const Eigen::RowVector3d &pos_rel) {                  \
        double dx = 0.000001;                                                                 \
        Eigen::RowVector3d temp = pos_rel;                                                    \
        temp(0) -= dx;                                                                        \
        double sdfold = getonlySDF(temp);                                                     \
        temp(0) += 2 * dx;                                                                    \
        double gradx = getonlySDF(temp) - sdfold;                                             \
                                                                                              \
        temp = pos_rel;                                                                       \
                                                                                              \
        temp(1) -= dx;                                                                        \
        sdfold = getonlySDF(temp);                                                            \
        temp(1) += 2 * dx;                                                                    \
        double grady = getonlySDF(temp) - sdfold;                                             \
                                                                                              \
        Eigen::Vector3d grad = Eigen::Vector3d(gradx, grady, 0) / (2 * dx);                   \
        return grad;                                                                          \
    }                                                                                         \
    inline Eigen::Vector3d helperfunc(const Eigen::RowVector3d &pos_rel, double &sdf) {       \
        double dx = 0.000001;                                                                 \
        Eigen::RowVector3d temp = pos_rel;                                                    \
        sdf = getonlySDF(pos_rel);                                                            \
        temp(0) -= dx;                                                                        \
        double sdfold = getonlySDF(temp);                                                     \
        temp(0) += 2 * dx;                                                                    \
        double gradx = getonlySDF(temp) - sdfold;                                             \
                                                                                              \
        temp = pos_rel;                                                                       \
                                                                                              \
        temp(1) -= dx;                                                                        \
        sdfold = getonlySDF(temp);                                                            \
        temp(1) += 2 * dx;                                                                    \
        double grady = getonlySDF(temp) - sdfold;                                             \
        Eigen::Vector3d grad = Eigen::Vector3d(gradx, grady, 0) / (2 * dx);                   \
        return grad;                                                                          \
    }                                                                                         \
    inline double getSDFwithGrad1(const Eigen::RowVector3d &pos_rel, Eigen::Vector3d &grad) { \
        double sdf;                                                                           \
        grad = helperfunc(pos_rel, sdf);                                                      \
        return sdf;                                                                           \
    }

class Range {
public:
    double left_board;
    double right_board;
    Range(double l, double r) {
        left_board = l;
        right_board = r;
    }
};

namespace shape {

const uint8_t or_mask[8] = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};
class BasicShape {
private:
    Config config;

    typedef class shapeKernel {
    public:
        // A dynamically allocated boolean array representing the occupancy grid of the shape.
        // The shapeKernel class represents the shape of the robot in a grid using a boolean array (bool *map). Each element in the array corresponds to a cell in the grid, where true indicates the
        // cell is occupied by the robot’s shape, and false indicates it is not.

        bool *map;
        // The total size of the map array.
        double yaw;
        int totalsize;
        int X_size;
        int Y_size;
        shapeKernel() { map = nullptr; }
        shapeKernel(const shapeKernel &other) {
            map = new bool[other.totalsize];
            memcpy(map, other.map, other.totalsize * sizeof(bool));

            yaw = other.yaw;
            totalsize = other.totalsize;
            X_size = other.X_size;
            Y_size = other.Y_size;
        }
        ~shapeKernel() { delete[] map; }

        inline void init(int xsize_, int ysize_) {
            X_size = xsize_;
            Y_size = ysize_;
            totalsize = X_size * Y_size;
            map = new bool[totalsize]();  // 初始化全为0即false
        }

        inline int toAddr(const int idx, const int idy) const { return idx * Y_size + idy; }

        inline bool getOccupied(const int idx, const int idy) const { return map[toAddr(idx, idy)]; }
    } Shapekernel;
    // The byteShapeKernel class is a more compact representation of the robot’s shape. Instead of using a boolean array, it uses a byte array (uint8_t *map). Each byte can represent up to 8 cells,
    // with each bit in the byte corresponding to a cell’s occupancy status.
    typedef class byteShapeKernel {
    public:
        uint8_t *map;
        double yaw;
        int totalsize;
        int X_size;
        int Y_size;
        byteShapeKernel() { map = nullptr; }
        byteShapeKernel(const byteShapeKernel &other) {
            map = new uint8_t[other.totalsize];
            memcpy(map, other.map, other.totalsize * sizeof(uint8_t));

            yaw = other.yaw;
            totalsize = other.totalsize;
            X_size = other.X_size;
            Y_size = other.Y_size;
        }
        ~byteShapeKernel() { delete[] map; }

        inline bool getOccupied(const int idx, const int idy) const {
            int bytes_of_last_dim = (Y_size + 7) / 8;
            int byteIdx = (idx * bytes_of_last_dim) + (idy) / 8;
            int offset = idy % 8;

            if ((map[byteIdx] & or_mask[offset])) {
                return true;
            } else {
                return false;
            }
            return false;
        }

        inline void generateByteKernel(int kernel_size, const Shapekernel &ori_kernel) {
            int kernel_size_y = kernel_size;
            int bytes_len_of_last_dim = (kernel_size + 7) / 8;
            int bytes_len = kernel_size * bytes_len_of_last_dim;
            X_size = Y_size = kernel_size;
            map = new uint8_t[bytes_len];
            memset(map, 0, bytes_len);

            for (int x = 0; x < kernel_size; x++) {
                for (int y = 0; y < kernel_size; y++) {
                    int byte_idx = (x * bytes_len_of_last_dim) + (y + 0) / 8;
                    int byte_offset = y % 8;
                    if (ori_kernel.getOccupied(x, y) == true) {
                        map[byte_idx] |= or_mask[byte_offset];
                    }
                }
            }
        }

    } ByteShapeKernel;

    // ================for collision kernels==================
    int kernel_count{-1};
    int kernelsize{-1};
    double kernelresu{-1};

    typedef struct numSDFGridCell {
        Eigen::Vector3d gradient{Eigen::Vector3d::Zero()};
        double distance{0.0};
    } NumSDFGridCell;

    // for numeric gradients
    NumSDFGridCell *num_sdf_map{nullptr};
    int num_sdf_map_X_size;
    int num_sdf_map_Y_size;
    double num_sdf_map_res;
    double num_sdf_map_xlength;
    double num_sdf_map_ylength;
    double x_min;
    double y_min;
    bool initsdfdone{false};
    bool initselfkerneldone{false};

public:
    nav_msgs::OccupancyGrid grid;
    Eigen::Vector3d trans;
    Eigen::Matrix3d Rotate;
    double yaw;
    Eigen::Matrix3Xd mesh_var;
    Eigen::Vector3d interior_var{Eigen::Vector3d::Zero()};
    Eigen::Matrix3Xd vertices;
    Eigen::Matrix3Xd vertices_var;
    Eigen::Matrix3Xd mesh;
    Eigen::MatrixXi F;
    Eigen::MatrixXd V;
    Eigen::MatrixXd V_var;

    Eigen::MatrixXd V_end;    // blender obj
    Eigen::MatrixXd V_start;  // blender obj

    typedef std::shared_ptr<BasicShape> Ptr;
    BasicShape() = delete;
    Shapekernel *shape_kernels{nullptr};
    ByteShapeKernel *byte_shape_kernels{nullptr};
    igl::AABB<Eigen::MatrixXd, 3> tree;
    igl::FastWindingNumberBVH fwn_bvh;

    // 这些默认实现只有在派生类忘记覆写时才会被调用。以前它们的函数体是空的，
    // 落到函数尾部而不返回值是未定义行为，实际表现为随机崩溃（例如 std::bad_alloc）。
    // 现在改为返回确定值并报错，方便定位缺失的覆写。
    virtual double getonlySDF(const Eigen::RowVector3d &pos_rel) {
        ROS_ERROR_STREAM("[BasicShape] getonlySDF(pos) is not overridden by the derived shape.");
        return 0.0;
    };
    virtual double getonlySDF(const Eigen::RowVector3d &pos_rel, const Eigen::Matrix3d &R_obj) {
        ROS_ERROR_STREAM("[BasicShape] getonlySDF(pos, R) is not overridden by the derived shape.");
        return 0.0;
    };
    virtual Eigen::Vector3d getonlyGrad1(const Eigen::RowVector3d &pos_rel) {
        ROS_ERROR_STREAM("[BasicShape] getonlyGrad1 is not overridden by the derived shape.");
        return Eigen::Vector3d::Zero();
    };
    virtual double getSDFwithGrad1(const Eigen::RowVector3d &pos_rel, Eigen::Vector3d &grad) {
        ROS_ERROR_STREAM("[BasicShape] getSDFwithGrad1 is not overridden by the derived shape.");
        grad.setZero();
        return 0.0;
    };
    virtual Eigen::Matrix3d getonlyGrad2(const Eigen::RowVector3d &pos_rel) {
        ROS_ERROR_STREAM("[BasicShape] getonlyGrad2 is not overridden by the derived shape.");
        return Eigen::Matrix3d::Zero();
    };
    BasicShape(const Config &conf) : config(conf) {
        cout << "==========init BasicShape constructor =========" << endl;
        kernel_count = conf.kernel_yaw_num;
        kernelsize = conf.kernel_size;
        kernelresu = conf.occupancy_resolution;

        num_sdf_map_res = 0.1;
        num_sdf_map_xlength = num_sdf_map_ylength = 1.0;
        num_sdf_map_X_size = num_sdf_map_Y_size = 10;

        std::vector<double> para;
        para = config.poly_params;
        assert(para.size() == 3 && " set the para size==3");
        string input = ros::package::getPath(string("grid_path_searcher")) + "/" + config.inputdata;
        // V is a #N by 3 matrix which stores the coordinates of the vertices. Each row stores the coordinate of a vertex, with its x,y and z coordinates in the first, second and third column,
        // respectively. The matrix F stores the triangle connectivity: each line of F denotes a triangle whose 3 vertices are represented as indices pointing to rows of V.
        igl::read_triangle_mesh(input, V, F);  // read V and F from obj
        ROS_WARN_STREAM("use input obj file as analytic shape");
        trans = Eigen::Vector3d(para[0], para[1], 0.0);
        // This converts the third element of para from degrees to radians.
        yaw = (para[2] * PI / 180.0);
        // This sets up a 3x3 rotation matrix for rotation around the Z-axis (yaw rotation).
        Rotate.setZero();
        Rotate(0, 0) = std::cos(yaw);
        Rotate(0, 1) = -std::sin(yaw);
        Rotate(1, 0) = std::sin(yaw);
        Rotate(1, 1) = std::cos(yaw);
        Rotate(2, 2) = 1;
        // This creates a 4x4 transformation matrix that includes both rotation and translation.
        // The upper-left 3x3 block is set to the rotation matrix, and the rightmost column (except the bottom
        // element) is set to the translation vector.
        Eigen::Matrix4d Trans{Eigen::Matrix4d::Identity()};
        Trans.block(0, 0, 3, 3) = Rotate;
        Eigen::Matrix4d Transorig{Eigen::Matrix4d::Identity()};
        Trans.block(0, 3, 3, 1) = trans;
        const Eigen::Matrix4d TT = Trans.cast<double>().transpose();
        V = (V.rowwise().homogeneous() * TT).rowwise().hnormalized();  // x3d rotation transform
        vertices = V.transpose();
        mesh.conservativeResize(3, 3 * F.rows());
        int faces = F.rows();
        for (int i = 0; i < faces; i++) {
            mesh.col(3 * i) = V.row(F.row(i)[0]);
            mesh.col(3 * i + 1) = V.row(F.row(i)[1]);
            mesh.col(3 * i + 2) = V.row(F.row(i)[2]);
        }
        tree.init(V, F);
        igl::fast_winding_number(V, F, 2, fwn_bvh);
    }

    void inline writetoobjfile(const std::string &output_path) {
        V_var = vertices_var.transpose();
        igl::write_triangle_mesh(output_path, V_var, F);
    }

    void inline Transform(const Matrix3d &R, const Vector3d &trans_ = Vector3d::Zero(), const Matrix3d &St = Matrix3d::Identity()) {
        trans = trans_;
        Rotate = R;
        vertices_var = (St * R * vertices).colwise() + trans;
        mesh_var = (St * R * mesh).colwise() + trans;
        interior_var = trans;
    }

    virtual inline double getonlySDF_igl(const Eigen::RowVector3d &point_relative) {
        int i;
        Eigen::RowVector3d c;
        Eigen::VectorXd w{Eigen::VectorXd::Zero(1)};
        igl::fast_winding_number(fwn_bvh, 2.0, point_relative, w);
        double s = 1. - 2. * w(0);
        return s * sqrt(tree.squared_distance(V, F, point_relative, i, c));
    }

    virtual void inline setStartEnd(const Matrix3d &R, const Vector3d &trans_, const Matrix3d &St = Matrix3d::Identity(), bool start = true) {
        trans = trans_;
        Rotate = R;
        vertices_var = (St * R * vertices).colwise() + trans;
        if (start) {
            V_start = vertices_var.transpose();
        } else {
            V_end = vertices_var.transpose();
        }
    }

    void getTransform(Matrix3d &R, RowVector3d &trans_) {
        R = Rotate;
        trans_ = trans.transpose();
        interior_var = trans;
    }
    ~BasicShape() {
        std::cout << "==========delete BasicShape=========" << std::endl;
        delete[] num_sdf_map;
        delete[] shape_kernels;
        delete[] byte_shape_kernels;
    }

    vector<double> getbound() {
        vector<double> bounds;
        bounds.clear();
        double xmax = x_min + (num_sdf_map_X_size - 1) * num_sdf_map_res;
        double ymax = y_min + (num_sdf_map_Y_size - 1) * num_sdf_map_res;
        bounds.emplace_back(x_min);
        bounds.emplace_back(y_min);
        bounds.emplace_back(xmax);
        bounds.emplace_back(ymax);
        return bounds;
    }

    void initShape(const double ndx, const double ndy, const double nres = 0.1) {
        std::cout << "==============enter initShape==============" << std::endl;
        std::cout << "==============calculate robot self kernels==============" << std::endl;
        // kernel_count 代表着机器人的360度yaw角被分成了多少份
        int zero_yaw_ind = (kernel_count) / 2;
        int size_side = 0.5 * (kernelsize - 1);
        double yaw_, x, y, sdf;
        Eigen::Matrix3d rot_mat;
        Eigen::RowVector3d pos_ori;
        // represents the shape of the robot at a specific yaw angle.
        shape_kernels = new Shapekernel[kernel_count];
        byte_shape_kernels = new ByteShapeKernel[kernel_count];
        // default front_end_safeh = 0, occupancy_resolution = 0.2
        double safemargin = max(config.front_end_safeh, config.occupancy_resolution / 2);

        // thus safemargin = 0.1
        // double safemargin = 0.15;
        double yaw_res = 2 * PI / kernel_count;
        // Count kernels with an integer: accumulating yaw += yaw_res in floating
        // point runs one extra iteration for many kernel counts (e.g. 36) and
        // writes past the end of both arrays.
        for (int ind = 0; ind < kernel_count; ind++) {
            double yaw = -PI + ind * yaw_res;
            shape_kernels[ind].init(kernelsize, kernelsize);
            // Kernel ind must hold the robot at exactly the yaw that
            // PathPoint(pos, ind) hands to the back end (-PI + ind * yaw_res).
            rot_mat = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

            shape_kernels[ind].yaw = yaw;
            byte_shape_kernels[ind].yaw = yaw;

            for (int a = 0; a < kernelsize; a++) {
                for (int b = 0; b < kernelsize; b++) {
                    // kernelresu = occupancy_resolution
                    x = kernelresu * a - size_side * kernelresu;
                    y = kernelresu * b - size_side * kernelresu;
                    pos_ori(0) = x;
                    pos_ori(1) = y;
                    pos_ori(2) = 0.0;
                    sdf = getonlySDF(pos_ori, rot_mat);
                    // The map is used to represent the occupancy grid of the robot's shape at each orientation.
                    if (sdf <= safemargin) {
                        shape_kernels[ind].map[a * kernelsize + b] = true;
                    }
                }
            }
            byte_shape_kernels[ind].generateByteKernel(kernelsize, shape_kernels[ind]);
            printByteShapeKernel(byte_shape_kernels[ind]);

            std::cout << " Byte kernel " << ind << " init done. " << std::endl;
            cout << "----------------------" << endl;
        }
        initselfkerneldone = true;
    }

    void printByteShapeKernel(const ByteShapeKernel &byte_kernel) {
        std::cout << "Orientation ByteShapeKernel (Yaw): " << byte_kernel.yaw << " radians" << std::endl;
        std::cout << "Occupancy Grid ByteShapeKernel (X_size: " << byte_kernel.X_size << ", Y_size: " << byte_kernel.Y_size << "):" << std::endl;

        for (int j = byte_kernel.Y_size - 1; j >= 0; --j) {
            for (int i = 0; i < byte_kernel.X_size; ++i) {
                if (byte_kernel.getOccupied(i, j)) {
                    std::cout << "X ";  // Occupied
                } else {
                    std::cout << ". ";  // Free
                }
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }
    void printShapeKernel(const Shapekernel &shape_kernel) {
        std::cout << "Orientation (Yaw): " << shape_kernel.yaw << " radians" << std::endl;
        std::cout << "Occupancy Grid (X_size: " << shape_kernel.X_size << ", Y_size: " << shape_kernel.Y_size << "):" << std::endl;

        for (int j = shape_kernel.Y_size - 1; j >= 0; --j) {
            for (int i = 0; i < shape_kernel.X_size; ++i) {
                if (shape_kernel.getOccupied(i, j)) {
                    std::cout << "X ";  // Occupied
                } else {
                    std::cout << ". ";  // Free
                }
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }
};

class star : public BasicShape {
public:
    const double r{2.8};
    const double rf{0.6};
    Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
    Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};
    star(const Config &conf) : BasicShape(conf) {
        getTransform(Rotate, trans);
        std::cout << "======Rotation:" << Rotate << "======" << std::endl;
        std::cout << "======trans:" << trans.transpose() << "======" << std::endl;
        initShape(8, 8, conf.occupancy_resolution);
    }

public:
    typedef std::shared_ptr<star> Ptr;
    inline double clip(double value, double min_val, double max_val) { return std::max(std::min(value, max_val), min_val); }

    inline double getonlySDF(const Eigen::RowVector3d &pos_rel) {
        Eigen::RowVector2d Pos_rel = ((pos_rel - trans) * Rotate).head(2);

        const Eigen::RowVector2d k1(0.809016994375, -0.587785252292);
        const Eigen::RowVector2d k2(-k1.x(), k1.y());
        Pos_rel.x() = std::abs(Pos_rel.x());
        Pos_rel -= 2.0 * std::max(k1.dot(Pos_rel), 0.0) * k1;
        Pos_rel -= 2.0 * std::max(k2.dot(Pos_rel), 0.0) * k2;
        Pos_rel.x() = std::abs(Pos_rel.x());
        Pos_rel.y() -= r;

        Eigen::RowVector2d ba = rf * Eigen::RowVector2d(-k1.y(), k1.x()) - Eigen::RowVector2d(0, 1);
        double h = clip(Pos_rel.dot(ba) / ba.dot(ba), 0.0, r);
        Eigen::RowVector2d diff = Pos_rel - ba * h;

        return diff.norm() * std::copysign(1.0, Pos_rel.y() * ba.x() - Pos_rel.x() * ba.y());
    }

    inline double getonlySDF(const Eigen::RowVector3d &pos_rel, const Eigen::Matrix3d &R_obj) {
        Eigen::RowVector2d Pos_rel = ((pos_rel - trans) * Rotate * R_obj).head(2);
        const Eigen::RowVector2d k1(0.809016994375, -0.587785252292);
        const Eigen::RowVector2d k2(-k1.x(), k1.y());
        Pos_rel.x() = std::abs(Pos_rel.x());
        Pos_rel -= 2.0 * std::max(k1.dot(Pos_rel), 0.0) * k1;
        Pos_rel -= 2.0 * std::max(k2.dot(Pos_rel), 0.0) * k2;
        Pos_rel.x() = std::abs(Pos_rel.x());
        Pos_rel.y() -= r;

        Eigen::RowVector2d ba = rf * Eigen::RowVector2d(-k1.y(), k1.x()) - Eigen::RowVector2d(0, 1);
        double h = clip(Pos_rel.dot(ba) / ba.dot(ba), 0.0, r);
        Eigen::RowVector2d diff = Pos_rel - ba * h;

        return diff.norm() * std::copysign(1.0, Pos_rel.y() * ba.x() - Pos_rel.x() * ba.y());
    }
    DEFINE_USEFUL_FUNCTION()
    ~star() {};
};

class sdDemo : public BasicShape {
public:
    Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
    Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};
    sdDemo(const Config &conf) : BasicShape(conf) {
        // get initial config set in line 236
        getTransform(Rotate, trans);
        std::cout << "======Rotation:" << Rotate << "======" << std::endl;
        std::cout << "======trans:" << trans.transpose() << "======" << std::endl;
        initShape(8, 8, conf.occupancy_resolution);
    }

public:
    typedef std::shared_ptr<sdDemo> Ptr;
    // must set the third element of pos_rel to 0, unless the sdf value is measurese in 3D, which is above the plane.

    inline double getonlySDF(const Eigen::RowVector3d &pos_rel) {
        // cout << "getonlySDF success in sdDemo" << endl;
        Eigen::RowVector3d new_pos = pos_rel;
        new_pos(2) = 0.0;
        // pos_rel is already in local coordinates thanks to posEva2Rel

        return getonlySDF_igl(new_pos);
    }

    inline double getonlySDF(const Eigen::RowVector3d &pos_rel, const Eigen::Matrix3d &R_obj) {
        Eigen::RowVector3d new_pos = pos_rel;
        new_pos(2) = 0.0;
        // Transform the point to the sdDemo's local coordinate system, including the additional rotation
        Eigen::RowVector3d local_pos = (new_pos - trans) * Rotate * R_obj;
        // Use the IGL-based SDF calculation
        return getonlySDF_igl(local_pos);
    }

    // You can keep any additional methods or properties specific to the sdDemo shape here
    DEFINE_USEFUL_FUNCTION()
    ~sdDemo() {}
};

class sdLShape : public BasicShape {
public:
    Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
    Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};
    sdLShape(const Config &conf) : BasicShape(conf) {
        // get initial config set in line 236
        getTransform(Rotate, trans);
        std::cout << "======Rotation:" << Rotate << "======" << std::endl;
        std::cout << "======trans:" << trans.transpose() << "======" << std::endl;
        initShape(8, 8, conf.occupancy_resolution);
    }

public:
    typedef std::shared_ptr<sdLShape> Ptr;
    // must set the third element of pos_rel to 0, unless the sdf value is measurese in 3D, which is above the plane.

    inline double getonlySDF(const Eigen::RowVector3d &pos_rel) {
        Eigen::RowVector3d new_pos = pos_rel;
        new_pos(2) = 0.0;
        // Transform the point to the sdLShape's local coordinate system
        Eigen::RowVector3d local_pos = (new_pos - trans);
        // Use the IGL-based SDF calculation
        return getonlySDF_igl(local_pos);
    }

    inline double getonlySDF(const Eigen::RowVector3d &pos_rel, const Eigen::Matrix3d &R_obj) {
        Eigen::RowVector3d new_pos = pos_rel;
        new_pos(2) = 0.0;
        // Transform the point to the sdLShape's local coordinate system, including the additional rotation
        Eigen::RowVector3d local_pos = (new_pos - trans) * R_obj;
        // Use the IGL-based SDF calculation
        return getonlySDF_igl(local_pos);
    }

    // You can keep any additional methods or properties specific to the sdLShape shape here
    DEFINE_USEFUL_FUNCTION()
    ~sdLShape() {}
};

class sdTShape : public BasicShape {
public:
    Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
    Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};
    sdTShape(const Config &conf) : BasicShape(conf) {
        // get initial config set in line 236
        getTransform(Rotate, trans);
        std::cout << "======Rotation:" << Rotate << "======" << std::endl;
        std::cout << "======trans:" << trans.transpose() << "======" << std::endl;
        initShape(8, 8, conf.occupancy_resolution);
    }

public:
    typedef std::shared_ptr<sdTShape> Ptr;
    // must set the third element of pos_rel to 0, unless the sdf value is measurese in 3D, which is above the plane.

    inline double getonlySDF(const Eigen::RowVector3d &pos_rel) {
        // Transform to local coordinates
        Eigen::RowVector2d p = ((pos_rel - trans) * Rotate).head(2);

        // Adjusted dimensions to match the grid pattern
        const double vBarWidth = 0.1;   // Increased width of vertical bar
        const double vBarHeight = 0.6;  // Height of vertical bar
        const double hBarWidth = 0.6;   // Increased width of horizontal bar
        const double hBarHeight = 0.2;  // Adjusted height of horizontal bar

        // Adjust center positions
        const double vBarOffsetX = 0.12;  // Right section
        const double hBarOffsetX = -0.1;  // Left section

        // Vertical bar distance calculation
        Eigen::RowVector2d d1 = (p - Eigen::RowVector2d(vBarOffsetX, 0)).cwiseAbs() - Eigen::RowVector2d(vBarWidth / 2, vBarHeight);
        double dist1 = d1.cwiseMax(0.0).norm() + std::min(std::max(d1.x(), d1.y()), 0.0);

        // Horizontal bar distance calculation
        Eigen::RowVector2d d2 = (p - Eigen::RowVector2d(hBarOffsetX, 0)).cwiseAbs() - Eigen::RowVector2d(hBarWidth / 2, hBarHeight);
        double dist2 = d2.cwiseMax(0.0).norm() + std::min(std::max(d2.x(), d2.y()), 0.0);

        // Use a smaller smoothing factor for sharper corners
        double k = 0.005;
        double h = std::max(k - std::abs(dist1 - dist2), 0.0) / k;
        return std::min(dist1, dist2) - h * h * k * 0.25;
    }

    inline double getonlySDF(const Eigen::RowVector3d &pos_rel, const Eigen::Matrix3d &R_obj) {
        // Transform to local coordinates with rotation
        Eigen::RowVector2d p = ((pos_rel - trans) * Rotate * R_obj).head(2);

        // Use same dimensions as the non-rotated version for consistency
        const double vBarWidth = 0.1;   // Increased width of vertical bar
        const double vBarHeight = 0.6;  // Height of vertical bar
        const double hBarWidth = 0.6;   // Increased width of horizontal bar
        const double hBarHeight = 0.2;  // Adjusted height of horizontal bar

        // Same center positions as non-rotated version
        const double vBarOffsetX = 0.12;  // Right section
        const double hBarOffsetX = -0.1;  // Left section
        // Vertical bar distance calculation
        Eigen::RowVector2d d1 = (p - Eigen::RowVector2d(vBarOffsetX, 0)).cwiseAbs() - Eigen::RowVector2d(vBarWidth / 2, vBarHeight);
        double dist1 = d1.cwiseMax(0.0).norm() + std::min(std::max(d1.x(), d1.y()), 0.0);

        // Horizontal bar distance calculation
        Eigen::RowVector2d d2 = (p - Eigen::RowVector2d(hBarOffsetX, 0)).cwiseAbs() - Eigen::RowVector2d(hBarWidth / 2, hBarHeight);
        double dist2 = d2.cwiseMax(0.0).norm() + std::min(std::max(d2.x(), d2.y()), 0.0);

        // Use same small smoothing factor for consistent sharp corners
        double k = 0.005;
        double h = std::max(k - std::abs(dist1 - dist2), 0.0) / k;
        return std::min(dist1, dist2) - h * h * k * 0.25;
    }

    // You can keep any additional methods or properties specific to the sdTShape shape here
    DEFINE_USEFUL_FUNCTION()
    ~sdTShape() {}
};

// 通用的输入网格形状：当 inputdata 的文件名不在解析式形状表里时使用。
// SDF 全部由 libigl 的 AABB 树 + fast winding number 查询，因此任何 obj 文件都可以直接使用。
class MeshShape : public BasicShape {
public:
    Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
    Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};
    MeshShape(const Config &conf) : BasicShape(conf) {
        getTransform(Rotate, trans);
        std::cout << "======Rotation:" << Rotate << "======" << std::endl;
        std::cout << "======trans:" << trans.transpose() << "======" << std::endl;
        initShape(8, 8, conf.occupancy_resolution);
    }

public:
    typedef std::shared_ptr<MeshShape> Ptr;

    inline double getonlySDF(const Eigen::RowVector3d &pos_rel) {
        Eigen::RowVector3d new_pos = pos_rel;
        new_pos(2) = 0.0;
        return getonlySDF_igl(new_pos);
    }

    inline double getonlySDF(const Eigen::RowVector3d &pos_rel, const Eigen::Matrix3d &R_obj) {
        Eigen::RowVector3d new_pos = pos_rel;
        new_pos(2) = 0.0;
        Eigen::RowVector3d local_pos = (new_pos - trans) * Rotate * R_obj;
        return getonlySDF_igl(local_pos);
    }

    DEFINE_USEFUL_FUNCTION()
    ~MeshShape() {}
};

class Polygon : public BasicShape {
public:
    typedef class PolygonEdge {
    public:
        Eigen::Vector2d start;
        Eigen::Vector2d end;
        PolygonEdge(const Eigen::Vector2d &s, const Eigen::Vector2d &e) : start(s), end(e) {}
        inline bool isCrossRayOnXDir(Eigen::Vector2d source) {
            Eigen::Vector2d s2p = start - source;
            Eigen::Vector2d e2p = end - source;
            double theta_s = atan2(s2p(1), s2p(0));
            double theta_e = atan2(e2p(1), e2p(0));
            theta_s = (theta_s < 0.0) ? (theta_s + 2 * PI) : theta_s;
            theta_e = (theta_e < 0.0) ? (theta_e + 2 * PI) : theta_e;
            double d1 = abs(theta_s - theta_e);
            if (d1 < PI)
                return false;
            else
                return true;
        }
        inline double dis2Seg(const Eigen::Vector2d &p, Eigen::Vector2d &c) {
            Eigen::Vector2d v = end - start;
            Eigen::Vector2d w = p - start;
            double t = w.dot(v) / (v.squaredNorm());
            if (t < 0.0) {
                t = 0.0;
            } else if (t > 1.0) {
                t = 1.0;
            }

            c = start + t * v;
            return (p - c).norm();
        }
    } Edge;

public:
    Eigen::MatrixX3d edge_start;
    Eigen::MatrixX3d edge_end;
    std::vector<Edge> edges;  // 可视化不会访问到
public:
    Polygon(const Config &conf) : BasicShape(conf) {
#define POLYGON_DEFAULT_EDGES 7
        edge_start.resize(POLYGON_DEFAULT_EDGES, 3);
        edge_end.resize(POLYGON_DEFAULT_EDGES, 3);
        Eigen::Vector2d edge_s, edge_e;
        double dangle = 2 * PI / POLYGON_DEFAULT_EDGES;
        for (int i = 0; i < POLYGON_DEFAULT_EDGES; i++) {
            edge_s = Eigen::Vector2d(4 * cos((i - 0.5) * dangle), 4 * sin((i - 0.5) * dangle));
            edge_e = Eigen::Vector2d(4 * cos((i + 0.5) * dangle), 4 * sin((i + 0.5) * dangle));
            Edge e(edge_s, edge_e);
            edges.push_back(e);
            edge_start.row(i) = Eigen::Vector3d(edge_s(0), edge_s(1), 0);
            edge_end.row(i) = Eigen::Vector3d(edge_e(0), edge_e(1), 0);
        }
        initShape(8, 8, conf.occupancy_resolution);
    }

    Polygon(const Config &conf, const Eigen::MatrixX2d &vertexes) : BasicShape(conf) {
        const int edge_count = vertexes.rows();
        edge_start.resize(edge_count, 3);
        edge_end.resize(edge_count, 3);
        Eigen::Vector2d edge_s, edge_e;
        for (size_t i = 0; i < edge_count; i++) {
            edge_s = vertexes.row(i);
            edge_e = vertexes.row((i + 1) % edge_count);
            Edge e(edge_s, edge_e);
            edges.push_back(e);
            edge_start.row(i) = Eigen::Vector3d(edge_s(0), edge_s(1), 0);
            edge_end.row(i) = Eigen::Vector3d(edge_e(0), edge_e(1), 0);
        }
        initShape(8, 8, conf.occupancy_resolution);
    }

public:
    inline double getonlySDF(const Eigen::RowVector3d &pos_rel) {
        double dis_min = 1e9;
        Eigen::Vector2d prel2(pos_rel(0), pos_rel(1));
        Eigen::Vector2d c;
        Eigen::Vector2d cmin;
        int rs = 0;
        for (size_t i = 0; i < edges.size(); i++) {
            double dis = edges[i].dis2Seg(prel2, c);
            if (dis < dis_min) {
                dis_min = dis;
                cmin = c;
            }
            if (edges[i].isCrossRayOnXDir(prel2)) {
                rs++;
            }
        }
        if (rs % 2 == 0) {
            return dis_min;
        } else {
            return -dis_min;
        }
    }
    inline double getonlySDF(const Eigen::RowVector3d &pos_rel, const Eigen::Matrix3d &R_obj) {
        // BasicShape::initShape 用 Matrix3d 调用，这里转成平面旋转再复用下面的二维实现。
        return getonlySDF(pos_rel, Eigen::Matrix2d(R_obj.topLeftCorner<2, 2>()));
    }
    inline double getonlySDF(const Eigen::RowVector3d &pos_rel, const Eigen::Matrix2d &R_obj) {
        std::cout << "getOnlySDF " << std::endl;
        double dis_min = 1e9;
        Eigen::Vector2d c;
        Eigen::Vector2d cmin;
        int rs = 0;
        Eigen::Vector2d pos_rel_rotated = R_obj.transpose() * pos_rel.head(2).transpose();
        for (size_t i = 0; i < edges.size(); i++) {
            double dis = edges[i].dis2Seg(pos_rel_rotated, c);
            if (dis < dis_min) {
                dis_min = dis;
                cmin = c;
            }
            if (edges[i].isCrossRayOnXDir(pos_rel_rotated)) {
                rs++;
            }
        }
        if (rs % 2 == 0) {
            return dis_min;
        } else {
            return -dis_min;
        }
    }

    inline Eigen::Vector3d getonlyGrad1(const Eigen::RowVector3d &pos_rel) {
        double dis_min = 1e9;
        Eigen::Vector2d c;
        Eigen::Vector2d cmin;
        int rs = 0;
        for (size_t i = 0; i < edges.size(); i++) {
            double dis = edges[i].dis2Seg(pos_rel.transpose().head(2), c);
            if (dis < dis_min) {
                dis_min = dis;
                cmin = c;
            }
            if (edges[i].isCrossRayOnXDir(pos_rel.transpose().head(2))) {
                rs++;
            }
        }
        if (rs % 2 == 0) {
            return (pos_rel.transpose() - Eigen::Vector3d(cmin(0), cmin(1), pos_rel(2))).normalized();
        } else {
            return -(pos_rel.transpose() - Eigen::Vector3d(cmin(0), cmin(1), pos_rel(2))).normalized();
        }
    }
    inline double getSDFwithGrad1(const Eigen::RowVector3d &pos_rel, Eigen::Vector3d &grad) {
        double dis_min = 1e9;
        Eigen::Vector2d c;
        Eigen::Vector2d cmin;
        int rs = 0;
        for (size_t i = 0; i < edges.size(); i++) {
            double dis = edges[i].dis2Seg(pos_rel.transpose().head(2), c);
            if (dis < dis_min) {
                dis_min = dis;
                cmin = c;
            }
            if (edges[i].isCrossRayOnXDir(pos_rel.transpose().head(2))) {
                rs++;
            }
        }
        if (rs % 2 == 0) {
            grad = (pos_rel.transpose() - Eigen::Vector3d(cmin(0), cmin(1), pos_rel(2))).normalized();
            return dis_min;
        } else {
            grad = -(pos_rel.transpose() - Eigen::Vector3d(cmin(0), cmin(1), pos_rel(2))).normalized();
            return -dis_min;
        }
    }
    inline Eigen::Matrix3d getonlyGrad2(const Eigen::RowVector3d &pos_rel) {
        Eigen::Matrix3d ret;
        ret.setZero();
        return ret;
    };
};
};  // namespace shape

#endif
