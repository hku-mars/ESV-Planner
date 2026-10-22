#ifndef MY_HEADER_H
#define MY_HEADER_H
#include <Eigen/Dense>
#include <vector>
#include <memory>
class GraphNode {
public:
    enum NODE_TYPE { Guard = 1, Connector = 2, Vertices = 3 };
    enum NODE_STATE { NEW = 1, CLOSE = 2, OPEN = 3 };

    GraphNode() {}
    GraphNode(Eigen::Vector2d pos, NODE_TYPE type, int id) : pos_(pos), type_(type), state_(NEW), id_(id) {}
    ~GraphNode() {}

    std::vector<std::shared_ptr<GraphNode>> neighbors_;
    Eigen::Vector2d pos_;
    NODE_TYPE type_;
    NODE_STATE state_;
    int id_;

    typedef std::shared_ptr<GraphNode> Ptr;
};

class PathNode {
public:
    int pathID;
    std::vector<Eigen::Vector2d> path;

    PathNode() {}
    PathNode(int id, std::vector<Eigen::Vector2d> path) : pathID(id), path(path) {}
    ~PathNode() {}
    typedef std::shared_ptr<PathNode> Ptr;
};


#endif // MY_HEADER_H