
#include "eigen3/Eigen/Dense"
#include <iostream>
#include <vector>

#include "kdtree3/kdtree3.hpp"

using namespace std;

// int main(int argc, char **argv) { return 0; }

int main(int argc, char const *argv[]) {

  vector<Eigen::Vector3f> points = {
      {0.0, 0.0, 0.0}, {1.0, 6.0, 0.0},  {2.0, -5., 2.0},
      {8.5, 1.0, 9.0}, {-1.0, 40, -1.0}, {2.0, -5., 6.0},
  };

  std::cout << "calling constructor" << std::endl;
  {
    auto kdtree3 = kdtree::kdtree3(points);

    std ::cout << "can construct kdtree3" << std::endl;

    std::cout << "inorder_traversal" << std::endl;
    kdtree3.inorder_traversal([](const Eigen::Vector3f &point) {
      std::cout << point.x() << " " << point.y() << " " << point.z()
                << std::endl;
    });
    std::cout << "postorder_traversal" << std::endl;
    kdtree3.postorder_traversal([](const Eigen::Vector3f &point) {
      std::cout << point.x() << " " << point.y() << " " << point.z()
                << std::endl;
    });

    std::cout << "preorder_traversal" << std::endl;
    kdtree3.preorder_traversal([](const Eigen::Vector3f &point) {
      std::cout << point.x() << " " << point.y() << " " << point.z()
                << std::endl;
    });

    for (auto point : points) {
      assert(kdtree3.contains(point));
    }

    auto point_not_in_tree = Eigen::Vector3f{10, 10, 10};
    assert(not kdtree3.contains(point_not_in_tree));
  }

  return 0;
}
