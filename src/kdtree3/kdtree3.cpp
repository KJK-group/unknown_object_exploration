#include "kdtree3/kdtree3.hpp"
#include "kdtree3/utils.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <variant>

using Eigen::Vector3f;

namespace kdtree {

kdtree3::kdtree3(const std::vector<Vector3f> &points) {
  // todo: figure out how insert should handle duplicates.
  std::for_each(points.cbegin(), points.cend(),
                [this](const Vector3f &point) { insert(point); });
  // version 2. find median of points
  //   auto xs = std::vector<float>(points.size());
  //   std::transform(points.cbegin(), points.cend(), xs.begin(),
  //                  [](const Vector3f &p) { return p.x(); });
  //   const auto median_x = utils::median(xs);
  //   auto left_subtree_it = std::stable_partition(
  //       points.begin(), points.end(),
  //       [=](const auto &point) { return point.x() < median_x; });
}

auto compare_positions_based_on_depth(const Vector3f &p1, const Vector3f &p2,
                                      unsigned int depth) -> bool {
  switch (depth % 3) {
  case 0:
    return p1.x() < p2.x();
    break;
  case 1:
    return p1.y() < p2.y();
    break;
  case 2:
    return p1.z() < p2.z();
    break;
  };
}

auto kdtree3::insert(std::unique_ptr<Node> &node, const Vector3f &value,
                     unsigned int depth) -> void {
  if (node) {
    auto less_than =
        compare_positions_based_on_depth(value, node->position_, depth);

    insert(less_than ? node->left_ : node->right_, value, depth + 1);
  } else {
    node = std::make_unique<Node>(value, depth + 1);
  }
}

auto kdtree3::contains(std::unique_ptr<Node> &node,
                       const Eigen::Vector3f &value, unsigned int depth)
    -> bool {

  if (!node) {
    return false;
  }
  if (node->position_ == value) {
    return true;
  }

  auto less_than =
      compare_positions_based_on_depth(value, node->position_, depth);
  return contains(less_than ? node->left_ : node->right_, value, depth + 1);
}

// todo: implementi
auto kdtree3::knn_search(std::unique_ptr<Node> &node, unsigned int k,
                         const Eigen::Vector3f &value)
    -> std::vector<Eigen::Vector3f> {}

auto kdtree3::is_balanced() -> bool {
  return max_depth() < std::log2(n_nodes_);
}

auto kdtree3::max_depth() -> unsigned int {
  auto depths = std::vector<unsigned int>();
  inorder_traversal([&depths](const Eigen::Vector3f &_, unsigned int depth) {
    depths.push_back(depth);
  });

  return *std::max_element(depths.begin(), depths.end());
}

auto kdtree3::inorder_traversal(std::unique_ptr<Node> &node,
                                std::function<void(const Eigen::Vector3f &)> fn)
    -> void {

  if (!node) {
    return;
  }
  inorder_traversal(node->left_, fn);
  fn(node->position_);
  inorder_traversal(node->right_, fn);
}

auto kdtree3::inorder_traversal(
    std::unique_ptr<Node> &node,
    std::function<void(const Eigen::Vector3f &, unsigned int)> fn,
    unsigned int depth) -> void {

  if (!node) {
    return;
  }
  inorder_traversal(node->left_, fn, depth + 1);
  fn(node->position_, depth);
  inorder_traversal(node->right_, fn, depth + 1);
}

auto kdtree3::preorder_traversal(
    std::unique_ptr<Node> &node,
    std::function<void(const Eigen::Vector3f &)> fn) -> void {

  if (!node) {
    return;
  }
  fn(node->position_);

  preorder_traversal(node->left_, fn);
  preorder_traversal(node->right_, fn);
}

auto kdtree3::postorder_traversal(
    std::unique_ptr<Node> &node,
    std::function<void(const Eigen::Vector3f &)> fn) -> void {

  if (!node) {
    return;
  }
  postorder_traversal(node->left_, fn);
  postorder_traversal(node->right_, fn);
  fn(node->position_);
}

// auto kdtree3::insert(const Vector3f &point) -> bool {
//   auto depth = 0;
//   //   return insert_(point, root_, depth);
//   if (root_ == nullptr) {
//     root_ = std::make_unique<node>(point, 0, nullptr, nullptr);
//     ++n_nodes_;
//     return true;
//   }

//   return insert_(point, root_, depth);

//   //   return true;
// }

// auto kdtree3::insert_(const Vector3f &point, node *n, unsigned int depth)
//     -> bool {

//   if (n == nullptr) {
//     n = std::make_unique<node>(point, depth, nullptr, nullptr);
//   }
//   //   std::cout << "index is " << depth << " and tree size is " <<
//   tree_.size()
//   //             << std::endl;
//   //   if (depth > tree_.size() - 1) {
//   //     resize_tree_to_double_its_size_();
//   //   }
//   //   assert(0 < depth && depth < tree_.size());

//   const auto &node_at_index = tree_.at(depth);
//   auto index_points_at_leaf = node_at_index.index() == 0;
//   if (index_points_at_leaf) {

//     tree_[depth] = point;
//     ++n_nodes_;
//     return true;
//   }

//   auto depth = depth_(depth);
//   auto ordering = compare_nodes_based_on_depth(point, node_at_index, depth);
//   switch (ordering) {
//   case partial_ordering::less_than: // go left
//   {
//     auto left_child_index = depth * 2;
//     insert_(point, left_child_index); // recurse left
//   } break;
//   case partial_ordering::greater_than: // go right
//   {
//     auto right_child_index = depth * 2 + 1;
//     insert_(point, right_child_index); // recurse right
//   } break;
//   case partial_ordering::equal: // dunno
//     break;
//   };

//   return false;
// }

// inline auto kdtree3::calculate_left_child_idx_(unsigned int parent_idx) const
//     -> unsigned int {
//   return parent_idx * 2;
// }
// inline auto kdtree3::calculate_right_child_idx_(unsigned int parent_idx)
// const
//     -> unsigned int {
//   return parent_idx * 2 + 1;
// }
// auto kdtree3::inorder_traversal_(
//     std::function<void(const Eigen::Vector3f &)> fn,
//     unsigned int node_idx) const -> void {
//   auto node = tree_[node_idx];
//   try {
//     auto point = std::get<Vector3f>(node);
//     fn(point);
//     inorder_traversal_(fn, calculate_left_child_idx_(node_idx));
//     inorder_traversal_(fn, calculate_right_child_idx_(node_idx));
//   } catch (const std::bad_variant_access &ex) {
//   }
// }

// auto kdtree3::inorder_traversal(
//     std::function<void(const Eigen::Vector3f &)> fn) const -> void {
//   inorder_traversal_(fn, 1);
// }

// auto kdtree3::is_leaf_(unsigned int idx) const -> bool {
//   auto left_child_index = idx * 2;
//   auto right_child_index = idx * 2 + 1;
//   if (left_child_index > tree_.size() || right_child_index > tree_.size()) {
//     return false;
//   }
//   auto right_child = tree_[right_child_index];
//   auto left_child = tree_[left_child_index];
//   // ! fix: i don't like this. too brittle
//   auto left_child_empty = left_child.index() == 0;
//   auto right_child_empty = right_child.index() == 0;
//   return left_child_empty && right_child_empty;
// }

// auto kdtree3::has_children_(unsigned int idx) const -> bool {
//   return !is_leaf_(idx);
// }

// auto kdtree3::get_parent_idx_(unsigned int child_index) const -> unsigned int
// {
//   auto is_even = child_index % 2 == 0;
//   auto child_is_left_of_parent = is_even;
//   return child_index - (child_is_left_of_parent ? 0 : 1) / 2;
// }

// auto kdtree3::depth_(unsigned int index) const -> unsigned int {
//   return std::floor(std::log2(index));
// }

// auto kdtree3::resize_tree_to_double_its_size_() -> void {
//   //   tree_.resize(tree_.size() * 2, std::monostate);

//   tree_.resize(tree_.size() * 2, std::monostate);
//   std::cout << "resize tree to double its size. new size: " << tree_.size() *
//   2
//             << std::endl;
// }

} // namespace kdtree
