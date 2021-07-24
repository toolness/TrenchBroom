/*
 Copyright (C) 2010-2017 Kristian Duske

 This file is part of TrenchBroom.

 TrenchBroom is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 TrenchBroom is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with TrenchBroom. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "Exceptions.h"

#include <vecmath/scalar.h>
#include <vecmath/bbox.h>
#include <vecmath/bbox_io.h>
#include <vecmath/ray.h>
#include <vecmath/intersection.h>

#include <kdl/opt_utils.h>
#include <kdl/overload.h>

#include <cassert>
#include <iosfwd>
#include <unordered_map>
#include <variant>
#include <vector>

namespace TrenchBroom {
    struct AABBFreeNode {
        std::optional<size_t> next;

        friend bool operator==(const AABBFreeNode& lhs, const AABBFreeNode& rhs) {
            return lhs.next == rhs.next;
        }

        friend bool operator!=(const AABBFreeNode& lhs, const AABBFreeNode& rhs) {
            return !(lhs == rhs);
        }

        friend std::ostream& operator<<(std::ostream& str, const AABBFreeNode& node) {
            str << "AABBFreeNode{next: " << kdl::opt_to_string(node.next) << "}";
            return str;
        }
    };

    template <typename T, size_t S>
    struct AABBInnerNode {
        vm::bbox<T, S> bounds;
        std::optional<size_t> parentIndex;
        size_t leftChildIndex;
        size_t rightChildIndex;
        size_t height;

        friend bool operator==(const AABBInnerNode& lhs, const AABBInnerNode& rhs) {
            return lhs.bounds == rhs.bounds 
                && lhs.height == rhs.height;
        }

        friend bool operator!=(const AABBInnerNode& lhs, const AABBInnerNode& rhs) {
            return !(lhs == rhs);
        }

        friend std::ostream& operator<<(std::ostream& str, const AABBInnerNode& node) {
            str << "AABBInnerNode{bounds: " << node.bounds 
                << ", height: " << node.height << "}";
            return str;
        }
    };

    template <typename T, size_t S, typename U>
    struct AABBLeafNode {
        vm::bbox<T, S> bounds;
        std::optional<size_t> parentIndex;
        U data;

        friend bool operator==(const AABBLeafNode& lhs, const AABBLeafNode& rhs) {
            return lhs.bounds == rhs.bounds 
                && lhs.data == rhs.data;
        }

        friend bool operator!=(const AABBLeafNode& lhs, const AABBLeafNode& rhs) {
            return !(lhs == rhs);
        }

        friend std::ostream& operator<<(std::ostream& str, const AABBLeafNode& node) {
            str << "AABBLeafNode{bounds: " << node.bounds
                << ", data: " << node.data << "}";
            return str;
        }
    };

    template <typename T, size_t S, typename U>
    using AABBNode = std::variant<AABBFreeNode, AABBInnerNode<T, S>, AABBLeafNode<T, S, U>>;

    template <typename T, size_t S, typename U>
    std::ostream& operator<<(std::ostream& str, const AABBNode<T, S, U>& node) {
        std::visit([&](const auto& x) {
            str << x;
        }, node);
        return str;
    }

    /**
    * An axis aligned bounding box tree that allows for quick ray intersection queries.
    *
    * @tparam T the floating point type
    * @tparam S the number of dimensions for vector types
    * @tparam U the node data to store in the leafs
    */
    template <typename T, size_t S, typename U>
    class AABBTree2 {
    public:
        using Node = AABBNode<T, S, U>;
        using FreeNode = AABBFreeNode;
        using InnerNode = AABBInnerNode<T, S>;
        using LeafNode = AABBLeafNode<T, S, U>;
    private:
        std::vector<Node> m_nodes;
        std::unordered_map<U, size_t> m_leafForData;
        std::optional<size_t> m_freeNode;
    public:
        AABBTree2(const size_t numLeafs = 0) {
            if (numLeafs > 0) {
                const size_t numNodes = (static_cast<size_t>(std::log2(numLeafs)) + 1) * numLeafs;
                m_nodes.reserve(numNodes);
            }
        }

        explicit AABBTree2(std::vector<Node> nodes) :
        m_nodes{std::move(nodes)} {
            for (size_t i = 0; i < m_nodes.size(); ++i) {
                std::visit(kdl::overload(
                    [](const auto&) {},
                    [&](const LeafNode& leafNode) {
                        m_leafForData[leafNode.data] = i;
                    }
                ), m_nodes[i]);
            }

            assert(checkInvariant());
        }

        /**
         * Indicates whether this tree is empty.
         *
         * @return true if this tree is empty and false otherwise
         */
        bool empty() const {
            return m_nodes.empty();
        }

        /**
         * Returns the bounds of all nodes in this tree.
         *
         * @return the bounds of all nodes in this tree, or a bounding box made up of NaN values if this tree is empty
         */
        const vm::bbox<T,S>& bounds() const {
            static constexpr auto EmptyBox = vm::bbox<T,S>{vm::vec<T,S>::nan(), vm::vec<T,S>::nan()};

            assert(!empty());
            if (empty()) {
                return EmptyBox;
            } else {
                return getNodeBounds(0);
            }
        }

        /**
         * Indicates whether a node with the given data exists in this tree.
         *
         * @param data the data to find
         * @return true if a node with the given data exists and false otherwise
         */
        bool contains(const U& data) const {
            return m_leafForData.find(data) != std::end(m_leafForData);
        }

        /**
         * Insert a node with the given bounds and data into this tree.
         *
         * @param bounds the bounds to insert
         * @param data the data to insert
         *
         * @throws NodeTreeException if a node with the given data already exists in this tree, or the bounds contains NaN
         */
        void insert(const vm::bbox<T,S>& bounds, U data) {
            assert(checkInvariant());

            checkBounds(bounds);

            // Check that the data isn't already inserted
            if (contains(data)) {
                throw NodeTreeException("Data already in tree");
            }

            if (empty()) {
                storeNode(LeafNode{bounds, std::nullopt, std::move(data)});
            } else {
                insert(0, bounds, std::move(data));
            }

            assert(checkInvariant());
        }

        bool remove(const U& data) {
            assert(checkInvariant());

            const auto it = m_leafForData.find(data);
            if (it == std::end(m_leafForData)) {
                return false;
            }
            
            const size_t index = it->second;
            assert(std::holds_alternative<LeafNode>(m_nodes[index]));
            
            if (index == 0) {
                clear();
            } else {
                auto parentIndex = getParentIndex(index);
                assert(parentIndex.has_value());
                assert(std::holds_alternative<InnerNode>(m_nodes[*parentIndex]));

                const auto grandParentIndex = getParentIndex(*parentIndex);

                const size_t siblingIndex = getSiblingIndex(index);
                moveNode(siblingIndex, *parentIndex);
                setParentIndex(*parentIndex, grandParentIndex);

                freeNode(index);
                m_leafForData.erase(it);

                bool boundsChanged = true;
                bool heightChanged = true;
                while (parentIndex && *parentIndex != 0 && (boundsChanged || heightChanged)) {
                    parentIndex = getParentIndex(*parentIndex);
                    boundsChanged = boundsChanged && updateBounds(*parentIndex);
                    heightChanged = heightChanged && updateHeight(*parentIndex);
                };
            }

            assert(checkInvariant());

            return true;
        }

        void update(const vm::bbox<T,S>& newBounds, const U& data) {
            checkBounds(newBounds);
            if (!remove(data)) {
                throw NodeTreeException("AABB node not found");
            }
            insert(newBounds, data);
        }

        void clear() {
            m_nodes.clear();
            m_leafForData.clear();
            m_freeNode.reset();
        }

        /**
         * Finds every data item in this tree whose bounding box intersects with the given ray and retuns a list of those items.
         *
         * @param ray the ray to test
         * @return a list containing all found data items
         */
        std::vector<U> findIntersectors(const vm::ray<T,S>& ray) const {
            auto result = std::vector<U>{};
            findIntersectors(ray, std::back_inserter(result));
            return result;
        }

        /**
         * Finds every data item in this tree whose bounding box intersects with the given ray and appends it to the given
         * output iterator.
         *
         * @tparam O the output iterator type
         * @param ray the ray to test
         * @param out the output iterator to append to
         */
        template <typename O>
        void findIntersectors(const vm::ray<T,S>& ray, O out) const {
            const auto intersects = [&](const auto& node) {
                return node.bounds.contains(ray.origin) || !vm::is_nan(vm::intersect_ray_bbox(ray, node.bounds));
            };

            visitNodes(kdl::overload(
                [&](const InnerNode& innerNode) {
                    return intersects(innerNode);
                },
                [&](const LeafNode& leafNode) {
                    if (intersects(leafNode)) {
                        out = leafNode.data;
                        ++out;
                    }
                    return false;
                }
            ));
        }

        /**
         * Finds every data item in this tree whose bounding box contains the given point and returns a list of those items.
         *
         * @param point the point to test
         * @return a list containing all found data items
         */
        std::vector<U> findContainers(const vm::vec<T,S>& point) const {
            auto result = std::vector<U>{};
            findContainers(point, result);
            return result;
        }

        /**
         * Finds every data item in this tree whose bounding box contains the given point and appends it to the given
         * output iterator.
         *
         * @tparam O the output iterator type
         * @param point the point to test
         * @param out the output iterator to append to
         */
        template <typename O>
        void findContainers(const vm::vec<T,S>& point, O out) const {
            visitNodes(kdl::overload(
                [&](const InnerNode& innerNode) {
                    return innerNode.bounds.contains(point);
                },
                [&](const LeafNode& leafNode) {
                    if (leafNode.bounds.contains(point)) {
                        out = leafNode.data;
                        ++out;
                    }
                    return false;
                }
            ));
        }

        friend bool operator==(const AABBTree2& lhs, const AABBTree2& rhs) {
            return (lhs.empty() && rhs.empty()) || compareSubtrees(lhs.m_nodes, rhs.m_nodes, 0, 0);
        }

        friend bool operator!=(const AABBTree2& lhs, const AABBTree2& rhs) {
            return !(lhs == rhs);
        }

        friend std::ostream& operator<<(std::ostream& str, const AABBTree2& tree) {
            if (!tree.empty()) {
                tree.appendToStream(str, 0, 0);
            }
            return str;
        }
    private:
        std::optional<size_t> getParentIndex(const size_t index) const {
            return std::visit(kdl::overload(
                [](const FreeNode&) -> std::optional<size_t> {
                    throw NodeTreeException{"Invalid node type"};
                },
                [](const InnerNode& innerNode) -> std::optional<size_t> {
                    return innerNode.parentIndex;
                },
                [](const LeafNode& leafNode) -> std::optional<size_t> {
                    return leafNode.parentIndex;
                }
            ), m_nodes[index]);
        }


        inline void setParentIndex(const size_t nodeIndex, const std::optional<size_t> parentIndex) {
            assert(nodeIndex < m_nodes.size());
            assert(!parentIndex || *parentIndex < m_nodes.size());

            std::visit(kdl::overload(
                [](const FreeNode&) {
                    throw NodeTreeException("Invalid node type");
                },
                [&](InnerNode& innerNode) {
                    innerNode.parentIndex = parentIndex;
                },
                [&](LeafNode& leafNode) {
                    leafNode.parentIndex = parentIndex;
                }
            ), m_nodes[nodeIndex]);
        }

        size_t getLeftChildIndex(const size_t index) const {
            return std::visit(kdl::overload(
                [](const FreeNode&) -> size_t {
                    throw NodeTreeException{"Invalid node type"};
                },
                [](const InnerNode& innerNode) -> size_t {
                    return innerNode.leftChildIndex;
                },
                [](const LeafNode&) -> size_t {
                    throw NodeTreeException{"Invalid node type"};
                }
            ), m_nodes[index]);
        }

        size_t getRightChildIndex(const size_t index) const {
            return std::visit(kdl::overload(
                [](const FreeNode&) -> size_t {
                    throw NodeTreeException{"Invalid node type"};
                },
                [](const InnerNode& innerNode) -> size_t {
                    return innerNode.rightChildIndex;
                },
                [](const LeafNode&) -> size_t {
                    throw NodeTreeException{"Invalid node type"};
                }
            ), m_nodes[index]);
        }

        size_t getSiblingIndex(const size_t index) const {
            const auto parentIndex = getParentIndex(index);
            assert(parentIndex.has_value());
            return std::visit(kdl::overload(
                [](const FreeNode&) -> size_t {
                    throw NodeTreeException{"Invalid node type"};
                },
                [&](const InnerNode& innerNode) -> size_t {
                    return index == innerNode.leftChildIndex ? innerNode.rightChildIndex : innerNode.leftChildIndex;
                },
                [](const LeafNode&) -> size_t {
                    throw NodeTreeException{"Invalid node type"};
                }
            ), m_nodes[*parentIndex]);
        }

        inline size_t getNodeHeight(const size_t index) const {
            return std::visit(kdl::overload(
                [](const FreeNode&) -> size_t {
                    throw NodeTreeException("Invalid node type");
                },
                [](const InnerNode& innerNode) -> size_t {
                    return innerNode.height;
                },
                [](const LeafNode&) -> size_t {
                    return 1u;
                }
            ), m_nodes[index]);
        }

        inline const vm::bbox<T,S>& getNodeBounds(const size_t index) const {
            return std::visit(kdl::overload(
                [](const FreeNode&) -> const vm::bbox<T,S>& {
                    throw NodeTreeException("Invalid node type");
                },
                [](const InnerNode& innerNode) -> const vm::bbox<T,S>& {
                    return innerNode.bounds;
                },
                [](const LeafNode& leafNode) -> const vm::bbox<T,S>& {
                    return leafNode.bounds;
                }
            ), m_nodes[index]);
        }

        size_t storeNode(Node&& node) {
            size_t index;
            if (m_freeNode) {
                index = *std::exchange(m_freeNode, std::get<FreeNode>(m_nodes[*m_freeNode]).next);
                m_nodes[index] = std::move(node);
            } else {
                index = m_nodes.size();
                m_nodes.push_back(std::move(node));
            }

            std::visit(kdl::overload(
                [](const FreeNode&) {},
                [](const InnerNode&) {},
                [&](const LeafNode& leafNode) {
                    m_leafForData[leafNode.data] = index;
                }
            ), m_nodes[index]);

            return index;
        }

        void moveNode(const size_t fromIndex, const size_t toIndex) {
            assert(fromIndex < m_nodes.size() && toIndex < m_nodes.size());
            assert(!std::holds_alternative<FreeNode>(m_nodes[fromIndex]));
            assert(!std::holds_alternative<FreeNode>(m_nodes[toIndex]));

            m_nodes[toIndex] = std::move(m_nodes[fromIndex]);
            std::visit(kdl::overload(
                [](const FreeNode&) {},
                [&](const InnerNode& innerNode) {
                    setParentIndex(innerNode.leftChildIndex, toIndex);
                    setParentIndex(innerNode.rightChildIndex, toIndex);
                },
                [&](const LeafNode& leafNode) {
                    m_leafForData[leafNode.data] = toIndex;
                }
            ), m_nodes[toIndex]);
            freeNode(fromIndex);
        }

        void freeNode(const size_t index) {
            m_nodes[index] = std::visit(kdl::overload(
                [](const FreeNode&) -> FreeNode {
                    throw NodeTreeException("Invalid node type");
                },
                [&](const InnerNode&) -> FreeNode {
                    return FreeNode{std::exchange(m_freeNode, index)};
                },
                [&](const LeafNode&) -> FreeNode {
                    return FreeNode{std::exchange(m_freeNode, index)};
                }
            ), m_nodes[index]);
        }

        bool updateBounds(const size_t index) {
            const auto leftChildIndex = getLeftChildIndex(index);
            const auto rightChildIndex = getRightChildIndex(index);

            auto& innerNode = std::get<InnerNode>(m_nodes[index]);
            const auto oldBounds = std::exchange(innerNode.bounds, vm::merge(getNodeBounds(leftChildIndex), getNodeBounds(rightChildIndex)));
            return innerNode.bounds != oldBounds;
        }

        bool updateHeight(const size_t index) {
            const auto leftChildIndex = getLeftChildIndex(index);
            const auto rightChildIndex = getRightChildIndex(index);

            auto& innerNode = std::get<InnerNode>(m_nodes[index]);
            const size_t oldHeight = std::exchange(innerNode.height, std::max(getNodeHeight(leftChildIndex), getNodeHeight(rightChildIndex)) + 1);
            return innerNode.height != oldHeight;
        }

        std::tuple<bool, bool> insert(const size_t nodeIndex, const vm::bbox<T,S>& bounds, U data) {
            return std::visit(kdl::overload(
                [](const FreeNode&) -> std::tuple<bool, bool> {
                    throw NodeTreeException("Invalid node type");
                },
                [&](const InnerNode&) -> std::tuple<bool, bool> {
                    const auto subtreeIndex = selectSubtreeForInsertion(getLeftChildIndex(nodeIndex), getRightChildIndex(nodeIndex), bounds);
                    auto [boundsChanged, heightChanged] = insert(subtreeIndex, bounds, std::move(data));
                    boundsChanged = boundsChanged && updateBounds(nodeIndex);
                    heightChanged = heightChanged && updateHeight(nodeIndex);
                    return {boundsChanged, heightChanged};
                },
                [&](LeafNode& leafNode) -> std::tuple<bool, bool> {
                    m_nodes[nodeIndex] = InnerNode{
                        vm::merge(leafNode.bounds, bounds),
                        leafNode.parentIndex,
                        storeNode(LeafNode{leafNode.bounds, nodeIndex, std::move(leafNode.data)}),
                        storeNode(LeafNode{bounds, nodeIndex, std::move(data)}),
                        2
                    };
                    return {true, true};
                }
            ), m_nodes[nodeIndex]);
        }

        size_t selectSubtreeForInsertion(const size_t node1Index, const size_t node2Index, const vm::bbox<T,S>& bounds) const {
            const auto& node1Bounds = getNodeBounds(node1Index);
            const auto& node2Bounds = getNodeBounds(node2Index);
            const bool node1Contains = node1Bounds.contains(bounds);
            const bool node2Contains = node2Bounds.contains(bounds);

            if (node1Contains && !node2Contains) {
                return node1Index;
            }
            
            if (!node1Contains && node2Contains) {
                return node2Index;
            }
            
            if (!node1Contains && !node2Contains) {
                const auto diff1 = vm::merge(node1Bounds, bounds).volume() - node1Bounds.volume();
                const auto diff2 = vm::merge(node2Bounds, bounds).volume() - node2Bounds.volume();

                if (diff1 < diff2) {
                    return node1Index;
                }
                if (diff2 < diff1) {
                    return node2Index;
                }
            }

            // both nodes volume is increased by the same amount
            static auto choice = 0u;
            const auto node1Height = getNodeHeight(node1Index);
            const auto node2Height = getNodeHeight(node2Index);
            
            if (node1Height < node2Height) {
                return node1Index;
            }

            if (node2Height < node1Height) {
                return node2Index;
            }

            if (choice++ % 2 == 0) {
                return node1Index;
            }

            return node2Index;
        }

        template <typename V>
        void visitNode(V&& visitor, const size_t index) const {
            assert(index < m_nodes.size());
            std::visit(kdl::overload(
                [](const FreeNode&) {
                    throw NodeTreeException("Invalid node type");
                },
                [&](const InnerNode& innerNode) {
                    if (visitor(innerNode)) {
                        visitNode(std::forward<V>(visitor), getLeftChildIndex(index));
                        visitNode(std::forward<V>(visitor), getRightChildIndex(index));
                    }
                },
                [&](const LeafNode& leafNode) {
                    visitor(leafNode);
                }
            ), m_nodes[index]);
        }

        template <typename V>
        void visitNodes(V&& visitor) const {
            if (!empty()) {
                visitNode(std::forward<V>(visitor), 0);
            }
        }

        static bool compareSubtrees(const std::vector<Node>& lhsNodes, const std::vector<Node>& rhsNodes, const size_t lhsIndex, const size_t rhsIndex) {
            if (lhsIndex >= lhsNodes.size() || rhsIndex >= rhsNodes.size()) {
                return false;
            }

            return std::visit(kdl::overload(
                [](const FreeNode&, const FreeNode&) { return true; },
                [&](const InnerNode& lhsNode, const InnerNode& rhsNode) {
                    return lhsNode == rhsNode
                        && compareSubtrees(lhsNodes, rhsNodes, lhsNode.leftChildIndex, rhsNode.leftChildIndex)
                        && compareSubtrees(lhsNodes, rhsNodes, lhsNode.rightChildIndex, rhsNode.rightChildIndex);
                },
                [](const LeafNode& lhsNode, const LeafNode& rhsNode) {
                    return lhsNode == rhsNode;
                },
                [](const auto&, const auto&) { return false; }
            ), lhsNodes[lhsIndex], rhsNodes[rhsIndex]);
        }

        void appendToStream(std::ostream& str, const size_t nodeIndex, const size_t nodeDepth) const {
            for (size_t i = 0; i < nodeDepth; ++i) {
                str << "  ";
            }
            str << m_nodes[nodeIndex] << std::endl;

            std::visit(kdl::overload(
                [](const FreeNode&) {},
                [&](const InnerNode&) {
                    appendToStream(str, getLeftChildIndex(nodeIndex), nodeDepth + 1);
                    appendToStream(str, getRightChildIndex(nodeIndex), nodeDepth + 1);
                },
                [&](const LeafNode&) {}
            ), m_nodes[nodeIndex]);
        }

        static void checkBounds(const vm::bbox<T,S>& bounds) {
            if (vm::is_nan(bounds.min) || vm::is_nan(bounds.max)) {
                throw NodeTreeException("Cannot add node to AABB tree with invalid bounds");
            }
        }

        bool checkInvariant() const {
            /* 
            if (!checkNodes()) {
                return false;
            }
            if (!checkLeafForData()) {
                return false;
            }
            */

            return true;
        }

        void markReachableNodesInSubtree(const size_t index, std::vector<bool>& reachableNodes) const {
            assert(index < m_nodes.size());
            reachableNodes[index] = true;

            std::visit(kdl::overload(
                [](const FreeNode&) {},
                [&](const InnerNode&) { 
                    markReachableNodesInSubtree(getLeftChildIndex(index), reachableNodes);
                    markReachableNodesInSubtree(getRightChildIndex(index), reachableNodes);
                },
                [](const LeafNode&) {}
            ), m_nodes[index]);
        }

        std::vector<bool> markReachableNodes() const {
            auto reachableNodes = std::vector<bool>(m_nodes.size(), false);
            if (!empty()) {
                markReachableNodesInSubtree(0, reachableNodes);
            }
            return reachableNodes;
        }

        bool checkNodes() const {
            const auto reachableNodes = markReachableNodes();
            assert(reachableNodes.size() == m_nodes.size());

            for (size_t i = 0; i < m_nodes.size(); ++i) {
                if (reachableNodes[i] == std::holds_alternative<FreeNode>(m_nodes[i])) {
                    return false;
                }
            }

            return true;
        }

        bool checkLeafForData() const {
            if (m_leafForData.empty()) {
                return m_nodes.empty() || std::holds_alternative<FreeNode>(m_nodes.front());
            }

            for (const auto& [data, index] : m_leafForData) {
                if (index >= m_nodes.size()
                    || !std::holds_alternative<LeafNode>(m_nodes[index])
                    || std::get<LeafNode>(m_nodes[index]).data != data) {
                    return false;
                }
            }

            for (size_t i = 0; i < m_nodes.size(); ++i) {
                const auto valid = std::visit(kdl::overload(
                    [](const FreeNode&) { return true; },
                    [](const InnerNode&) { return true; },
                    [&](const LeafNode& leafNode) {
                        const auto it = m_leafForData.find(leafNode.data);
                        return it != std::end(m_leafForData) && it->second == i;
                    }
                ), m_nodes[i]);
                if (!valid) {
                    return false;
                }
            }

            return true;
        }
    };
}

