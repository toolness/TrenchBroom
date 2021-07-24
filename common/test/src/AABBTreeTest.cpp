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

#include "AABBTree2.h"

#include <vecmath/vec.h>
#include <vecmath/ray.h>

#include <kdl/overload.h>
#include <kdl/string_utils.h>

#include <set>
#include <sstream>

#include "Catch2.h"

namespace TrenchBroom {
    using AABB = AABBTree2<double, 3, int>;
    
    // Helpers for creating trees with a known structure to assert against.

    template <typename U> struct InnerNode;
    template <typename U> struct LeafNode;

    template <typename U>
    using Node = std::variant<InnerNode<U>, LeafNode<U>>;

    template <typename U>
    struct LeafNode {
        vm::bbox3d bounds;
        U data;

        LeafNode(const vm::bbox3d& i_bounds, U i_data) :
        bounds{i_bounds},
        data{std::move(i_data)} {}
    };

    template <typename U>
    LeafNode(const vm::bbox3d&, U) -> LeafNode<U>;

    template <typename U>
    struct InnerNode {
        std::unique_ptr<Node<U>> left;
        std::unique_ptr<Node<U>> right;

        InnerNode(Node<U> i_left, Node<U> i_right) :
        left{std::make_unique<Node<U>>(std::move(i_left))},
        right{std::make_unique<Node<U>>(std::move(i_right))} {}
    };

    template <template <typename> typename N1, template <typename> typename N2, typename U>
    InnerNode(N1<U>, N2<U>) -> InnerNode<U>;

    template <typename U>
    std::tuple<size_t, vm::bbox3d, size_t> makeNode(Node<U>&& currentNode, const std::optional<size_t> parentIndex, std::vector<typename AABBTree2<double, 3, U>::Node>& nodes) {
        return std::visit(kdl::overload(
            [&](InnerNode<U>&& innerNode) -> std::tuple<size_t, vm::bbox3d, size_t> {
                // insert a placeholder free node
                const auto nodeIndex = nodes.size();
                nodes.emplace_back(typename AABBTree2<double, 3, U>::FreeNode{std::nullopt});
                const auto [leftIndex, leftBounds, leftHeight] = makeNode(std::move(*innerNode.left), nodeIndex, nodes);
                const auto [rightIndex, rightBounds, rightHeight] = makeNode(std::move(*innerNode.right), nodeIndex, nodes);
                
                const auto bounds = vm::merge(leftBounds, rightBounds);
                const auto height = std::max(leftHeight, rightHeight) + 1;

                nodes[nodeIndex] = typename AABBTree2<double, 3, U>::InnerNode{
                    bounds,
                    parentIndex,
                    leftIndex,
                    rightIndex,
                    height
                };

                return std::make_tuple(nodeIndex, bounds, height);
            },
            [&](LeafNode<U>&& leafNode) -> std::tuple<size_t, vm::bbox3d, size_t> {
                const auto nodeIndex = nodes.size();
                nodes.emplace_back(typename AABBTree2<double, 3, U>::LeafNode{leafNode.bounds, parentIndex, std::move(leafNode.data)});
                return std::make_tuple(nodeIndex, leafNode.bounds, 1);
            }
        ), std::move(currentNode));
    }

    template <template <typename> typename N, typename U>
    std::vector<typename AABBTree2<double, 3, U>::Node> makeNodes(N<U> node) {
        auto nodes = std::vector<typename AABBTree2<double, 3, U>::Node>{};
        makeNode<U>(Node<U>{std::move(node)}, std::nullopt, nodes);
        return nodes;
    }

    template <template <typename> typename N, typename U>
    AABBTree2<double, 3, U> makeTree(N<U> node) {
        return AABBTree2<double, 3, U>{makeNodes(std::move(node))};
    }

    TEST_CASE("makeNodes") {
        CHECK(makeNodes(LeafNode{{{0, 0, 0}, {1, 2, 3}}, 1}) == std::vector<AABB::Node>{
            AABB::LeafNode{{{0, 0, 0}, {1, 2, 3}}, std::nullopt, 1}
        });

        CHECK(makeNodes(
            InnerNode{
                LeafNode{{{0, 0, 0}, {1, 2, 3}}, 1},
                LeafNode{{{1, 1, 1}, {3, 2, 1}}, 2}
            }) == std::vector<AABB::Node>{
                AABB::InnerNode{{{0, 0, 0}, {3, 2, 3}}, std::nullopt, 1, 2, 2},
                AABB::LeafNode{{{0, 0, 0}, {1, 2, 3}}, 0, 1},
                AABB::LeafNode{{{1, 1, 1}, {3, 2, 1}}, 0, 2},
        });

        CHECK(makeNodes(
            InnerNode{
                LeafNode{{{ 0,  0,  0}, {2, 1, 1}}, 1},
                InnerNode{
                    LeafNode{{{-1, -1, -1}, {1, 1, 1}}, 2},
                    LeafNode{{{-2, -2, -1}, {0, 0, 1}}, 3}
                }
            }) == std::vector<AABB::Node>{
                AABB::InnerNode{{{-2, -2, -1}, {2, 1, 1}}, std::nullopt, 1, 2, 3},
                AABB::LeafNode{{{0, 0, 0}, {2, 1, 1}}, 0, 1},
                AABB::InnerNode{{{-2, -2, -1}, {1, 1, 1}}, 0, 3, 4, 2},
                AABB::LeafNode{{{-1, -1, -1}, {1, 1, 1}}, 2, 2},
                AABB::LeafNode{{{-2, -2, -1}, {0, 0, 1}}, 2, 3},
        });
    }

    TEST_CASE("AABBTree.constructor") {
        auto tree = AABB{};
        CHECK(tree.empty());
    }

    TEST_CASE("AABBTree.insertSingleNode") {
        const auto bounds = vm::bbox3d{{0, 0, 0}, {2, 1, 1}};

        auto tree = AABB{};
        tree.insert(bounds, 1);

        CHECK(tree == makeTree(
            LeafNode{bounds, 1}
        ));
    }

    TEST_CASE("AABBTree.insertDuplicateNode") {
        const auto bounds = vm::bbox3d{{0, 0, 0}, {2, 1, 1}};

        auto tree = AABB{};
        tree.insert(bounds, 1);

        REQUIRE(tree == makeTree(
            LeafNode{bounds, 1}
        ));

        CHECK_THROWS_AS(tree.insert(bounds, 1), NodeTreeException);

        CHECK(tree == makeTree(
            LeafNode{bounds, 1}
        ));
    }

    TEST_CASE("AABBTree.insertTwoNodes") {
        const auto bounds1 = vm::bbox3d{{ 0,  0,  0}, {2, 1, 1}};
        const auto bounds2 = vm::bbox3d{{-1, -1, -1}, {1, 1, 1}};

        auto tree = AABB{};
        tree.insert(bounds1, 1);
        tree.insert(bounds2, 2);

        CHECK(tree == makeTree(
            InnerNode{
                LeafNode{bounds1, 1},
                LeafNode{bounds2, 2}
            }
        ));
    }

    TEST_CASE("AABBTree.insertThreeNodes") {
        const auto bounds1 = vm::bbox3d{{ 0,  0,  0}, {2, 1, 1}};
        const auto bounds2 = vm::bbox3d{{-1, -1, -1}, {1, 1, 1}};
        const auto bounds3 = vm::bbox3d{{-2, -2, -1}, {0, 0, 1}};

        auto tree = AABB{};
        tree.insert(bounds1, 1);
        tree.insert(bounds2, 2);
        tree.insert(bounds3, 3);

        CHECK(tree == makeTree(
            InnerNode{
                LeafNode{bounds1, 1},
                InnerNode{
                    LeafNode{bounds2, 2},
                    LeafNode{bounds3, 3}
                }
            }
        ));
    }

    TEST_CASE("AABBTree.insertFourContainedNodes") {
        const auto bounds1 = vm::bbox3d{{-4, -4, -4}, {4, 4, 4}};
        const auto bounds2 = vm::bbox3d{{-3, -3, -3}, {3, 3, 3}};
        const auto bounds3 = vm::bbox3d{{-2, -2, -2}, {2, 2, 2}};
        const auto bounds4 = vm::bbox3d{{-1, -1, -1}, {1, 1, 1}};

        auto tree = AABB{};
        tree.insert(bounds1, 1);
        tree.insert(bounds2, 2);

        CHECK(tree == makeTree(
            InnerNode{
                LeafNode{bounds1, 1},
                LeafNode{bounds2, 2}
            }
        ));

        tree.insert(bounds3, 3);

        CHECK(tree == makeTree(
            InnerNode{
                InnerNode{
                    LeafNode{bounds1, 1},
                    LeafNode{bounds3, 3}
                },
                LeafNode{bounds2, 2}
            }
        ));

        tree.insert(bounds4, 4);

        CHECK(tree == makeTree(
            InnerNode{
                InnerNode{
                    LeafNode{bounds1, 1},
                    LeafNode{bounds3, 3}
                },
                InnerNode{
                    LeafNode{bounds2, 2},
                    LeafNode{bounds4, 4}
                }
            }
        ));
    }

    TEST_CASE("AABBTree.insertFourContainedNodesinverse") {
        const auto bounds1 = vm::bbox3d{{-1, -1, -1}, {1, 1, 1}};
        const auto bounds2 = vm::bbox3d{{-2, -2, -2}, {2, 2, 2}};
        const auto bounds3 = vm::bbox3d{{-3, -3, -3}, {3, 3, 3}};
        const auto bounds4 = vm::bbox3d{{-4, -4, -4}, {4, 4, 4}};

        auto tree = AABB{};
        tree.insert(bounds1, 1);
        tree.insert(bounds2, 2);

        CHECK(tree == makeTree(
            InnerNode{
                LeafNode{bounds1, 1},
                LeafNode{bounds2, 2}
            }
        ));

        tree.insert(bounds3, 3);

        CHECK(tree == makeTree(
            InnerNode{
                LeafNode{bounds1, 1},
                InnerNode{
                    LeafNode{bounds2, 2},
                    LeafNode{bounds3, 3}
                }
            }
        ));

        tree.insert(bounds4, 4);

        CHECK(tree == makeTree(
            InnerNode{
                LeafNode{bounds1, 1},
                InnerNode{
                    LeafNode{bounds2, 2},
                    InnerNode{
                        LeafNode{bounds3, 3},
                        LeafNode{bounds4, 4}
                    }
                }
            }
        ));
    }

    TEST_CASE("AABBTree.removeThreeLeafs") {
        const auto bounds1 = vm::bbox3d{{ 0,  0,  0}, {2, 1, 1}};
        const auto bounds2 = vm::bbox3d{{-1, -1, -1}, {1, 1, 1}};
        const auto bounds3 = vm::bbox3d{{-2, -2, -1}, {0, 0, 1}};

        auto tree = AABB{};
        tree.insert(bounds1, 1);
        tree.insert(bounds2, 2);
        tree.insert(bounds3, 3);

        REQUIRE(tree == makeTree(
            InnerNode{
                LeafNode{bounds1, 1},
                InnerNode{
                    LeafNode{bounds2, 2},
                    LeafNode{bounds3, 3}
                }
            }
        ));

        SECTION("In order of insertion") {
            CHECK(tree.remove(1));
            CHECK(tree == makeTree(
                InnerNode{
                    LeafNode{bounds2, 2},
                    LeafNode{bounds3, 3}
                }
            ));

            REQUIRE_FALSE(tree.remove(1));
            CHECK(tree.remove(2));
            CHECK(tree == makeTree(
                LeafNode{bounds3, 3}
            ));
            
            REQUIRE_FALSE(tree.remove(2));
            CHECK(tree.remove(3));
            CHECK(tree.empty());
        }

        SECTION("In inverse order of insertion") {
            CHECK(tree.remove(3));
            CHECK(tree == makeTree(
                InnerNode{
                    LeafNode{bounds1, 1},
                    LeafNode{bounds2, 2}
                }
            ));

            REQUIRE_FALSE(tree.remove(3));
            CHECK(tree.remove(2));
            CHECK(tree == makeTree(
                LeafNode{bounds1, 1}
            ));
            
            REQUIRE_FALSE(tree.remove(2));
            CHECK(tree.remove(1));
            CHECK(tree.empty());
        }
    }

    TEST_CASE("AABBTree.removeFourContainedNodes") {
        const auto bounds1 = vm::bbox3d{{-1, -1, -1}, {1, 1, 1}};
        const auto bounds2 = vm::bbox3d{{-2, -2, -2}, {2, 2, 2}};
        const auto bounds3 = vm::bbox3d{{-3, -3, -3}, {3, 3, 3}};
        const auto bounds4 = vm::bbox3d{{-4, -4, -4}, {4, 4, 4}};

        auto tree = AABB{};
        tree.insert(bounds1, 1);
        tree.insert(bounds2, 2);
        tree.insert(bounds3, 3);
        tree.insert(bounds4, 4);

        REQUIRE(tree == makeTree(
            InnerNode{
                LeafNode{bounds1, 1},
                InnerNode{
                    LeafNode{bounds2, 2},
                    InnerNode{
                        LeafNode{bounds3, 3},
                        LeafNode{bounds4, 4}
                    }
                }
            }
        ));

        CHECK(tree.remove(4));
        CHECK(tree == makeTree(
            InnerNode{
                LeafNode{bounds1, 1},
                InnerNode{
                    LeafNode{bounds2, 2},
                    LeafNode{bounds3, 3}
                }
            }
        ));

        CHECK(tree.remove(3));
        CHECK(tree == makeTree(
            InnerNode{
                LeafNode{bounds1, 1},
                LeafNode{bounds2, 2}
            }
        ));

        CHECK(tree.remove(2));
        CHECK(tree == makeTree(
            LeafNode{bounds1, 1}
        ));

        CHECK(tree.remove(1));
        CHECK(tree.empty());
    }

    /*
    template <typename K>
    BOX makeBounds(const K min, const K max) {
        return BOX(VEC(static_cast<double>(min), -1.0, -1.0), VEC(static_cast<double>(max), 1.0, 1.0));
    }

    TEST_CASE("AABBTree.findIntersectorsOfEmptyTree", "[AABBTree]") {
        AABB tree;
        assertIntersectors(tree, RAY(VEC::zero(), VEC::pos_x()), {});
    }

    TEST_CASE("AABBTree.findIntersectorsOfTreeWithOneNode", "[AABBTree]") {
        AABB tree;
        tree.insert(BOX(VEC(-1.0, -1.0, -1.0), VEC(1.0, 1.0, 1.0)), 1u);

        assertIntersectors(tree, RAY(VEC(-2.0, 0.0, 0.0), VEC::neg_x()), {});
        assertIntersectors(tree, RAY(VEC(-2.0, 0.0, 0.0), VEC::pos_x()), { 1u });
    }

    TEST_CASE("AABBTree.findIntersectorsOfTreeWithTwoNodes", "[AABBTree]") {
        AABB tree;
        tree.insert(BOX(VEC(-2.0, -1.0, -1.0), VEC(-1.0, +1.0, +1.0)), 1u);
        tree.insert(BOX(VEC(+1.0, -1.0, -1.0), VEC(+2.0, +1.0, +1.0)), 2u);

        assertIntersectors(tree, RAY(VEC(+3.0,  0.0,  0.0), VEC::pos_x()), {});
        assertIntersectors(tree, RAY(VEC(-3.0,  0.0,  0.0), VEC::neg_x()), {});
        assertIntersectors(tree, RAY(VEC( 0.0,  0.0,  0.0), VEC::pos_z()), {});
        assertIntersectors(tree, RAY(VEC( 0.0,  0.0,  0.0), VEC::pos_x()), { 2u });
        assertIntersectors(tree, RAY(VEC( 0.0,  0.0,  0.0), VEC::neg_x()), { 1u });
        assertIntersectors(tree, RAY(VEC(-3.0,  0.0,  0.0), VEC::pos_x()), { 1u, 2u });
        assertIntersectors(tree, RAY(VEC(+3.0,  0.0,  0.0), VEC::neg_x()), { 1u, 2u });
        assertIntersectors(tree, RAY(VEC(-1.5, -2.0,  0.0), VEC::pos_y()), { 1u });
        assertIntersectors(tree, RAY(VEC(+1.5, -2.0,  0.0), VEC::pos_y()), { 2u });
    }

    TEST_CASE("AABBTree.findIntersectorFromInside", "[AABBTree]") {
        AABB tree;
        tree.insert(BOX(VEC(-4.0, -1.0, -1.0), VEC(+4.0, +1.0, +1.0)), 1u);

        assertIntersectors(tree, RAY(VEC(0.0,  0.0,  0.0), VEC::pos_x()), { 1u });
    }

    TEST_CASE("AABBTree.findIntersectorsFromInsideRootBBox", "[AABBTree]") {
        AABB tree;
        tree.insert(BOX(VEC(-4.0, -1.0, -1.0), VEC(-2.0, +1.0, +1.0)), 1u);
        tree.insert(BOX(VEC(+2.0, -1.0, -1.0), VEC(+4.0, +1.0, +1.0)), 2u);

        assertIntersectors(tree, RAY(VEC(0.0,  0.0,  0.0), VEC::pos_x()), { 2u });
    }

    TEST_CASE("AABBTree.clear", "[AABBTree]") {
        const BOX bounds1(VEC(0.0, 0.0, 0.0), VEC(2.0, 1.0, 1.0));
        const BOX bounds2(VEC(-1.0, -1.0, -1.0), VEC(1.0, 1.0, 1.0));

        AABB tree;
        tree.insert(bounds1, 1u);
        tree.insert(bounds2, 2u);

        REQUIRE(tree.contains(1u));
        REQUIRE(tree.contains(2u));
        REQUIRE_THAT(tree.findContainers(vm::vec3d{0.5, 0.5, 0.5}), Catch::UnorderedEquals(std::vector<size_t>{
            1u, 2u
        }));

        tree.clear();

        CHECK(tree.empty());
        CHECK_FALSE(tree.contains(1u));
        CHECK_FALSE(tree.contains(2u));
        REQUIRE_THAT(tree.findContainers(vm::vec3d{0.5, 0.5, 0.5}), Catch::UnorderedEquals(std::vector<size_t>{}));
    }
    */
}
