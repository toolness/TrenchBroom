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

#include "Expression.h"

#include "Ensure.h"
#include "Macros.h"
#include "EL/EvaluationContext.h"
#include "EL/Expressions.h"

#include <kdl/overload.h>

#include <sstream>

namespace TrenchBroom {
    namespace EL {
        Expression::Expression(LiteralExpression expression, const size_t line, const size_t column) :
        m_expression{std::make_unique<ExpressionVariant>(std::move(expression))},
        m_line{line},
        m_column{column} {}
        
        Expression::Expression(VariableExpression expression, const size_t line, const size_t column) :
        m_expression{std::make_unique<ExpressionVariant>(std::move(expression))},
        m_line{line},
        m_column{column} {}
        
        Expression::Expression(ArrayExpression expression, const size_t line, const size_t column) :
        m_expression{std::make_unique<ExpressionVariant>(std::move(expression))},
        m_line{line},
        m_column{column} {}
        
        Expression::Expression(MapExpression expression, const size_t line, const size_t column) :
        m_expression{std::make_unique<ExpressionVariant>(std::move(expression))},
        m_line{line},
        m_column{column} {}
        
        Expression::Expression(UnaryExpression expression, const size_t line, const size_t column) :
        m_expression{std::make_unique<ExpressionVariant>(std::move(expression))},
        m_line{line},
        m_column{column} {}
        
        Expression::Expression(BinaryExpression expression, const size_t line, const size_t column) :
        m_expression{std::make_unique<ExpressionVariant>(std::move(expression))},
        m_line{line},
        m_column{column} {
            rebalanceByPrecedence();
        }
        
        Expression::Expression(SubscriptExpression expression, const size_t line, const size_t column) :
        m_expression{std::make_unique<ExpressionVariant>(std::move(expression))},
        m_line{line},
        m_column{column} {}
        
        Expression::Expression(SwitchExpression expression, const size_t line, const size_t column) :
        m_expression{std::make_unique<ExpressionVariant>(std::move(expression))},
        m_line{line},
        m_column{column} {}
        
        Value Expression::evaluate(const EvaluationContext& context) const {
            return std::visit([&](const auto& e) { return e.evaluate(context); }, *m_expression);
        }

        Expression Expression::optimize() const {
            auto optimizedExpression = std::visit(kdl::overload(
                [](const LiteralExpression& e) -> ExpressionVariant {
                    return e;
                },
                [](const VariableExpression& e) -> ExpressionVariant {
                    return e;
                },
                [](const auto& e) { 
                    return e.optimize();
                }
            ), *m_expression);

            return std::visit([&](auto&& e) { return Expression{std::move(e), m_line, m_column}; }, std::move(optimizedExpression));
       }

        size_t Expression::line() const {
            return m_line;
        }
        
        size_t Expression::column() const {
            return m_column;
        }

        std::string Expression::asString() const {
            auto str = std::stringstream{};
            str << *this;
            return str.str();
        }

        bool operator==(const Expression& lhs, const Expression& rhs) {
            return std::visit(kdl::overload(
                [](const LiteralExpression& l, const LiteralExpression& r)     { return l == r; },
                [](const VariableExpression& l, const VariableExpression& r)   { return l == r; },
                [](const ArrayExpression& l, const ArrayExpression& r)         { return l == r; },
                [](const MapExpression& l, const MapExpression& r)             { return l == r; },
                [](const UnaryExpression& l, const UnaryExpression& r)         { return l == r; },
                [](const BinaryExpression& l, const BinaryExpression& r)       { return l == r; },
                [](const SubscriptExpression& l, const SubscriptExpression& r) { return l == r; },
                [](const SwitchExpression& l, const SwitchExpression& r)       { return l == r; },
                [](const auto&, const auto&)                                   { return false; }
            ), *lhs.m_expression, *rhs.m_expression);
        }

        bool operator!=(const Expression& lhs, const Expression& rhs) {
            return !(lhs == rhs);
        }

        std::ostream& operator<<(std::ostream& str, const Expression& exp) {
            std::visit([&](const auto& e) { str << e; }, *exp.m_expression);
            return str;
        }
        
        void Expression::rebalanceByPrecedence() {
            /*
             * The expression tree has a similar invariant to a heap: For any given node, its precedence must be less
             * than or equal to the precedences of its children. This guarantees that the tree evaluating the tree in a
             * depth first traversal yields correct results because the nodes with the highest precedence are evaluated
             * before the nodes with lower precedence.
             */
        
            assert(std::holds_alternative<BinaryExpression>(*m_expression));
        
            const auto parentPrecedence = std::get<BinaryExpression>(*m_expression).precedence();
            const auto leftPrecedence = std::get<BinaryExpression>(*m_expression).m_leftOperand.precedence();
            const auto rightPrecedence = std::get<BinaryExpression>(*m_expression).m_rightOperand.precedence();
            
            if (parentPrecedence > std::min(leftPrecedence, rightPrecedence)) {
                if (leftPrecedence < rightPrecedence) {
                    // push this operator into the right subtree, rotating the right node up, and rebalancing the right subtree again
                    Expression leftExpression = std::move(std::get<BinaryExpression>(*m_expression).m_leftOperand);
                    
                    assert(std::holds_alternative<BinaryExpression>(*leftExpression.m_expression));
                    std::get<BinaryExpression>(*m_expression).m_leftOperand = std::move(std::get<BinaryExpression>(*leftExpression.m_expression).m_rightOperand);
                    std::get<BinaryExpression>(*leftExpression.m_expression).m_rightOperand = std::move(*this);
                    *this = std::move(leftExpression);
                    
                    std::get<BinaryExpression>(*m_expression).m_rightOperand.rebalanceByPrecedence();
                } else {
                    // push this operator into the left subtree, rotating the left node up, and rebalancing the left subtree again
                    Expression rightExpression = std::move(std::get<BinaryExpression>(*m_expression).m_rightOperand);
                    
                    assert(std::holds_alternative<BinaryExpression>(*rightExpression.m_expression));
                    std::get<BinaryExpression>(*m_expression).m_rightOperand = std::move(std::get<BinaryExpression>(*rightExpression.m_expression).m_leftOperand);
                    std::get<BinaryExpression>(*rightExpression.m_expression).m_leftOperand = std::move(*this);
                    *this = std::move(rightExpression);
                    
                    std::get<BinaryExpression>(*m_expression).m_leftOperand.rebalanceByPrecedence();
                }
            }
        }
        
        size_t Expression::precedence() const {
            return std::visit(kdl::overload(
                [](const BinaryExpression& exp) -> size_t {
                    return exp.precedence();
                },
                [](const auto&) -> size_t {
                    return 13u;
                }
            ), *m_expression);
        }
    }
}
