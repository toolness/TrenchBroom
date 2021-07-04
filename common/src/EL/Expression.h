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

#include "Macros.h"
#include "EL/EL_Forward.h"
#include "EL/Value.h"

#include <iosfwd>
#include <map>
#include <memory>
#include <string>
#include <variant>
#include <vector>

namespace TrenchBroom {
    namespace EL {
        class LiteralExpression;
        class VariableExpression;
        
        class ArrayExpression;
        class MapExpression;
        
        class UnaryExpression;
        class BinaryExpression;
        class SubscriptExpression;
        class SwitchExpression;

        using ExpressionVariant = std::variant<
            LiteralExpression, VariableExpression,
            ArrayExpression, MapExpression,
            UnaryExpression, BinaryExpression, SubscriptExpression, SwitchExpression>;

        class Expression {
        private:
            std::shared_ptr<ExpressionVariant> m_expression;
            size_t m_line;
            size_t m_column;
        public:
            Expression(LiteralExpression expression, size_t line, size_t column);
            Expression(VariableExpression expression, size_t line, size_t column);
            Expression(ArrayExpression expression, size_t line, size_t column);
            Expression(MapExpression expression, size_t line, size_t column);
            Expression(UnaryExpression expression, size_t line, size_t column);
            Expression(BinaryExpression expression, size_t line, size_t column);
            Expression(SubscriptExpression expression, size_t line, size_t column);
            Expression(SwitchExpression expression, size_t line, size_t column);
            
            Value evaluate(const EvaluationContext& context) const;
            Expression optimize() const;

            size_t line() const;
            size_t column() const;

            std::string asString() const;

            friend bool operator==(const Expression& lhs, const Expression& rhs);
            friend bool operator!=(const Expression& lhs, const Expression& rhs);
            friend std::ostream& operator<<(std::ostream& str, const Expression& exp);
        private:
            void rebalanceByPrecedence();
            size_t precedence() const;
        };
        
        class LiteralExpression {
        private:
            Value m_value;
        public:
            LiteralExpression(Value value);
            
            const Value& evaluate(const EvaluationContext& context) const;

            friend bool operator==(const LiteralExpression& lhs, const LiteralExpression& rhs);
            friend bool operator!=(const LiteralExpression& lhs, const LiteralExpression& rhs);
            friend std::ostream& operator<<(std::ostream& str, const LiteralExpression& exp);
        };
        
        class VariableExpression {
        private:
            std::string m_variableName;
        public:
            VariableExpression(std::string variableName);
            
            Value evaluate(const EvaluationContext& context) const;

            friend bool operator==(const VariableExpression& lhs, const VariableExpression& rhs);
            friend bool operator!=(const VariableExpression& lhs, const VariableExpression& rhs);
            friend std::ostream& operator<<(std::ostream& str, const VariableExpression& exp);
        };
        
        class ArrayExpression {
        private:
            std::vector<Expression> m_elements;
        public:
            ArrayExpression(std::vector<Expression> elements);
            
            Value evaluate(const EvaluationContext& context) const;
            ExpressionVariant optimize() const;

            friend bool operator==(const ArrayExpression& lhs, const ArrayExpression& rhs);
            friend bool operator!=(const ArrayExpression& lhs, const ArrayExpression& rhs);
            friend std::ostream& operator<<(std::ostream& str, const ArrayExpression& exp);
        };
        
        class MapExpression {
        private:
            std::map<std::string, Expression> m_elements;
        public:
            MapExpression(std::map<std::string, Expression> elements);

            Value evaluate(const EvaluationContext& context) const;
            ExpressionVariant optimize() const;

            friend bool operator==(const MapExpression& lhs, const MapExpression& rhs);
            friend bool operator!=(const MapExpression& lhs, const MapExpression& rhs);
            friend std::ostream& operator<<(std::ostream& str, const MapExpression& exp);
        };
        
        enum class UnaryOperator {
            Plus,
            Minus,
            LogicalNegation,
            BitwiseNegation,
            Group
        };
        
        class UnaryExpression {
        private:
            UnaryOperator m_operator;
            Expression m_operand;
        public:
            UnaryExpression(UnaryOperator i_operator, Expression operand);

            Value evaluate(const EvaluationContext& context) const;
            ExpressionVariant optimize() const;

            friend bool operator==(const UnaryExpression& lhs, const UnaryExpression& rhs);
            friend bool operator!=(const UnaryExpression& lhs, const UnaryExpression& rhs);
            friend std::ostream& operator<<(std::ostream& str, const UnaryExpression& exp);
        };
        
        enum class BinaryOperator {
            Addition,
            Subtraction,
            Multiplication,
            Division,
            Modulus,
            LogicalAnd,
            LogicalOr,
            BitwiseAnd,
            BitwiseXOr,
            BitwiseOr,
            BitwiseShiftLeft,
            BitwiseShiftRight,
            Less,
            LessOrEqual,
            Greater,
            GreaterOrEqual,
            Equal,
            NotEqual,
            Range,
            Case,
        };
        
        class BinaryExpression {
        public:
            friend class Expression;
        private:
            BinaryOperator m_operator;
            Expression m_leftOperand;
            Expression m_rightOperand;
        public:
            BinaryExpression(BinaryOperator i_operator, Expression leftOperand, Expression rightOperand);
            static Expression createAutoRangeWithRightOperand(Expression rightOperand, size_t line, size_t column);
            static Expression createAutoRangeWithLeftOperand(Expression leftOperand, size_t line, size_t column);

            Value evaluate(const EvaluationContext& context) const;
            ExpressionVariant optimize() const;
            
            size_t precedence() const;

            friend bool operator==(const BinaryExpression& lhs, const BinaryExpression& rhs);
            friend bool operator!=(const BinaryExpression& lhs, const BinaryExpression& rhs);
            friend std::ostream& operator<<(std::ostream& str, const BinaryExpression& exp);
        };
        
        class SubscriptExpression {
        public:
            static const std::string& AutoRangeParameterName();
        private:
            Expression m_leftOperand;
            Expression m_rightOperand;
        public:
            SubscriptExpression(Expression leftOperand, Expression rightOperand);
            
            Value evaluate(const EvaluationContext& context) const;
            ExpressionVariant optimize() const;

            friend bool operator==(const SubscriptExpression& lhs, const SubscriptExpression& rhs);
            friend bool operator!=(const SubscriptExpression& lhs, const SubscriptExpression& rhs);
            friend std::ostream& operator<<(std::ostream& str, const SubscriptExpression& exp);
        };
        
        class SwitchExpression {
        private:
            std::vector<Expression> m_cases;
        public:
            SwitchExpression(std::vector<Expression> cases);

            Value evaluate(const EvaluationContext& context) const;
            ExpressionVariant optimize() const;

            friend bool operator==(const SwitchExpression& lhs, const SwitchExpression& rhs);
            friend bool operator!=(const SwitchExpression& lhs, const SwitchExpression& rhs);
            friend std::ostream& operator<<(std::ostream& str, const SwitchExpression& exp);
        };
    }
}

