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

#include "Expressions.h"

#include "Ensure.h"
#include "Macros.h"
#include "EL/EvaluationContext.h"

#include <kdl/overload.h>
#include <kdl/vector_utils.h>

#include <algorithm>
#include <sstream>
#include <string>

namespace TrenchBroom {
    namespace EL {
        LiteralExpression::LiteralExpression(Value value) :
        m_value{std::move(value)} {}
        
        const Value& LiteralExpression::evaluate(const EvaluationContext&) const {
            return m_value;
        }
        
        bool operator==(const LiteralExpression& lhs, const LiteralExpression& rhs) {
            return lhs.m_value == rhs.m_value;
        }

        bool operator!=(const LiteralExpression& lhs, const LiteralExpression& rhs) {
            return !(lhs == rhs);
        }

        std::ostream& operator<<(std::ostream& str, const LiteralExpression& exp) {
            str << exp.m_value;
            return str;
        }

        VariableExpression::VariableExpression(std::string variableName) :
        m_variableName{std::move(variableName)} {}
        
        Value VariableExpression::evaluate(const EvaluationContext& context) const {
            return context.variableValue(m_variableName);
        }
        
        bool operator==(const VariableExpression& lhs, const VariableExpression& rhs) {
            return lhs.m_variableName == rhs.m_variableName;
        }

        bool operator!=(const VariableExpression& lhs, const VariableExpression& rhs) {
            return !(lhs == rhs);
        }

        std::ostream& operator<<(std::ostream& str, const VariableExpression& exp) {
            str << exp.m_variableName;
            return str;
        }

        ArrayExpression::ArrayExpression(std::vector<Expression> elements) :
        m_elements{std::move(elements)} {}
        
        Value ArrayExpression::evaluate(const EvaluationContext& context) const {
            auto array = ArrayType{};
            array.reserve(m_elements.size());
            for (const auto& element : m_elements) {
                auto value = element.evaluate(context);
                if (value.type() == ValueType::Range) {
                    const auto& range = value.rangeValue();
                    if (!range.empty()) {
                        array.reserve(array.size() + range.size() - 1u);
                        for (size_t i = 0u; i < range.size(); ++i) {
                            array.emplace_back(range[i], value.line(), value.column());
                        }
                    }
                } else {
                    array.push_back(std::move(value));
                }
            }
            
            return Value{std::move(array)};
        }
        
        ExpressionVariant ArrayExpression::optimize() const {
            auto optimizedExpressions = kdl::vec_transform(m_elements, [](const auto& expression) {
                return expression.optimize();
            });
            
            auto values = ArrayType{};
            values.reserve(m_elements.size());

            const auto evaluationContext = EvaluationContext{};
            for (const auto& expression : optimizedExpressions) {
                auto value = expression.evaluate(evaluationContext);
                if (value.undefined()) {
                    return ArrayExpression{std::move(optimizedExpressions)};
                }

                values.push_back(std::move(value));
            }
            
            return LiteralExpression{Value{std::move(values)}};
        }

        bool operator==(const ArrayExpression& lhs, const ArrayExpression& rhs) {
            return lhs.m_elements == rhs.m_elements;
        }

        bool operator!=(const ArrayExpression& lhs, const ArrayExpression& rhs) {
            return !(lhs == rhs);
        }

        std::ostream& operator<<(std::ostream& str, const ArrayExpression& exp) {
            str << "[ ";
            size_t i = 0u;
            for (const auto& expression : exp.m_elements) {
                str << expression;
                if (i < exp.m_elements.size() - 1u) {
                    str << ", ";
                }
                ++i;
            }
            str << " ]";
            return str;
        }
        
        MapExpression::MapExpression(std::map<std::string, Expression> elements) :
        m_elements{std::move(elements)} {}

        Value MapExpression::evaluate(const EvaluationContext& context) const {
            auto map = MapType{};
            for (const auto& [key, expression] : m_elements) {
                map.emplace(key, expression.evaluate(context));
            }

            return Value{std::move(map)};
        }
        
        ExpressionVariant MapExpression::optimize() const {
            auto optimizedExpressions = std::map<std::string, Expression>{};
            for (const auto& [key, expression] : m_elements) {
                optimizedExpressions.emplace(key, expression.optimize());
            }

            auto values = MapType{};

            const auto evaluationContext = EvaluationContext{};
            for (const auto& [key, expression] : optimizedExpressions) {
                auto value = expression.evaluate(evaluationContext);
                if (value.undefined()) {
                    return MapExpression{std::move(optimizedExpressions)};
                }
                values.emplace(key, std::move(value));
            }
            
            return LiteralExpression{Value{std::move(values)}};
        }

        bool operator==(const MapExpression& lhs, const MapExpression& rhs) {
            return lhs.m_elements == rhs.m_elements;
        }

        bool operator!=(const MapExpression& lhs, const MapExpression& rhs) {
            return !(lhs == rhs);
        }

        std::ostream& operator<<(std::ostream& str, const MapExpression& exp) {
            str << "{ ";
            size_t i = 0u;
            for (const auto& [key, expression] : exp.m_elements) {
                str << "\"" << key << "\": " << expression;
                if (i < exp.m_elements.size() - 1u) {
                    str << ", ";
                }
                ++i;
            }
            str << " }";
            return str;
        }

        UnaryExpression::UnaryExpression(const UnaryOperator i_operator, Expression operand) :
        m_operator{i_operator},
        m_operand{std::move(operand)} {}

        static Value evaluateUnaryExpression(const UnaryOperator& operator_, const Value& operand) {
            if (operand.undefined()) {
                return Value::Undefined;
            }

            switch (operator_) {
                case UnaryOperator::Plus:
                    return Value{+operand};
                case UnaryOperator::Minus:
                    return Value{-operand};
                case UnaryOperator::LogicalNegation:
                    return Value{!operand};
                case UnaryOperator::BitwiseNegation:
                    return Value{~operand};
                case UnaryOperator::Group:
                    return Value{operand};
                switchDefault();
            }
        }

        Value UnaryExpression::evaluate(const EvaluationContext& context) const {
            return evaluateUnaryExpression(m_operator, m_operand.evaluate(context));
        }
        
        ExpressionVariant UnaryExpression::optimize() const {
            auto optimizedOperand = m_operand.optimize();
            if (auto value = evaluateUnaryExpression(m_operator, optimizedOperand.evaluate(EvaluationContext{})); !value.undefined()) {
                return LiteralExpression{std::move(value)};
            }
            return UnaryExpression{m_operator, std::move(optimizedOperand)};
        }

        bool operator==(const UnaryExpression& lhs, const UnaryExpression& rhs) {
            return lhs.m_operator == rhs.m_operator && lhs.m_operand == rhs.m_operand;
        }

        bool operator!=(const UnaryExpression& lhs, const UnaryExpression& rhs) {
            return !(lhs == rhs);
        }

        std::ostream& operator<<(std::ostream& str, const UnaryExpression& exp) {
            switch (exp.m_operator) {
                case UnaryOperator::Plus:
                    str << "+" << exp.m_operand;
                    break;
                case UnaryOperator::Minus:
                    str << "-" << exp.m_operand;
                    break;
                case UnaryOperator::LogicalNegation:
                    str << "!" << exp.m_operand;
                    break;
                case UnaryOperator::BitwiseNegation:
                    str << "~" << exp.m_operand;
                    break;
                case UnaryOperator::Group:
                    str << "( " << exp.m_operand << " )";
                    break;
                switchDefault();
            }
            return str;
        }

        BinaryExpression::BinaryExpression(const BinaryOperator i_operator, Expression leftOperand, Expression rightOperand) :
        m_operator{i_operator},
        m_leftOperand{std::move(leftOperand)},
        m_rightOperand{std::move(rightOperand)} {}

        Expression BinaryExpression::createAutoRangeWithRightOperand(Expression rightOperand, const size_t line, const size_t column) {
            auto leftOperand = Expression{VariableExpression{SubscriptExpression::AutoRangeParameterName()}, line, column};
            return Expression{BinaryExpression{BinaryOperator::Range, std::move(leftOperand), std::move(rightOperand)}, line, column};
        }
        
        Expression BinaryExpression::createAutoRangeWithLeftOperand(Expression leftOperand, const size_t line, const size_t column) {
            auto rightOperand = Expression{VariableExpression{SubscriptExpression::AutoRangeParameterName()}, line, column};
            return Expression{BinaryExpression{BinaryOperator::Range, std::move(leftOperand), std::move(rightOperand)}, line, column};
        }

        static Value evaluateBinaryExpression(const BinaryOperator operator_, const Value& leftOperand, const Value& rightOperand) {
            if (leftOperand.undefined() || rightOperand.undefined()) {
                return Value::Undefined;
            }

            switch (operator_) {
                case BinaryOperator::Addition:
                    return Value{leftOperand + rightOperand};
                case BinaryOperator::Subtraction:
                    return Value{leftOperand - rightOperand};
                case BinaryOperator::Multiplication:
                    return Value{leftOperand * rightOperand};
                case BinaryOperator::Division:
                    return Value{leftOperand / rightOperand};
                case BinaryOperator::Modulus:
                    return Value{leftOperand % rightOperand};
                case BinaryOperator::LogicalAnd:
                    return Value{leftOperand && rightOperand};
                case BinaryOperator::LogicalOr:
                    return Value{leftOperand || rightOperand};
                case BinaryOperator::BitwiseAnd:
                    return Value{leftOperand & rightOperand};
                case BinaryOperator::BitwiseXOr:
                    return Value{leftOperand ^ rightOperand};
                case BinaryOperator::BitwiseOr:
                    return Value{leftOperand | rightOperand};
                case BinaryOperator::BitwiseShiftLeft:
                    return Value{leftOperand << rightOperand};
                case BinaryOperator::BitwiseShiftRight:
                    return Value{leftOperand >> rightOperand};
                case BinaryOperator::Less:
                    return Value{leftOperand < rightOperand};
                case BinaryOperator::LessOrEqual:
                    return Value{leftOperand <= rightOperand};
                case BinaryOperator::Greater:
                    return Value{leftOperand > rightOperand};
                case BinaryOperator::GreaterOrEqual:
                    return Value{leftOperand >= rightOperand};
                case BinaryOperator::Equal:
                    return Value{leftOperand == rightOperand};
                case BinaryOperator::NotEqual:
                    return Value{leftOperand != rightOperand};
                case BinaryOperator::Range: {
                    const auto from = static_cast<long>(leftOperand.convertTo(ValueType::Number).numberValue());
                    const auto to = static_cast<long>(rightOperand.convertTo(ValueType::Number).numberValue());
                    
                    auto range = RangeType{};
                    if (from <= to) {
                        range.reserve(static_cast<size_t>(to - from + 1));
                        for (long i = from; i <= to; ++i) {
                            assert(range.capacity() > range.size());
                            range.push_back(i);
                        }
                    } else if (to < from) {
                        range.reserve(static_cast<size_t>(from - to + 1));
                        for (long i = from; i >= to; --i) {
                            assert(range.capacity() > range.size());
                            range.push_back(i);
                        }
                    }
                    assert(range.capacity() == range.size());

                    return Value{std::move(range)};
                }
                case BinaryOperator::Case: {
                    if (leftOperand.convertTo(ValueType::Boolean)) {
                        return rightOperand;
                    } else {
                        return Value::Undefined;
                    }
                }
                switchDefault();
            };
        }

        Value BinaryExpression::evaluate(const EvaluationContext& context) const {
            return evaluateBinaryExpression(m_operator, m_leftOperand.evaluate(context), m_rightOperand.evaluate(context));
        }
        
        ExpressionVariant BinaryExpression::optimize() const {
            auto optimizedLeftOperand = m_leftOperand.optimize();
            auto optimizedRightOperand = m_rightOperand.optimize();

            const auto evaluationContext = EvaluationContext{};
            auto leftValue = optimizedLeftOperand.evaluate(evaluationContext);
            auto rightValue = optimizedRightOperand.evaluate(evaluationContext);

            if (auto value = evaluateBinaryExpression(m_operator, leftValue, rightValue); !value.undefined()) {
                return LiteralExpression{std::move(value)};
            }

            return BinaryExpression{m_operator, std::move(optimizedLeftOperand), std::move(optimizedRightOperand)};
        }

        size_t BinaryExpression::precedence() const {
            switch (m_operator) {
                case BinaryOperator::Multiplication:
                case BinaryOperator::Division:
                case BinaryOperator::Modulus:
                    return 12;
                case BinaryOperator::Addition:
                case BinaryOperator::Subtraction:
                    return 11;
                case BinaryOperator::BitwiseShiftLeft:
                case BinaryOperator::BitwiseShiftRight:
                    return 10;
                case BinaryOperator::Less:
                case BinaryOperator::LessOrEqual:
                case BinaryOperator::Greater:
                case BinaryOperator::GreaterOrEqual:
                    return 9;
                case BinaryOperator::Equal:
                case BinaryOperator::NotEqual:
                    return 8;
                case BinaryOperator::BitwiseAnd:
                    return 7;
                case BinaryOperator::BitwiseXOr:
                    return 6;
                case BinaryOperator::BitwiseOr:
                    return 5;
                case BinaryOperator::LogicalAnd:
                    return 4;
                case BinaryOperator::LogicalOr:
                    return 3;
                case BinaryOperator::Range:
                    return 2;
                case BinaryOperator::Case:
                    return 1;
                switchDefault();
            };
        }

        bool operator==(const BinaryExpression& lhs, const BinaryExpression& rhs) {
            return lhs.m_operator == rhs.m_operator && lhs.m_leftOperand == rhs.m_leftOperand && lhs.m_rightOperand == rhs.m_rightOperand;;
        }

        bool operator!=(const BinaryExpression& lhs, const BinaryExpression& rhs) {
            return !(lhs == rhs);
        }

        std::ostream& operator<<(std::ostream& str, const BinaryExpression& exp) {
            switch (exp.m_operator) {
                case BinaryOperator::Addition:
                    str << exp.m_leftOperand << " + " << exp.m_rightOperand;
                    break;
                case BinaryOperator::Subtraction:
                    str << exp.m_leftOperand << " - " << exp.m_rightOperand;
                    break;
                case BinaryOperator::Multiplication:
                    str << exp.m_leftOperand << " * " << exp.m_rightOperand;
                    break;
                case BinaryOperator::Division:
                    str << exp.m_leftOperand << " / " << exp.m_rightOperand;
                    break;
                case BinaryOperator::Modulus:
                    str << exp.m_leftOperand << " % " << exp.m_rightOperand;
                    break;
                case BinaryOperator::LogicalAnd:
                    str << exp.m_leftOperand << " && " << exp.m_rightOperand;
                    break;
                case BinaryOperator::LogicalOr:
                    str << exp.m_leftOperand << " || " << exp.m_rightOperand;
                    break;
                case BinaryOperator::BitwiseAnd:
                    str << exp.m_leftOperand << " & " << exp.m_rightOperand;
                    break;
                case BinaryOperator::BitwiseXOr:
                    str << exp.m_leftOperand << " ^ " << exp.m_rightOperand;
                    break;
                case BinaryOperator::BitwiseOr:
                    str << exp.m_leftOperand << " | " << exp.m_rightOperand;
                    break;
                case BinaryOperator::BitwiseShiftLeft:
                    str << exp.m_leftOperand << " << " << exp.m_rightOperand;
                    break;
                case BinaryOperator::BitwiseShiftRight:
                    str << exp.m_leftOperand << " >> " << exp.m_rightOperand;
                    break;
                case BinaryOperator::Less:
                    str << exp.m_leftOperand << " < " << exp.m_rightOperand;
                    break;
                case BinaryOperator::LessOrEqual:
                    str << exp.m_leftOperand << " <= " << exp.m_rightOperand;
                    break;
                case BinaryOperator::Greater:
                    str << exp.m_leftOperand << " > " << exp.m_rightOperand;
                    break;
                case BinaryOperator::GreaterOrEqual:
                    str << exp.m_leftOperand << " >= " << exp.m_rightOperand;
                    break;
                case BinaryOperator::Equal:
                    str << exp.m_leftOperand << " == " << exp.m_rightOperand;
                    break;
                case BinaryOperator::NotEqual:
                    str << exp.m_leftOperand << " != " << exp.m_rightOperand;
                    break;
                case BinaryOperator::Range:
                    str << exp.m_leftOperand << ".." << exp.m_rightOperand;
                    break;
                case BinaryOperator::Case:
                    str << exp.m_leftOperand << " -> " << exp.m_rightOperand;
                    break;
                switchDefault();
            };
            return str;
        }

        const std::string& SubscriptExpression::AutoRangeParameterName() {
            static const std::string Name = "__AutoRangeParameter";
            return Name;
        }

        SubscriptExpression::SubscriptExpression(Expression leftOperand, Expression rightOperand) :
        m_leftOperand{std::move(leftOperand)},
        m_rightOperand{std::move(rightOperand)} {}
        
        Value SubscriptExpression::evaluate(const EvaluationContext& context) const {
            const auto leftValue = m_leftOperand.evaluate(context);
            
            auto stack = EvaluationStack{context};
            stack.declareVariable(AutoRangeParameterName(), Value(leftValue.length() - 1u));
            
            const auto rightValue = m_rightOperand.evaluate(stack);
            return leftValue[rightValue];
        }
        
        ExpressionVariant SubscriptExpression::optimize() const {
            auto optimizedLeftOperand = m_leftOperand.optimize();
            auto optimizedRightOperand = m_rightOperand.optimize();

            auto evaluationContext = EvaluationContext{};
            if (auto leftValue = optimizedLeftOperand.evaluate(evaluationContext); !leftValue.undefined()) {
                auto stack = EvaluationStack{evaluationContext};
                stack.declareVariable(AutoRangeParameterName(), Value(leftValue.length() - 1u));

                if (auto rightValue = optimizedRightOperand.evaluate(stack); !rightValue.undefined()) {
                    if (auto value = leftValue[rightValue]; !value.undefined()) {
                        return LiteralExpression{std::move(value)};
                    }
                }
            }

            return SubscriptExpression{std::move(optimizedLeftOperand), std::move(optimizedRightOperand)};
        }

        bool operator==(const SubscriptExpression& lhs, const SubscriptExpression& rhs) {
            return lhs.m_leftOperand == rhs.m_leftOperand && lhs.m_rightOperand == rhs.m_rightOperand;;
        }

        bool operator!=(const SubscriptExpression& lhs, const SubscriptExpression& rhs) {
            return !(lhs == rhs);
        }

        std::ostream& operator<<(std::ostream& str, const SubscriptExpression& exp) {
            str << exp.m_leftOperand << "[" << exp.m_rightOperand << "]";
            return str;
        }

        SwitchExpression::SwitchExpression(std::vector<Expression> cases) :
        m_cases{std::move(cases)} {}

        Value SwitchExpression::evaluate(const EvaluationContext& context) const {
            for (const auto& case_ : m_cases) {
                auto result = case_.evaluate(context);
                if (!result.undefined()) {
                    return result;
                }
            }
            return Value::Undefined;
        }
        
        ExpressionVariant SwitchExpression::optimize() const {
            if (m_cases.empty()) {
                return *this;
            }

            auto optimizedExpressions = kdl::vec_transform(m_cases, [](const auto& expression) { return expression.optimize(); });
            if (auto firstValue = optimizedExpressions.front().evaluate(EvaluationContext{}); !firstValue.undefined()) {
                return LiteralExpression{std::move(firstValue)};
            }

            return SwitchExpression{std::move(optimizedExpressions)};
        }

        bool operator==(const SwitchExpression& lhs, const SwitchExpression& rhs) {
            return lhs.m_cases == rhs.m_cases;
        }

        bool operator!=(const SwitchExpression& lhs, const SwitchExpression& rhs) {
            return !(lhs == rhs);
        }

        std::ostream& operator<<(std::ostream& str, const SwitchExpression& exp) {
            str << "{{ ";
            size_t i = 0u;
            for (const auto& case_ : exp.m_cases) {
                str << case_;
                if (i < exp.m_cases.size() - 1u) {
                    str << ", ";
                }
                ++i;
            }
            str << " }}";
            return str;
        }
    }
}
