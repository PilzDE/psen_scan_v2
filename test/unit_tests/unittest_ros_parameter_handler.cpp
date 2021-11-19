// Copyright (c) 2020-2021 Pilz GmbH & Co. KG
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <string>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "psen_scan_v2/ros_parameter_handler.h"

namespace psen_scan_v2_test
{
using namespace psen_scan_v2;

template <class T, class TIncorrect>
class ParamTestItem
{
public:
  virtual ~ParamTestItem() = default;

public:
  T callRequiredGetter(const rclcpp::Node::SharedPtr& node, const std::string& param_name)
  {
    return getRequiredParam<T>(*node, param_name);
  }

  T callOptionalGetter(const rclcpp::Node::SharedPtr& node, const std::string& param_name)
  {
    return getOptionalParam<T>(*node, param_name, getDefaultValue());
  }

  virtual T getDefaultValue() const = 0;
  virtual T getArbitraryValue() const = 0;
  virtual TIncorrect getIncorrectTypeValue() const = 0;
};

class StringTestItem : public ParamTestItem<std::string, double>
{
public:
  StringTestItem() : ParamTestItem<std::string, double>()
  {
  }

public:
  std::string getDefaultValue() const override
  {
    return "Default string value";
  }
  std::string getArbitraryValue() const override
  {
    return "Arbitrary string value";
  }
  double getIncorrectTypeValue() const override
  {
    return 0.;
  }
};

class IntTestItem : public ParamTestItem<int, std::string>
{
public:
  IntTestItem() : ParamTestItem<int, std::string>()
  {
  }

public:
  int getDefaultValue() const override
  {
    return 7;
  }
  int getArbitraryValue() const override
  {
    return 403;
  }
  std::string getIncorrectTypeValue() const override
  {
    return "Incorrect type value";
  }
};

class DoubleTestItem : public ParamTestItem<double, std::string>
{
public:
  DoubleTestItem() : ParamTestItem<double, std::string>()
  {
  }

public:
  double getDefaultValue() const override
  {
    return 5.;
  }
  double getArbitraryValue() const override
  {
    return 1007.;
  }
  std::string getIncorrectTypeValue() const override
  {
    return "Incorrect type value";
  }
};

template <typename T>
class ParamTestSuite : public ::testing::Test
{
};

using TypesToTest = ::testing::Types<StringTestItem, IntTestItem, DoubleTestItem>;
#ifdef TYPED_TEST_SUITE  // in this case TYPED_TEST_CASE is deprecated
TYPED_TEST_SUITE(ParamTestSuite, TypesToTest);
#else
TYPED_TEST_CASE(ParamTestSuite, TypesToTest);
#endif

TYPED_TEST(ParamTestSuite, testParamNotOnServer)
{
  TypeParam test_item;
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("testParamNotOnServer_node");
  const std::string param_name{ "MissingParam" };
  ASSERT_THROW(test_item.callRequiredGetter(node, param_name), ParameterNotSet);
  ASSERT_EQ(test_item.getDefaultValue(), test_item.callOptionalGetter(node, param_name)) << "Default value incorrect ";
}

TYPED_TEST(ParamTestSuite, testIncorrectParamType)
{
  TypeParam test_item;
  const std::string param_name{ "ParamWithIncorrectType" };

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("testIncorrectParamType_node");
  node->declare_parameter(param_name, test_item.getIncorrectTypeValue());
  ASSERT_THROW(test_item.callRequiredGetter(node, param_name), rclcpp::exceptions::InvalidParameterTypeException);
  ASSERT_THROW(test_item.callOptionalGetter(node, param_name), rclcpp::exceptions::InvalidParameterTypeException);
}

TYPED_TEST(ParamTestSuite, testGettingOfParam)
{
  TypeParam test_item;
  const std::string param_name{ "CorrectParam" };

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("testGettingOfParam_node");
  node->declare_parameter(param_name, test_item.getArbitraryValue());
  ASSERT_EQ(test_item.getArbitraryValue(), test_item.callRequiredGetter(node, param_name)) << "Param value incorrect";
  ASSERT_EQ(test_item.getArbitraryValue(), test_item.callOptionalGetter(node, param_name)) << "Param value incorrect";
}

}  // namespace psen_scan_v2_test

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
