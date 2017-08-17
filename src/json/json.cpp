#include <dmp/json/json.h>

namespace dmp
{
Json::Json()
    : type_(Type::Undefined)
{
}

Json::Json(long long v)
    : type_(Type::Int), int_value_(v)
{
}

Json::Json(int v)
    : type_(Type::Int), int_value_(v)
{
}

Json::Json(bool v)
    : type_(Type::Bool), bool_value_(v)
{
}

Json::Json(double v)
    : type_(Type::Double), double_value_(v)
{
}

Json::Json(const std::string& v)
    : type_(Type::String), string_value_(v)
{
}

Json::Json(std::string&& v)
    : type_(Type::String), string_value_(std::move(v))
{
}

long long Json::toInt()
{
  switch (type_)
  {
    case Type::Int:
      return int_value_;
    case Type::Bool:
      return bool_value_;
    case Type::Double:
      return double_value_;
    default:
      return 0L;
  }
}

bool Json::toBool()
{
  switch (type_)
  {
    case Type::Bool:
      return bool_value_;
    case Type::Int:
      return int_value_ != 0;
    case Type::Double:
      return double_value_ != 0.;
    default:
      return false;
  }
}

double Json::toDouble()
{
  switch (type_)
  {
    case Type::Double:
      return double_value_;
    case Type::Int:
      return int_value_;
    case Type::Bool:
      return bool_value_;
    default:
      return 0.;
  }
}

std::string Json::toString()
{
  switch (type_)
  {
    case Type::String:
      return string_value_;
    default:
      return "";
  }
}

void Json::set(bool v)
{
  clear();
  type_ = Type::Bool;
  bool_value_ = v;
}

void Json::set(int v)
{
  clear();
  type_ = Type::Int;
  int_value_ = v;
}

void Json::set(long long v)
{
  clear();
  type_ = Type::Int;
  int_value_ = v;
}

void Json::set(double v)
{
  clear();
  type_ = Type::Double;
  double_value_ = v;
}

void Json::set(const std::string& v)
{
  clear();
  type_ = Type::String;
  string_value_ = v;
}

Json& Json::operator[](int index)
{
  return array_[index];
}

Json& Json::operator[](const std::string& key)
{
  return *object_[key];
}

Json& Json::operator[](std::string&& key)
{
  return *object_[key];
}

Json& Json::operator=(int v)
{
  set(v);
  return *this;
}

Json& Json::operator=(bool v)
{
  set(v);
  return *this;
}

Json& Json::operator=(double v)
{
  set(v);
  return *this;
}

Json& Json::operator=(const char* v)
{
  set(std::string(v));
  return *this;
}

Json& Json::operator=(const std::string& v)
{
  set(v);
  return *this;
}

Json& Json::operator=(std::string&& v)
{
  set(std::move(v));
  return *this;
}

unsigned long Json::size()
{
  switch (type_)
  {
    case Type::Array:
      return array_.size();
    case Type::Object:
      return object_.size();
    default:
      return 0;
  }
}

void Json::add(const Json& value)
{
  if (type_ != Type::Array)
  {
    clear();
    type_ = Type::Array;
  }
  array_.push_back(value);
}

void Json::add(Json&& value)
{
  if (type_ != Type::Array)
  {
    clear();
    type_ = Type::Array;
  }
  array_.emplace_back(value);
}

bool Json::containsKey(const std::string& key)
{
  return object_.find(key) != object_.cend();
}

bool Json::containsKey(std::string&& key)
{
  return object_.find(std::move(key)) != object_.cend();
}

void Json::clear()
{
  array_.clear();
  object_.clear();
}
}