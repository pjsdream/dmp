#include <dmp/json/json.h>

namespace dmp
{
/*
Json::Json()
    : type_(Type::Undefined)
{
}


Json::Json(int v)
{
  type_ = Type::Int;
  int_value_ = v;
}
Json::Json(bool v)
{
  type_ = Type::Bool;
  bool_value_ = v;
}
Json::Json(double v)
{
  type_ = Type::Double;
  double_value_ = v;
}
Json::Json(const std::string& v)
{
  type_ = Type::String;
  string_value_ = v;
}
Json::Json(std::string&& v)
{
  type_ = Type::String;
  string_value_ = std::move(v);
}

int Json::toInt()
{
  switch (type_)
  {
    case Type::Int: return int_value_;
    default: return 0;
  }
}
bool Json::toBool()
{
  switch (type_)
  {
    case Type::Bool: return bool_value_;
    default: return false;
  }
}
double Json::toDouble()
{
  switch (type_)
  {
    case Type::Double: return double_value_;
    default: return 0.;
  }
}
std::string Json::toString()
{
  switch (type_)
  {
    case Type::String: return string_value_;
    default: return "";
  }
}

void Json::set(bool v)
{
  if (type_ != Type::Bool)
  {
    string_value_.clear();
    array_.clear();
    object_.clear();
    type_ = Type::Bool;
  }

  bool_value_ = v;
}
void Json::set(int v)
{
  if (type_ != Type::Int)
  {
    string_value_.clear();
    array_.clear();
    object_.clear();
    type_ = Type::Int;
  }

  int_value_ = v;
}
void Json::set(double v)
{
  if (type_ != Type::Double)
  {
    string_value_.clear();
    array_.clear();
    object_.clear();
    type_ = Type::Double;
  }

  double_value_ = v;
}
void Json::set(const std::string& v)
{
  if (type_ != Type::String)
  {
    array_.clear();
    object_.clear();
    type_ = Type::String;
  }

  string_value_ = v;
}

Json& Json::operator[](int index)
{
  if (type_ != Type::Array)
  {
    string_value_.clear();
    object_.clear();
    type_ = Type::Array;
  }

  return array_[index];
}
Json& Json::operator[](const std::string& key)
{
  if (type_ != Type::Object)
  {
    string_value_.clear();
    array_.clear();
    type_ = Type::Object;
  }

  return object_[key];
}
Json& Json::operator[](std::string&& key)
{
  if (type_ != Type::Object)
  {
    string_value_.clear();
    array_.clear();
    type_ = Type::Object;
  }

  return object_[key];
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
  *this = std::string(v);
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

// for array only
int Json::size()
{
  return array_.size();
}
void Json::add(const Json& value)
{
  if (type_ != Type::Array)
  {
    string_value_.clear();
    object_.clear();
    type_ = Type::Array;
  }

  array_.push_back(value);
}
void Json::add(Json&& value)
{
  if (type_ != Type::Array)
  {
    string_value_.clear();
    object_.clear();
    type_ = Type::Array;
  }

  array_.push_back(std::move(value));
}

// for object only
bool Json::containsKey(const std::string& key)
{
  if (type_ != Type::Object)
  {
    string_value_.clear();
    array_.clear();
    type_ = Type::Object;
  }

  return object_.find(key) != object_.cend();
}
bool Json::containsKey(std::string&& key)
{
  if (type_ != Type::Object)
  {
    string_value_.clear();
    array_.clear();
    type_ = Type::Object;
  }

  return object_.find(key) != object_.cend();
}
*/
}