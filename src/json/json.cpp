#include <dmp/json/json.h>

namespace dmp
{
Json::Json()
    : type_(Type::Undefined)
{
}

std::shared_ptr<Json> Json::createInt(int v)
{
  auto json = std::make_shared<Json>();
  json->set(v);
  return json;
}
std::shared_ptr<Json> Json::createBool(bool v)
{
  auto json = std::make_shared<Json>();
  json->set(v);
  return json;
}
std::shared_ptr<Json> Json::createDouble(double v)
{
  auto json = std::make_shared<Json>();
  json->set(v);
  return json;
}
std::shared_ptr<Json> Json::createString(const std::string& v)
{
  auto json = std::make_shared<Json>();
  json->set(v);
  return json;
}
std::shared_ptr<Json> Json::createArray()
{
  return std::make_shared<Json>();
}
std::shared_ptr<Json> Json::createObject()
{
  return std::make_shared<Json>();
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

std::shared_ptr<Json> Json::at(int index)
{
  if (type_ != Type::Array)
  {
    string_value_.clear();
    object_.clear();
    type_ = Type::Array;
  }

  return array_[index];
}
std::shared_ptr<Json> Json::at(const std::string& key)
{
  if (type_ != Type::Object)
  {
    string_value_.clear();
    array_.clear();
    type_ = Type::Object;
  }

  return object_[key];
}
std::shared_ptr<Json> Json::at(std::string&& key)
{
  if (type_ != Type::Object)
  {
    string_value_.clear();
    array_.clear();
    type_ = Type::Object;
  }

  return object_[key];
}

// for array only
void Json::add(const std::shared_ptr<Json>& value)
{
  if (type_ != Type::Array)
  {
    string_value_.clear();
    object_.clear();
    type_ = Type::Array;
  }

  array_.push_back(value);
}

// for object only
void Json::add(const std::string& key, const std::shared_ptr<Json>& value)
{
  if (type_ != Type::Object)
  {
    string_value_.clear();
    array_.clear();
    type_ = Type::Object;
  }

  object_[key] = value;
}
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
}