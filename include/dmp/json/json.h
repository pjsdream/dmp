#ifndef DMP_JSON_H
#define DMP_JSON_H

#include <string>
#include <memory>
#include <vector>
#include <unordered_map>

namespace dmp
{
class Json
{
private:
  enum class Type
  {
    Undefined,
    Int,
    Bool,
    Double,
    String,
    Array,
    Object,
  };

public:
  Json();
  ~Json() = default;

  explicit Json(int v);
  explicit Json(bool v);
  explicit Json(double v);
  explicit Json(const std::string& v);
  explicit Json(std::string&& v);

  Json(const Json& rhs) = default;
  Json& operator=(const Json& rhs) = default;

  Json(Json&& rhs) = default;
  Json& operator=(Json&& rhs) = default;

  int toInt();
  bool toBool();
  double toDouble();
  std::string toString();

  void set(bool v);
  void set(int v);
  void set(double v);
  void set(const std::string& v);

  Json& operator[](int index);
  Json& operator[](const std::string& key);
  Json& operator[](std::string&& key);

  Json& operator=(int v);
  Json& operator=(bool v);
  Json& operator=(double v);
  Json& operator=(const char* v);
  Json& operator=(const std::string& v);
  Json& operator=(std::string&& v);

  // for array only
  int size();
  void add(const Json& value);
  void add(Json&& value);

  // for object only
  bool containsKey(const std::string& key);
  bool containsKey(std::string&& key);

private:
  Type type_;

  union
  {
    int int_value_;
    bool bool_value_;
    double double_value_;
  };

  std::string string_value_;
  std::vector<Json> array_;
  std::unordered_map<std::string, Json> object_;
};
}

#endif //DMP_JSON_H
