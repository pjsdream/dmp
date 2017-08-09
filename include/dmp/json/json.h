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

  Json(Json&& rhs) = default;
  Json& operator=(Json&& rhs) = default;

  static std::shared_ptr<Json> createInt(int v);
  static std::shared_ptr<Json> createBool(bool v);
  static std::shared_ptr<Json> createDouble(double v);
  static std::shared_ptr<Json> createString(const std::string& v);
  static std::shared_ptr<Json> createArray();
  static std::shared_ptr<Json> createObject();

  int toInt();
  bool toBool();
  double toDouble();
  std::string toString();

  void set(bool v);
  void set(int v);
  void set(double v);
  void set(const std::string& v);

  std::shared_ptr<Json> at(int index);
  std::shared_ptr<Json> at(const std::string& key);
  std::shared_ptr<Json> at(std::string&& key);

  // for array only
  void add(const std::shared_ptr<Json>& value);

  // for object only
  void add(const std::string& key, const std::shared_ptr<Json>& value);
  bool containsKey(const std::string& key);

private:
  Type type_;

  union
  {
    int int_value_;
    bool bool_value_;
    double double_value_;
  };

  std::string string_value_;
  std::vector<std::shared_ptr<Json>> array_;
  std::unordered_map<std::string, std::shared_ptr<Json>> object_;
};
}

#endif //DMP_JSON_H
