#ifndef DMP_JSON_LOADER_H
#define DMP_JSON_LOADER_H

#include <string>
#include <memory>

namespace dmp
{
class Json;
class JsonLoader
{
public:
  Json loadJson(const std::string& filename);

private:
  Json parseJsonFromString(int& x);
  Json parseJsonStringFromString(int& x);
  Json parseJsonArrayFromString(int& x);
  Json parseJsonObjectFromString(int& x);
  void moveIndexToNonWhitespace(int& x);

  static bool isWhitespace(char x);
  static bool isDigit(char x);

  std::string json_string_;
};
}

#endif //DMP_JSON_LOADER_H
