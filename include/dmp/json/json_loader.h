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
  std::shared_ptr<Json> loadJson(const std::string& filename);

private:
  std::shared_ptr<Json> parseJsonFromString(int& x);

  static bool isWhitespace(char x);

  std::string json_string_;
};
}

#endif //DMP_JSON_LOADER_H
