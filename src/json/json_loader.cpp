#include <dmp/json/json_loader.h>
#include <dmp/json/json.h>

#include <stack>

namespace dmp
{
std::shared_ptr<Json> JsonLoader::loadJson(const std::string& filename)
{
  json_string_.clear();

  std::FILE* fp = fopen(filename, "r");
  char buffer[1024];
  while (fgets(buffer, sizeof buffer, fp) != NULL)
    json_string_ += buffer;
  std::fclose(fp);

  int x = 0;
  return parseJsonFromString(x);
}

std::shared_ptr<Json> JsonLoader::parseJsonFromString(int& x)
{
  while (isWhitespace(json_string_[x]))
    x++;

  switch (json_string_[x])
  {
    // string
    case '\"':
    {
      std::string string_value_;
      x++;
      while (json_string_[x] != '\"')
      {
        if (json_string_[x] == '\\')
        {
          // TODO
        } else
          string_value_ += json_string_[x++];
      }
      return std::make_shared<Json>(string_value_);
    }

      // bool(true)
    case 't':
    {
      x += 4;
      return std::make_shared<Json>(true);
    }

      // bool(false)
    case 'f':
    {
      x += 5;
      return std::make_shared<Json>(false);
    }

      // array
    case '[':
    {
      x++;
      return parseJsonArrayFromString(x);
    }

      // object
    case '{':
    {
      x++;
      return parseJsonObjectFromString(x);
    }

      // number
    default:
    {
      enum class State
      {
        MinusOrNone = 0,
        ZeroOrDigit,
        Digit,
        DotOrNone,
        DigitAfterDot,
        ExponentOrNone,
        ExponentSignOrNone,
        DigitAfterExponent
      };

      State state = State::MinusOrNone;
      int sign = 1;
      double num = 0;
      while (x < json_string_.size())
      {
        switch (state)
        {
          case State::MinusOrNone:
          {
            if (json_string_[x] == '-')
            {
              sign = -1;
              x++;
            }
            state = State::ZeroOrDigit;
          }
            break;

          case State::ZeroOrDigit:
          {
            if ()
            {
            } else
            {
              num = num * 10. + json_string_[x] - '0';
              x++;
            }
          }
            break;
        }
      }
      return std::make_shared<Json>(string_value_);
    }
  }
}

bool JsonLoader::isWhitespace(char x)
{
  return x == '\n' || x == '\r' || x == ' ' || x == '\t';
}
}
