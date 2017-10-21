#include <dmp/comm/serializer.h>
#include <dmp/comm/deserializer.h>

#include <iostream>

class TestClass
{
public:
  TestClass() : s("default")
  {}

  TestClass(int i, char c, float f, double d, std::string s, std::initializer_list<int> il)
      : i(i), c(c), f(f), d(d), s(s), v(il)
  {}

  template<typename Archive>
  Archive& serialize(Archive& ar)
  {
    ar & i & c & f & d & s & v;
    return ar;
  }

  void print()
  {
    std::cout << "TestClass int " << i << "\n"
              << "TestClass char " << c << "\n"
              << "TestClass float " << f << "\n"
              << "TestClass double " << d << "\n"
              << "TestClass string " << s << "\n"
              << "TestClass vector { ";
    for (int i = 0; i < v.size(); i++)
      std::cout << v[i] << " ";
    std::cout << "}\n";
  }

private:
  int i = 0;
  char c = 0;
  float f = 0.f;
  double d = 0.;
  std::string s;
  std::vector<int> v;
};

int main()
{
  using namespace dmp;

  std::vector<char> buffer;
  {
    Serializer serializer(buffer);
    serializer << int(-1) << char('A') << float(5.6f) << double(7.8) << std::string("i am str-ing");
  }
  {
    Deserializer deserializer(buffer);
    int i;
    char c;
    float f;
    double d;
    std::string s;
    deserializer >> i >> c >> f >> d >> s;

    std::cout << "int " << i << "\n"
              << "char " << c << "\n"
              << "float " << f << "\n"
              << "double " << d << "\n"
              << "string " << s << "\n";
  }
  buffer.clear();

  {
    Serializer serializer(buffer);
    serializer << TestClass(-2, 'B', 1.2f, 3.4, "another i am str-ing", {1, 3, 5});
  }
  {
    Deserializer deserializer(buffer);
    TestClass test_class;
    deserializer >> test_class;
    test_class.print();
  }
}
