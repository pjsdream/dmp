#include <core/comm/serializer.h>
#include <core/comm/deserializer.h>
#include <core/comm/zmq_serializer.h>

#include <iostream>
#include <core/comm/zmq_deserializer.h>

class TestClass
{
public:
  TestClass() : s("default")
  {}

  TestClass(int i, char c, float f, double d, std::string s, std::initializer_list<int> il, std::initializer_list<std::string> sl)
      : i(i), c(c), f(f), d(d), s(s), v(il), vs(sl)
  {}

  template<typename Archive>
  Archive& serialize(Archive& ar)
  {
    ar & i & c & f & d & s & v & vs;
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

    std::cout << "TestClass string vector { ";
    for (int i = 0; i < vs.size(); i++)
      std::cout << vs[i] << " ";
    std::cout << "}\n";
  }

private:
  int i = 0;
  char c = 0;
  float f = 0.f;
  double d = 0.;
  std::string s;
  std::vector<int> v;
  std::vector<std::string> vs;
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
    Deserializer deserializer(buffer.data());
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
    serializer << TestClass(-2, 'B', 1.2f, 3.4, "another i am str-ing", {1, 3, 5}, {"i", "am", "string", "vector"});
  }
  {
    Deserializer deserializer(buffer.data());
    TestClass test_class;
    deserializer >> test_class;
    test_class.print();
  }

  // Zmq serializer / deserializer
  {
    TestClass value(-2, 'B', 1.2f, 3.4, "another i am str-ing", {1, 3, 5}, {"i", "am", "string", "vector"});

    ZmqSerializerSizeEvaluator size_evaluator;
    size_evaluator << value;

    ZmqSerializer serializer(size_evaluator.size());
    serializer << value;
    const auto& message = serializer.getMessage();

    std::cout << "message size: " << message.size() << "\n";
    for (int i=0; i<message.size(); i++)
      std::cout << static_cast<int>(*(message.data<char>() + i)) << ' ';
    std::cout << "\n";

    ZmqDeserializer deserializer(message);
    TestClass deserialized_value;
    deserializer >> deserialized_value;
    deserialized_value.print();
  }
}
