#ifndef DMP_SHADER_H
#define DMP_SHADER_H

#include <string>

#include <dmp/rendering/gl_base.h>

namespace dmp
{
class Shader : public GlBase
{
public:
  enum class ShaderType
  {
    Vertex,
    Fragment
  };

  Shader();

  void loadShader(const std::string& filename, ShaderType type);
  void createShader();

private:
  GLuint shaderTypeToGlType(ShaderType type);

  std::vector<GLuint> shaders_;
};
}

#endif // DMP_SHADER_H
