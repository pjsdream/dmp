#ifndef DMP_SHADER_H
#define DMP_SHADER_H

#include <dmp/rendering/gl_base.h>

#include <string>
#include <memory>

#include <Eigen/Dense>

namespace dmp
{
class Shader
{
public:
  enum class ShaderType
  {
    Vertex,
    Fragment
  };

  Shader() = delete;
  explicit Shader(const std::shared_ptr<GlFunctions>& gl);

  void loadShader(const std::string& filename, ShaderType type);
  void linkShader();

  void start();
  void end();

  void uniform(GLint location, int v);
  void uniform(GLint location, float v);
  void uniform(GLint location, const Eigen::Vector3f& v);
  void uniformMatrix4f(GLint location, const Eigen::Matrix4f& matrix);

protected:
  std::shared_ptr<GlFunctions> gl_;

  GLint getUniformLocation(const std::string& name);
  void bindAttribLocation(GLuint index, const std::string& name);

private:
  static GLuint shaderTypeToGlType(ShaderType type);

  std::vector<GLuint> shaders_;
  GLuint program_;
};
}

#endif // DMP_SHADER_H
