#include <dmp/gui/shader.h>
#include <QtGui/qopengl.h>

namespace dmp
{
Shader::Shader() = default;

void Shader::loadShader(const std::string& filename, ShaderType type)
{
  FILE* fp = fopen(filename.c_str(), "rb");
  if (fp == NULL)
    return;

  fseek(fp, 0, SEEK_END);
  int len = ftell(fp);
  fseek(fp, 0, SEEK_SET);

  GLchar* source = new GLchar[len + 1];
  fread(source, 1, len, fp);
  fclose(fp);

  source[len] = 0;

  const GLchar* const_source = const_cast<const GLchar*>(source);

  GLuint shader = gl_->glCreateShader(shaderTypeToGlType(type));

  gl_->glShaderSource(shader, 1, &const_source, NULL);
  delete source;

  gl_->glCompileShader(shader);

  GLint compiled;
  gl_->glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);

  if (!compiled)
  {
    GLsizei len;
    gl_->glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &len);

    GLchar* log = new GLchar[len + 1];
    gl_->glGetShaderInfoLog(shader, len, &len, log);
    fprintf(stderr, "Shader compilation failed:\n%s\n", log);
    delete log;
  }

  shaders_.push_back(shader);
}

void Shader::createShader()
{
  std::make_shared<Shader>();

  GLuint program = gl_->glCreateProgram();

  for (GLuint shader : shaders_)
    gl_->glAttachShader(program, shader);

  gl_->glLinkProgram(program);

  GLint linked;
  gl_->glGetProgramiv(program, GL_LINK_STATUS, &linked);

  if (!linked)
  {
    GLsizei len;
    gl_->glGetProgramiv(program, GL_INFO_LOG_LENGTH, &len);

    GLchar* log = new GLchar[len + 1];
    gl_->glGetProgramInfoLog(program, len, &len, log);
    fprintf(stderr, "Shader linking failed:\n%s\n", log);
    delete log;

    return;
  }
}

GLuint Shader::shaderTypeToGlType(ShaderType type)
{
  switch (type)
  {
    case ShaderType::Vertex:return GL_VERTEX_SHADER;
    case ShaderType::Fragment:return GL_FRAGMENT_SHADER;
  }
}
}