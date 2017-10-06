#include <include/dmp/rendering/shader/shader.h>

namespace dmp
{
Shader::Shader(const std::shared_ptr<GlFunctions>& gl)
    : gl_(gl)
{
}

Shader::~Shader()
{
  printf("deleting shader\n");
  gl_->glDeleteProgram(program_);
}

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
  delete[] source;

  printf("compiling shader %d\n", shader);
  gl_->glCompileShader(shader);
  printf("shader %d compile done\n", shader);

  GLint compiled;
  gl_->glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);

  if (!compiled)
  {
    GLsizei len;
    gl_->glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &len);

    GLchar* log = new GLchar[len + 1];
    gl_->glGetShaderInfoLog(shader, len, &len, log);
    fprintf(stderr, "Shader compilation failed:\n%s\n", log);
    delete[] log;

    // Don't need the shader anymore
    gl_->glDeleteShader(shader);
  }

  shaders_.push_back(shader);
}

void Shader::linkShader()
{
  program_ = gl_->glCreateProgram();

  for (GLuint shader : shaders_)
    gl_->glAttachShader(program_, shader);

  gl_->glLinkProgram(program_);

  GLint linked;
  gl_->glGetProgramiv(program_, GL_LINK_STATUS, &linked);

  if (!linked)
  {
    GLsizei len;
    gl_->glGetProgramiv(program_, GL_INFO_LOG_LENGTH, &len);

    GLchar* log = new GLchar[len + 1];
    gl_->glGetProgramInfoLog(program_, len, &len, log);
    fprintf(stderr, "Shader linking failed:\n%s\n", log);
    delete[] log;

    // Don't need shaders and program anymore
    gl_->glDeleteProgram(program_);
    for (GLuint shader : shaders_)
      gl_->glDeleteShader(shader);
    shaders_.clear();

    return;
  }

  // Detach shaders after a successful link
  for (GLuint shader : shaders_)
    gl_->glDetachShader(program_, shader);
}

GLuint Shader::shaderTypeToGlType(ShaderType type)
{
  switch (type)
  {
    case ShaderType::Vertex:
      return GL_VERTEX_SHADER;
    case ShaderType::Fragment:
      return GL_FRAGMENT_SHADER;
  }
}

void Shader::start()
{
  gl_->glUseProgram(program_);
}

void Shader::end()
{
  gl_->glUseProgram(0);
}

void Shader::uniform(GLint location, int v)
{
  gl_->glUniform1i(location, v);
}

void Shader::uniform(GLint location, float v)
{
  gl_->glUniform1f(location, v);
}

void Shader::uniform(GLint location, const Eigen::Vector3f& v)
{
  gl_->glUniform3fv(location, 1, v.data());
}

void Shader::uniformMatrix4f(GLint location, const Eigen::Matrix4f& matrix)
{
  gl_->glUniformMatrix4fv(location, 1, GL_FALSE, matrix.data());
}

GLint Shader::getUniformLocation(const std::string& name)
{
  return gl_->glGetUniformLocation(program_, name.c_str());
}

void Shader::bindAttribLocation(GLuint index, const std::string& name)
{
  gl_->glBindAttribLocation(program_, index, name.c_str());
}
}