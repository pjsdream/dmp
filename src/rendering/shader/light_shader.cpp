#include <dmp/rendering/shader/light_shader.h>

namespace dmp
{
LightShader::LightShader(const std::shared_ptr<GlFunctions>& gl)
    : Shader(gl)
{
  // TODO: define PROJECT_SOURCE_DIR in a common header file
  const static std::string PROJECT_SOURCE_DIR = "/home/jaesungp/cpp_workspace/dmp";
  loadShader(PROJECT_SOURCE_DIR + "/shader/light.vert", ShaderType::Vertex);
  loadShader(PROJECT_SOURCE_DIR + "/shader/light.frag", ShaderType::Fragment);

  bindAttribLocations();

  linkShader();

  setUniformLocations();
}

void LightShader::setUniformLocations()
{
  loc_model_ = getUniformLocation("model");
  loc_view_ = getUniformLocation("view");
  loc_projection_ = getUniformLocation("projection");
}

void LightShader::bindAttribLocations()
{
  bindAttribLocation(0, "position");
  bindAttribLocation(1, "normal");
  bindAttribLocation(2, "tex_coord");
}

void LightShader::loadModel(const Eigen::Affine3d& model)
{
  uniform(loc_model_, model.cast<float>().matrix());
}

void LightShader::loadView(const Eigen::Affine3d& view)
{
  uniform(loc_view_, view.cast<float>().matrix());
}

void LightShader::loadProjection(const Eigen::Affine3d& projection)
{
  uniform(loc_projection_, projection.cast<float>().matrix());
}
}