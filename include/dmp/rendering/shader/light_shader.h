#ifndef DMP_LIGHT_SHADER_H
#define DMP_LIGHT_SHADER_H

#include <dmp/rendering/shader/shader.h>

namespace dmp
{
class LightShader : public Shader
{
public:
  LightShader(const std::shared_ptr<GlFunctions>& gl);

  void loadModel(const Eigen::Affine3d& model);
  void loadView(const Eigen::Affine3d& view);
  void loadProjection(const Eigen::Affine3d& projection);

private:
  void setUniformLocations();
  void bindAttribLocations();

  GLint loc_model_;
  GLint loc_projection_;
  GLint loc_view_;
};
}

#endif //DMP_LIGHT_SHADER_H
