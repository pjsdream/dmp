#ifndef DMP_LIGHT_SHADER_H
#define DMP_LIGHT_SHADER_H

#include <dmp/rendering/shader/shader.h>

namespace dmp
{
class LightShader : public Shader
{
public:
  LightShader(const std::shared_ptr<GlBase>& base);
};
}

#endif //DMP_LIGHT_SHADER_H
