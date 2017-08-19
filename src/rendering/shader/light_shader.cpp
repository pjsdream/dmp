#include <dmp/rendering/shader/light_shader.h>
#include <dmp/rendering/camera/camera.h>
#include <dmp/rendering/light/light.h>
#include <dmp/rendering/material/material.h>
#include <dmp/rendering/resource/resource_texture.h>

namespace dmp
{
LightShader::LightShader(const std::shared_ptr<GlFunctions>& gl)
    : Shader(gl)
{
  // TODO: define PROJECT_SOURCE_DIR in a common header file
  const static std::string PROJECT_SOURCE_DIR = "/Users/jaesungp/cpp_workspace/dmp";
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

  loc_eye_position_ = getUniformLocation("eye_position");

  loc_has_texture_ = getUniformLocation("has_texture");
  loc_has_color_ = getUniformLocation("has_color");
  loc_has_global_color_ = getUniformLocation("has_global_color");
  loc_global_color_ = getUniformLocation("global_color");
  loc_has_material_ = getUniformLocation("has_material");

  for (int i=0; i<NUM_LIGHTS; i++)
  {
    const std::string varname = std::string("lights[") + std::to_string(i) + "]";
    loc_lights_[i].use = getUniformLocation(varname + ".use");
    loc_lights_[i].type = getUniformLocation(varname + ".type");
    loc_lights_[i].position = getUniformLocation(varname + ".position");
    loc_lights_[i].ambient = getUniformLocation(varname + ".ambient");
    loc_lights_[i].diffuse = getUniformLocation(varname + ".diffuse");
    loc_lights_[i].specular = getUniformLocation(varname + ".specular");
    loc_lights_[i].attenuation = getUniformLocation(varname + ".attenuation");
  }

  loc_material_.ambient = getUniformLocation("material.ambient");
  loc_material_.diffuse = getUniformLocation("material.diffuse");
  loc_material_.specular = getUniformLocation("material.specular");
  loc_material_.shininess = getUniformLocation("material.shininess");

  loc_material_texture_ = getUniformLocation("material_texture");
}

void LightShader::bindAttribLocations()
{
  bindAttribLocation(0, "position");
  bindAttribLocation(1, "normal");
  bindAttribLocation(2, "tex_coord");
  bindAttribLocation(3, "color");
}

void LightShader::hasTexture(const std::shared_ptr<ResourceTexture>& texture)
{
  if (color_option_ != ColorOption::Texture)
  {
    color_option_ = ColorOption::Texture;
    uniform(loc_has_texture_, 1);
    uniform(loc_has_color_, 0);
    uniform(loc_has_global_color_, 0);
    uniform(loc_has_material_, 0);
  }
  texture->bind();
  uniform(loc_material_texture_, 0);
}
void LightShader::hasColor()
{
  if (color_option_ != ColorOption::Color)
  {
    color_option_ = ColorOption::Color;
    uniform(loc_has_texture_, 0);
    uniform(loc_has_color_, 1);
    uniform(loc_has_global_color_, 0);
    uniform(loc_has_material_, 0);
  }
}
void LightShader::hasGlobalColor(const Eigen::Vector3f& color)
{
  if (color_option_ != ColorOption::GlobalColor)
  {
    color_option_ = ColorOption::GlobalColor;
    uniform(loc_has_texture_, 0);
    uniform(loc_has_color_, 0);
    uniform(loc_has_global_color_, 1);
    uniform(loc_has_material_, 0);
  }
  uniform(loc_global_color_, color);
}

void LightShader::loadModel(const Eigen::Affine3d& model)
{
  uniformMatrix4f(loc_model_, model.cast<float>().matrix());
}

void LightShader::loadView(const Eigen::Affine3d& view)
{
  uniformMatrix4f(loc_view_, view.cast<float>().matrix());
}

void LightShader::loadView(const Eigen::Matrix4d& view)
{
  uniformMatrix4f(loc_view_, view.cast<float>());
}

void LightShader::loadProjection(const Eigen::Affine3d& projection)
{
  uniformMatrix4f(loc_projection_, projection.cast<float>().matrix());
}

void LightShader::loadProjection(const Eigen::Matrix4d& projection)
{
  uniformMatrix4f(loc_projection_, projection.cast<float>());
}

void LightShader::loadCamera(const std::shared_ptr<Camera>& camera)
{
  uniform(loc_eye_position_, camera->eyePosition().cast<float>());
  loadView(camera->viewMatrix());
  loadProjection(camera->projectionMatrix());
}

void LightShader::loadLight(int index, const std::shared_ptr<Light>& light)
{
  if (index >= 0 && index < NUM_LIGHTS)
  {
    uniform(loc_lights_[index].use, 1);
    uniform(loc_lights_[index].position, light->position);
    uniform(loc_lights_[index].ambient, light->ambient);
    uniform(loc_lights_[index].diffuse, light->diffuse);
    uniform(loc_lights_[index].specular, light->specular);

    switch (light->type)
    {
      case Light::LightType::Directional:
        uniform(loc_lights_[index].type, 0);
        break;
      case Light::LightType::Point:
        uniform(loc_lights_[index].type, 1);
        uniform(loc_lights_[index].attenuation, light->attenuation);
        break;
      default:
        uniform(loc_lights_[index].type, 1);
    }
  }
}

void LightShader::loadMaterial(const std::shared_ptr<Material>& material)
{
  uniform(loc_material_.ambient, material->ambient);
  uniform(loc_material_.diffuse, material->diffuse);
  uniform(loc_material_.specular, material->specular);
  uniform(loc_material_.shininess, material->shininess);
}
}
