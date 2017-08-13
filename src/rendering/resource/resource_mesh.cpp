#include <dmp/rendering/resource/resource_mesh.h>
#include <dmp/rendering/resource/resource_texture.h>
#include <dmp/utils/mesh_loader.h>
#include <dmp/utils/texture_loader.h>

namespace dmp
{
ResourceMesh::ResourceMesh(const std::shared_ptr<GlFunctions>& gl, const std::string& filename)
    : Resource(gl),
      ready_rendering_(false),
      color_option_(ColorOption::GlobalColor),
      global_color_(Eigen::Vector3f(0.8f, 0.8f, 0.8f))
{
  future_raw_mesh_ = MeshLoader::loadMesh(filename);
}

ResourceMesh::~ResourceMesh()
{
  // TODO: clear gl objects
}

void ResourceMesh::draw()
{
  using namespace std::literals;

  if (!ready_rendering_)
  {
    if (future_raw_mesh_.valid() &&
        future_raw_mesh_.wait_for(0s) == std::future_status::ready)
    {
      prepareGlBuffers();
      ready_rendering_ = true;
    }
  }

  if (!ready_texture_)
  {
    if (future_texture_.valid() &&
        future_texture_.wait_for(0s) == std::future_status::ready)
    {
      color_option_ = ColorOption::Texture;
      texture_ = std::make_shared<ResourceTexture>(gl_);
      texture_->loadTexture(future_texture_.get());
      ready_texture_ = true;
    }
  }

  if (ready_rendering_)
  {
    gl_->glBindVertexArray(vao_);
    gl_->glDrawElements(GL_TRIANGLES, num_faces_ * 3, GL_UNSIGNED_INT, 0);
    gl_->glBindVertexArray(0);
  }
}

void ResourceMesh::prepareGlBuffers()
{
  auto raw_mesh = future_raw_mesh_.get();

  gl_->glGenVertexArrays(1, &vao_);

  vbos_.resize(5);
  gl_->glGenBuffers(5, &vbos_[0]);

  gl_->glBindVertexArray(vao_);
  gl_->glBindBuffer(GL_ARRAY_BUFFER, vbos_[0]);
  gl_->glBufferData(GL_ARRAY_BUFFER,
                    sizeof(float) * raw_mesh.vertex_buffer.size(),
                    &raw_mesh.vertex_buffer[0],
                    GL_STATIC_DRAW);
  gl_->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*) 0);
  gl_->glEnableVertexAttribArray(0);

  gl_->glBindBuffer(GL_ARRAY_BUFFER, vbos_[1]);
  gl_->glBufferData(GL_ARRAY_BUFFER,
                    sizeof(float) * raw_mesh.normal_buffer.size(),
                    &raw_mesh.normal_buffer[0],
                    GL_STATIC_DRAW);
  gl_->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*) 0);
  gl_->glEnableVertexAttribArray(1);

  if (!raw_mesh.texture_buffer.empty())
  {
    // will be changed to texture when it is loaded
    color_option_ = ColorOption::GlobalColor;

    gl_->glBindBuffer(GL_ARRAY_BUFFER, vbos_[2]);
    gl_->glBufferData(GL_ARRAY_BUFFER,
                      sizeof(float) * raw_mesh.texture_buffer.size(),
                      &raw_mesh.texture_buffer[0],
                      GL_STATIC_DRAW);
    gl_->glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, (void*) 0);
    gl_->glEnableVertexAttribArray(2);

    // load texture
    future_texture_ = TextureLoader::asyncLoadTexture(raw_mesh.texture_filename);
  }

  else if (!raw_mesh.color_buffer.empty())
  {
    // TODO: mesh that has both texture and color
    color_option_ = ColorOption::Color;

    gl_->glBindBuffer(GL_ARRAY_BUFFER, vbos_[3]);
    gl_->glBufferData(GL_ARRAY_BUFFER,
                      sizeof(float) * raw_mesh.color_buffer.size(),
                      &raw_mesh.color_buffer[0],
                      GL_STATIC_DRAW);
    gl_->glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 0, (void*) 0);
    gl_->glEnableVertexAttribArray(3);
  }

  else
  {
    color_option_ = ColorOption::GlobalColor;
  }

  gl_->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbos_[4]);
  gl_->glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                    sizeof(int) * raw_mesh.face_buffer.size(),
                    &raw_mesh.face_buffer[0],
                    GL_STATIC_DRAW);

  gl_->glBindVertexArray(0);
  gl_->glBindBuffer(GL_ARRAY_BUFFER, 0);
  gl_->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

  num_faces_ = raw_mesh.face_buffer.size() / 3;
}

bool ResourceMesh::hasTexture()
{
  return color_option_ == ColorOption::Texture;
}
std::shared_ptr<ResourceTexture> ResourceMesh::getTexture()
{
  return texture_;
}

bool ResourceMesh::hasColor()
{
  return color_option_ == ColorOption::Color;
}

bool ResourceMesh::hasGlobalColor()
{
  return color_option_ == ColorOption::GlobalColor;
}

const Eigen::Vector3f& ResourceMesh::getGlobalColor()
{
  return global_color_;
}
}
