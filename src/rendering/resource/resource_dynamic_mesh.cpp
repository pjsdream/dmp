#include <dmp/rendering/resource/resource_dynamic_mesh.h>

namespace dmp
{
ResourceDynamicMesh::ResourceDynamicMesh(const std::shared_ptr<GlFunctions>& gl)
    : Resource(gl), vao_(0)
{
}

ResourceDynamicMesh::~ResourceDynamicMesh()
{
  // TODO: clear gl objects
}

void ResourceDynamicMesh::updateVertexBuffer(std::vector<float>&& b)
{
  gl_->glBindVertexArray(vao_);
  gl_->glBindBuffer(GL_ARRAY_BUFFER, vbos_[0]);
  gl_->glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float) * b.size(), (void*)&b[0]);
  gl_->glBindBuffer(GL_ARRAY_BUFFER, vbos_[0]);
}
void ResourceDynamicMesh::updateNormalBuffer(std::vector<float>&& b)
{
  gl_->glBindVertexArray(vao_);
  gl_->glBindBuffer(GL_ARRAY_BUFFER, vbos_[1]);
  gl_->glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float) * b.size(), (void*)&b[0]);
  gl_->glBindBuffer(GL_ARRAY_BUFFER, vbos_[1]);
}
void ResourceDynamicMesh::updateTextureBuffer(std::vector<float>&& b)
{
  gl_->glBindVertexArray(vao_);
  gl_->glBindBuffer(GL_ARRAY_BUFFER, vbos_[2]);
  gl_->glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float) * b.size(), (void*)&b[0]);
  gl_->glBindBuffer(GL_ARRAY_BUFFER, vbos_[2]);
}
void ResourceDynamicMesh::updateColorBuffer(std::vector<float>&& b)
{
  gl_->glBindVertexArray(vao_);
  gl_->glBindBuffer(GL_ARRAY_BUFFER, vbos_[3]);
  gl_->glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float) * b.size(), (void*)&b[0]);
  gl_->glBindBuffer(GL_ARRAY_BUFFER, vbos_[3]);
}
void ResourceDynamicMesh::updateFaceBuffer(std::vector<int>&& b)
{
  gl_->glBindVertexArray(vao_);
  gl_->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbos_[4]);
  gl_->glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, sizeof(float) * b.size(), (void*)&b[0]);
  gl_->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbos_[4]);
}

void ResourceDynamicMesh::draw()
{
  if (vao_ == 0)
    allocateGlBuffers();
}

void ResourceDynamicMesh::allocateGlBuffers()
{
  GLuint vbo;

  gl_->glGenVertexArrays(1, &vao_);
  gl_->glGenBuffers(5, &vbos_[0]);

  gl_->glBindVertexArray(vao_);

  gl_->glBindBuffer(GL_ARRAY_BUFFER, vbos_[0]);
  gl_->glBufferData(GL_ARRAY_BUFFER, INITIAL_BUFFER_ELEMENTS, 0, GL_DYNAMIC_DRAW);
  gl_->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
  gl_->glEnableVertexAttribArray(0);

  gl_->glBindBuffer(GL_ARRAY_BUFFER, vbos_[1]);
  gl_->glBufferData(GL_ARRAY_BUFFER, INITIAL_BUFFER_ELEMENTS, 0, GL_DYNAMIC_DRAW);
  gl_->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
  gl_->glEnableVertexAttribArray(1);

  gl_->glBindBuffer(GL_ARRAY_BUFFER, vbos_[2]);
  gl_->glBufferData(GL_ARRAY_BUFFER, INITIAL_BUFFER_ELEMENTS, 0, GL_DYNAMIC_DRAW);
  gl_->glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, 0);
  gl_->glEnableVertexAttribArray(2);

  gl_->glBindBuffer(GL_ARRAY_BUFFER, vbos_[3]);
  gl_->glBufferData(GL_ARRAY_BUFFER, INITIAL_BUFFER_ELEMENTS, 0, GL_DYNAMIC_DRAW);
  gl_->glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 0, 0);
  gl_->glEnableVertexAttribArray(3);

  gl_->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbos_[4]);
  gl_->glBufferData(GL_ELEMENT_ARRAY_BUFFER, INITIAL_BUFFER_ELEMENTS, 0, GL_DYNAMIC_DRAW);

  gl_->glBindVertexArray(0);
  gl_->glBindBuffer(GL_ARRAY_BUFFER, 0);
  gl_->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}
}
