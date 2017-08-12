#include <dmp/rendering/resource/resource_mesh.h>
#include <dmp/rendering/resource/resource_texture.h>

#include <assimp/scene.h>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>

namespace dmp
{
struct ResourceMesh::RawMesh
{
  std::vector<float> vertex_buffer;
  std::vector<float> normal_buffer;
  std::vector<float> texture_buffer;
  std::vector<float> color_buffer;
  std::vector<int> face_buffer;

  RawMesh(RawMesh&& rhs) = default;
};

ResourceMesh::ResourceMesh(const std::shared_ptr<GlFunctions>& gl, const std::string& filename)
    : Resource(gl),
      ready_rendering_(false),
      color_option_(ColorOption::GlobalColor),
      global_color_(Eigen::Vector3f(0.8f, 0.8f, 0.8f))
{
  future_raw_mesh_ = std::async(std::launch::async,
                                [=]()
                                { return asyncLoadMesh(filename); }
  );
}

ResourceMesh::~ResourceMesh()
{
  // TODO: clear gl objects
}

ResourceMesh::RawMesh ResourceMesh::asyncLoadMesh(std::string filename)
{
  RawMesh raw_mesh{};

  Assimp::Importer assimp_importer{};
  const aiScene* scene =
      assimp_importer.ReadFile(filename, aiProcess_Triangulate);

  if (scene != 0)
  {
    aiMesh* mesh = scene->mMeshes[0];

    raw_mesh.vertex_buffer.reserve(mesh->mNumVertices);
    std::copy((float*) mesh->mVertices,
              (float*) (mesh->mVertices + mesh->mNumVertices),
              std::back_inserter(raw_mesh.vertex_buffer));

    raw_mesh.normal_buffer.reserve(mesh->mNumVertices);
    std::copy((float*) mesh->mNormals,
              (float*) (mesh->mNormals + mesh->mNumVertices),
              std::back_inserter(raw_mesh.normal_buffer));

    if (mesh->HasTextureCoords(0))
    {
      raw_mesh.texture_buffer.resize(mesh->mNumVertices * 2);
      for (int i = 0; i < mesh->mNumVertices; i++)
      {
        raw_mesh.texture_buffer[i * 2 + 0] =
            mesh->mTextureCoords[0][i][0];
        raw_mesh.texture_buffer[i * 2 + 1] =
            mesh->mTextureCoords[0][i][1];
      }
    }

    raw_mesh.face_buffer.resize(mesh->mNumFaces * 3);
    for (int i = 0; i < mesh->mNumFaces; i++)
    {
      raw_mesh.face_buffer[i * 3 + 0] = mesh->mFaces[i].mIndices[0];
      raw_mesh.face_buffer[i * 3 + 1] = mesh->mFaces[i].mIndices[1];
      raw_mesh.face_buffer[i * 3 + 2] = mesh->mFaces[i].mIndices[2];
    }
  }

  return raw_mesh;
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
    color_option_ = ColorOption::Texture;

    gl_->glBindBuffer(GL_ARRAY_BUFFER, vbos_[2]);
    gl_->glBufferData(GL_ARRAY_BUFFER,
                      sizeof(float) * raw_mesh.texture_buffer.size(),
                      &raw_mesh.texture_buffer[0],
                      GL_STATIC_DRAW);
    gl_->glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, (void*) 0);
    gl_->glEnableVertexAttribArray(2);
  }

  if (!raw_mesh.color_buffer.empty())
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