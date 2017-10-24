#include <renderer/resource/mesh_loader.h>
#include <renderer/resource/raw_mesh.h>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/mesh.h>
#include <assimp/scene.h>

namespace dmp
{
std::future<RawMesh> MeshLoader::loadMesh(const std::string& filename)
{
  return std::async(std::launch::async, [filename](){
    RawMesh raw_mesh;

    Assimp::Importer assimp_importer{};
    const aiScene* scene =
        assimp_importer.ReadFile(filename, aiProcess_Triangulate | aiProcess_GenSmoothNormals);

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
          raw_mesh.texture_buffer[i * 2 + 0] = mesh->mTextureCoords[0][i][0];
          raw_mesh.texture_buffer[i * 2 + 1] = 1 - mesh->mTextureCoords[0][i][1];
        }

        auto material = scene->mMaterials[0];
        aiString path;
        material->GetTexture(aiTextureType_DIFFUSE, 0, &path);

        raw_mesh.texture_filename = getDirectory(filename) + "/" + path.C_Str();
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
  });
}

std::string MeshLoader::getDirectory(const std::string filename)
{
  return filename.substr(0, filename.find_last_of('/'));
}
}
