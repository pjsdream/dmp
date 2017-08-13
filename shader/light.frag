#version 410 core

struct DirectionalLight
{
  bool use;

  vec3 position;

  vec3 ambient;
  vec3 diffuse;
  vec3 specular;
};

struct Material
{
  vec3 ambient;
  vec3 diffuse;
  vec3 specular;

  sampler2D texture;

  float shininess;
};

uniform vec3 eye_position;
uniform DirectionalLight directional_lights[8];

uniform bool has_material;
uniform bool has_texture;
uniform Material material;

in vec2 texture_coord;
in vec3 surface_position;
in vec3 surface_normal;
in vec3 surface_color;

out vec4 out_color;

void main()
{
  vec3 material_final_ambient = vec3(0.f, 0.f, 0.f);
  vec3 material_final_diffuse = surface_color;
  vec3 material_final_specular = vec3(1.f, 1.f, 1.f) * 0.1f;
  float material_final_shininess = 10.f;

  if (has_material)
  {
    material_final_ambient = material.ambient;
    material_final_diffuse = material.diffuse;
    material_final_specular = material.specular;
    material_final_shininess = material.shininess;
  }

  if (has_texture)
  {
    material_final_diffuse = texture(material.texture, texture_coord).rgb;
    material_final_ambient = material_final_diffuse * 0.2;
  }

  vec3 N = normalize(surface_normal);
  vec3 V = normalize(eye_position - surface_position);

  vec3 total_color = vec3(0.f, 0.f, 0.f);

  for (int i=0; i<8; i++)
  {
    if (directional_lights[i].use)
    {
      vec3 L = normalize(directional_lights[i].position);

      vec3 R = - L + 2.0 * dot(L, N) * N;

      float NdotL = dot(N, L);
      float VdotR = dot(V, R);

      float diffuse_strength = clamp(NdotL, 0.f, 1.f);
      float specular_strength = pow( clamp(VdotR, 0.f, 1.f), material_final_shininess );

      vec3 ambient = directional_lights[i].ambient * material_final_ambient;
      vec3 diffuse = directional_lights[i].diffuse * material_final_diffuse * diffuse_strength;
      vec3 specular = directional_lights[i].specular * material_final_specular * specular_strength;

      total_color += ambient + diffuse + specular;
    }
  }

  out_color = vec4(total_color, 1.0);
}
