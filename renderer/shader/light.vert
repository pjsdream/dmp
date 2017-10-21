#version 430 core
#extension GL_ARB_explicit_uniform_location : enable

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec2 tex_coord;
layout(location = 3) in vec3 color;

layout(location = 0) uniform bool has_color;
layout(location = 1) uniform bool has_global_color;
layout(location = 2) uniform vec3 global_color;

layout(location = 3) uniform mat4 model;
layout(location = 4) uniform mat4 projection;
layout(location = 5) uniform mat4 view;

out vec2 texture_coord;
out vec3 surface_position;
out vec3 surface_normal;
out vec3 surface_color;

void main()
{
  if (has_global_color)
    surface_color = global_color;
  else if (has_color)
    surface_color = color;
  else
    surface_color = vec3(0.5);

  surface_position = vec3(model * vec4(position, 1.0));
  surface_normal = mat3(model) * normal;

  texture_coord = tex_coord;

  gl_Position = projection * view * model * vec4(position, 1.0);
}
