#version 430 core

in vec3 position;
in vec3 normal;
in vec2 tex_coord;
in vec3 color;

uniform bool has_color;
uniform bool has_global_color;
uniform vec3 global_color;

uniform mat4 model;
uniform mat4 projection;
uniform mat4 view;

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
