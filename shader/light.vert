#version 410 core

in vec3 position;
in vec3 normal;
in vec2 tex_coord;
in vec3 color;

uniform int has_texture;
uniform int has_color;
uniform int has_global_color;
uniform vec4 global_color;

uniform mat4 model;
uniform mat4 projection;
uniform mat4 view;

out vec2 texture_coord;
out vec3 surface_color;

void main()
{
  vec4 world_position = model * vec4(position, 1.0);

  if (has_global_color)
    surface_color = global_color;
  else if (has_color)
    surface_color = color;
  else
    surface_color = vec4(0.5, 0.5, 0.5, 1);

  texture_coord = tex_coord;

  gl_Position = projection * view * world_position;
}
