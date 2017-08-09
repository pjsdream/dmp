#version 410 core

in vec3 position;
in vec3 normal;
in vec2 tex_coord;

uniform mat4 model;
uniform mat4 projection;
uniform mat4 view;

out vec3 surface_color;

void main()
{
    vec4 world_position = model * vec4(position, 1.0);
	
    surface_color = normal;

    gl_Position = projection * view * world_position;
}
