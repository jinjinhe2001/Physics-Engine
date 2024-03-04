#version 410 core

layout (location = 0) in vec3 a_Position;

out vec3 TexCoords;

uniform mat4 projection;
uniform mat4 view;

void main()
{
	TexCoords = a_Position;
	gl_Position = projection * mat4(mat3(view)) * vec4(a_Position, 1.0f);
}