#version 330

in  vec4 color;
out vec4 fColor;

void
main()
{
    fColor[0] = color[0];
	fColor[1] = color[1];
	fColor[2] = color[2];
	fColor[3] = color[3];
}
