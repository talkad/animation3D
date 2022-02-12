#version 330 core

out vec4 FragColor;

in float Height;

void main()
{
    float h = (Height + 16)/32.0f;	// shift and scale the height in to a grayscale value
    FragColor = vec4(2.4 * h, 2.4 * h, 1.3 * h, 1.0);
}