#version 330
uniform mat4 WorldView, WorldProj, ModelTr;
uniform mat3 normalTR;
in vec4 vertex;
in vec3 vertexNormal;
in vec2 vertexTexture;

out vec3 normalVec, worldPos;
out vec2 texCoord;

void main()
{
    worldPos = (ModelTr*vertex).xyz;
    normalVec = vertexNormal*normalTR;
    texCoord = vertexTexture;
    gl_Position = WorldProj*WorldView*ModelTr*vertex;
}
