#version 430

layout(location = 0) out vec3 position;
layout(location = 1) out float depth;
layout(location = 2) out vec2 texcoords;
layout(location = 3) out vec3 diffuseColor;
layout(location = 4) out vec2 metallicRoughness;
layout(location = 5) out vec3 normal;

in vec3 vertexPosition;
in vec3 vertexNormal;
in vec2 vTexcoord;
uniform mat4 projection;
uniform mat4 view;

uniform sampler2D diffuseColorSampler;

// This only works for current scenes provided by the TAs 
// because the scenes we provide is transformed from gltf
uniform sampler2D normalMapSampler;
uniform sampler2D metallicRoughnessSampler;

void main() {
    position = vertexPosition;
    vec4 clipPos = projection * view * (vec4(position, 1.0));
    depth = clipPos.z / clipPos.w;
    texcoords = vTexcoord;

    diffuseColor = texture2D(diffuseColorSampler, vTexcoord).xyz;
    metallicRoughness = texture2D(metallicRoughnessSampler, vTexcoord).zy;

    vec3 normalmap_value = texture2D(normalMapSampler, vTexcoord).xyz;
    normal = normalize(vertexNormal);

    // HW6_TODO: Apply normal map here. Use normal textures to modify vertex normals.

    // Calculate tangent and bitangent
    vec3 edge1 = dFdx(vertexPosition);
    vec3 edge2 = dFdy(vertexPosition);
    vec2 deltaUV1 = dFdx(vTexcoord);
    vec2 deltaUV2 = dFdy(vTexcoord);

    vec3 tangent = edge1 * deltaUV2.y - edge2 * deltaUV1.y;

    // Robust tangent and bitangent evaluation
    if(length(tangent) < 1E-7) {
        vec3 bitangent = -edge1 * deltaUV2.x + edge2 * deltaUV1.x;
        tangent = normalize(cross(bitangent, normal));
    }
    tangent = normalize(tangent - dot(tangent, normal) * normal);
    vec3 bitangent = normalize(cross(tangent,normal));
}