// #define __GNUC__

#include "../camera.h"
#include "../light.h"
#include "nodes/core/def/node_def.hpp"
#include "pxr/base/gf/matrix4f.h"
#include "pxr/imaging/glf/simpleLight.h"
#include "pxr/imaging/hd/tokens.h"
#include "render_node_base.h"
#include "rich_type_buffer.hpp"
#include "utils/draw_fullscreen.h"
NODE_DEF_OPEN_SCOPE

NODE_DECLARATION_FUNCTION(deferred_lighting)
{
    b.add_input<TextureHandle>("Position");
    b.add_input<TextureHandle>("diffuseColor");
    b.add_input<TextureHandle>("MetallicRoughness");
    b.add_input<TextureHandle>("Normal");
    b.add_input<TextureHandle>("Shadow Maps");

    b.add_input<std::string>("Lighting Shader")
        .default_val("shaders/blinn_phong.fs");
    b.add_output<TextureHandle>("Color");
}

struct LightInfo {
    GfMatrix4f light_projection;
    GfMatrix4f light_view;
    GfVec3f position;
    float radius;
    GfVec3f luminance;
    int shadow_map_id;
};

NODE_EXECUTION_FUNCTION(deferred_lighting)
{
    // Fetch all the information

    auto position_texture = params.get_input<TextureHandle>("Position");
    auto diffuseColor_texture = params.get_input<TextureHandle>("diffuseColor");

    auto metallic_roughness =
        params.get_input<TextureHandle>("MetallicRoughness");
    auto normal_texture = params.get_input<TextureHandle>("Normal");

    auto shadow_maps = params.get_input<TextureHandle>("Shadow Maps");

    Hd_USTC_CG_Camera* free_camera = get_free_camera(params);
    // Creating output textures.
    auto size = position_texture->desc.size;
    TextureDesc color_output_desc;
    color_output_desc.format = HdFormatFloat32Vec4;
    color_output_desc.size = size;
    auto color_texture = resource_allocator.create(color_output_desc);

    unsigned int VBO, VAO;
    CreateFullScreenVAO(VAO, VBO);

    auto shaderPath = params.get_input<std::string>("Lighting Shader");

    ShaderDesc shader_desc;
    shader_desc.set_vertex_path(
        std::filesystem::path(RENDER_NODES_FILES_DIR) /
        std::filesystem::path("shaders/fullscreen.vs"));

    shader_desc.set_fragment_path(
        std::filesystem::path(RENDER_NODES_FILES_DIR) /
        std::filesystem::path(shaderPath));
    auto shader = resource_allocator.create(shader_desc);
    GLuint framebuffer;
    glGenFramebuffers(1, &framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
    glFramebufferTexture2D(
        GL_FRAMEBUFFER,
        GL_COLOR_ATTACHMENT0,
        GL_TEXTURE_2D,
        color_texture->texture_id,
        0);

    glClearColor(0.f, 0.f, 0.f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    shader->shader.use();
    shader->shader.setVec2("iResolution", size);

    shader->shader.setInt("diffuseColorSampler", 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, diffuseColor_texture->texture_id);

    shader->shader.setInt("normalMapSampler", 1);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, normal_texture->texture_id);

    shader->shader.setInt("metallicRoughnessSampler", 2);
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, metallic_roughness->texture_id);

    shader->shader.setInt("shadow_maps", 3);
    glActiveTexture(GL_TEXTURE3);
    glBindTexture(GL_TEXTURE_2D_ARRAY, shadow_maps->texture_id);

    shader->shader.setInt("position", 4);
    glActiveTexture(GL_TEXTURE4);
    glBindTexture(GL_TEXTURE_2D, position_texture->texture_id);

    GfVec3f camPos =
        GfMatrix4f(free_camera->GetTransform()).ExtractTranslation();
    shader->shader.setVec3("camPos", camPos);

    GLuint lightBuffer;
    glGenBuffers(1, &lightBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, lightBuffer);
    glViewport(0, 0, size[0], size[1]);
    std::vector<LightInfo> light_vector;

    for (int i = 0; i < lights.size(); ++i) {
        if (!lights[i]->GetId().IsEmpty()&& lights[i]->GetLightType() ==  HdPrimTypeTokens->sphereLight) {
            GlfSimpleLight light_params =
                lights[i]->Get(HdTokens->params).Get<GlfSimpleLight>();
            auto diffuse4 = light_params.GetDiffuse();
            pxr::GfVec3f diffuse3(diffuse4[0], diffuse4[1], diffuse4[2]);
            auto position4 = light_params.GetPosition();
            pxr::GfVec3f position3(position4[0], position4[1], position4[2]);

            if (lights[i]->Get(HdLightTokens->radius).IsHolding<float>()) {
                auto radius =
                    lights[i]->Get(HdLightTokens->radius).Get<float>();

                light_vector.emplace_back(
                    GfMatrix4f(), GfMatrix4f(), position3, 0.f, diffuse3, i);
            }

            // You can add directional light here, and also the corresponding
            // shadow map calculation part.
        }
    }

    shader->shader.setInt("light_count", light_vector.size());

    glBufferData(
        GL_SHADER_STORAGE_BUFFER,
        light_vector.size() * sizeof(LightInfo),
        light_vector.data(),
        GL_STATIC_DRAW);

    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, lightBuffer);

    glBindVertexArray(VAO);
    glDrawArrays(GL_TRIANGLES, 0, 6);

    DestroyFullScreenVAO(VAO, VBO);

    resource_allocator.destroy(shader);
    glDeleteBuffers(1, &lightBuffer);
    glDeleteFramebuffers(1, &framebuffer);
    params.set_output("Color", color_texture);

    auto shader_error = shader->shader.get_error();
    if (!shader_error.empty()) {
        throw std::runtime_error(shader_error);
    }
}

NODE_DECLARATION_UI(deferred_lighting);
NODE_DEF_CLOSE_SCOPE
