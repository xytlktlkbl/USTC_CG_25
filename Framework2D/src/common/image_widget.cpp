#include "common/image_widget.h"

#include <stdexcept>

#define STB_IMAGE_IMPLEMENTATION
#include "iostream"
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

namespace USTC_CG
{
ImageWidget::ImageWidget(const std::string& label, const std::string& filename)
    : filename_(filename),
      Widget(label)
{
    glGenTextures(1, &tex_id_);
    auto image_data =
        stbi_load(filename.c_str(), &image_width_, &image_height_, NULL, 4);
    if (image_data == nullptr)
    {
        std::cout << "Failed to load image from file " << filename << std::endl;
        data_ = std::make_shared<Image>(image_width_, image_height_, 4);
    }
    else
    {
        std::cout << "Successfully load image from file " << filename
                  << std::endl;
        std::unique_ptr<unsigned char[]> tmp(image_data);
        data_ = std::make_shared<Image>(
            image_width_, image_height_, 4, std::move(tmp));
    }
    load_gltexture();
}

ImageWidget::~ImageWidget()
{
    glDeleteTextures(1, &tex_id_);
}

void ImageWidget::draw()
{
    draw_image();
}

void ImageWidget::set_position(const ImVec2& pos)
{
    position_ = pos;
}

ImVec2 ImageWidget::get_image_size() const
{
    return ImVec2((float)image_width_, (float)image_height_);
}

void ImageWidget::update()
{
    load_gltexture();
}

void ImageWidget::save_to_disk(const std::string& filename)
{
    if (data_)
    {
        stbi_write_png(
            filename.c_str(),
            data_->width(),
            data_->height(),
            data_->channels(),
            data_->data(),
            data_->width() * data_->channels());
    }
}

void ImageWidget::load_gltexture()
{
    glBindTexture(GL_TEXTURE_2D, tex_id_);

    // Setup filtering parameters for display
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // Upload pixels into texture (different type of channels)
    if (data_->channels() == 3)
    {
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexImage2D(
            GL_TEXTURE_2D,
            0,
            GL_RGB,
            image_width_,
            image_height_,
            0,
            GL_RGB,
            GL_UNSIGNED_BYTE,
            data_->data());
        glPixelStorei(GL_UNPACK_ALIGNMENT, 4); 
    }
    else if (data_->channels() == 4)
    {
        glTexImage2D(
            GL_TEXTURE_2D,
            0,
            GL_RGBA,
            image_width_,
            image_height_,
            0,
            GL_RGBA,
            GL_UNSIGNED_BYTE,
            data_->data());
    }
    else
    {
        throw std::runtime_error("Unsupported number of channels");
    }
}

void ImageWidget::draw_image()
{
    auto draw_list = ImGui::GetWindowDrawList();
    if (data_)
    {
        ImVec2 p_min = position_;
        ImVec2 p_max = ImVec2(p_min.x + image_width_, p_min.y + image_height_);
        draw_list->AddImage((intptr_t)tex_id_, p_min, p_max);
    }
}
}  // namespace USTC_CG