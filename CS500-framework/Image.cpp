#include "Image.h"
#include "stb_image.h"
Image::Image()
{

}

unsigned char* Image::Load_Image(std::string path, int& w, int& h, bool isFlip)
{
		if (!color_datas.empty())
		{
			color_datas.clear();
		}
		stbi_set_flip_vertically_on_load(isFlip);
		
		unsigned char* data = stbi_load(path.c_str(), &image_w, &image_h, &image_channel, color_channel);
		if (data)
		{
			pixel_size = image_w * image_h * color_channel;

			for (size_t i = 0; i < pixel_size; i += 4)
			{
				color_data.r = data[i];
				color_data.g = data[i + 1];
				color_data.b = data[i + 2];
				color_data.a = data[i + 3];
				color_datas.push_back(color_data);
			}
			stbi_image_free(data);
			w = image_w;
			h = image_h;

			return &color_datas[0].r;
		}
		else
		{
			stbi_image_free(data);
			return nullptr;
		}
}
