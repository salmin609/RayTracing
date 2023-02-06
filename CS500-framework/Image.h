#pragma once


#include <vector>
#include <string>

class Image
{
public:
	Image();
	unsigned char* Load_Image(std::string path, int& w, int &h, bool isFlip);

	struct Color_Ub {
		unsigned char r;
		unsigned char g;
		unsigned char b;
		unsigned char a;
	};
	Color_Ub GetValue(int pixU, int pixV)
	{
		return color_datas[(pixV * image_w) + pixU];
	}

private:
	

	std::vector<Color_Ub> color_datas;
	Color_Ub color_data;
	unsigned color_channel = 4;
	int image_w;
	int image_h;
	int image_channel;
	int pixel_size;
};