/// <summary>
/// 色の操作を行うためのユーティリティクラス
/// </summary>
#pragma once

// インクルード
//--------------------------------------------------------------------
#include <cstdint>

// 名前空間
//--------------------------------------------------------------------
namespace PCLWrapper
{


// 構造体定義
//--------------------------------------------------------------------
typedef struct RgbColor
{
	uint8_t r;
	uint8_t g;
	uint8_t b;

	RgbColor() { }
	RgbColor( uint8_t _r, uint8_t _g, uint8_t _b )
	{
		r = _r;
		g = _g;
		b = _b;
	}


} RgbColor;

typedef struct HsvColor
{
	uint8_t h;
	uint8_t s;
	uint8_t v;

	HsvColor() { }
	HsvColor( uint8_t _h, uint8_t _s, uint8_t _v )
	{
		h = _h;
		s = _s;
		v = _v;
	}

} HsvColor;


class PWUColor
{

private:

	PWUColor() = delete;
	~PWUColor() = delete;

	static RgbColor HsvToRgb( HsvColor & );
	static HsvColor RgbToHsv( RgbColor & );

public:
	static float* HsvToRgb( uint8_t, uint8_t, uint8_t );

};


// 名前空間
//--------------------------------------------------------------------
} // PCLWrapper