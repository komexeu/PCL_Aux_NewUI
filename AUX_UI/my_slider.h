#pragma once
#include <qslider.h>
#include "common_data.h"

class my_slider :public QSlider
{
public:
	my_slider(QWidget *parent);
	~my_slider();
	void SetSliderStylesheet_default(const QColor &sub_page_color, const QColor &add_page_color,const QColor &handler_color);
};

