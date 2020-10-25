#include "my_slider.h"

my_slider::my_slider(QWidget *parent) :QSlider(parent)
{
	this->setFocusPolicy(Qt::NoFocus);
	this->setOrientation(Qt::Orientation::Horizontal);
	this->setRange(1, 400);
}

void my_slider::SetSliderStylesheet_default(const QColor &sub_pageColor, const QColor &add_pageColor, const QColor &handlerColor) {
	QString sub_page_color = QString("background-color: rgb(%1, %2, %3);")
		.arg(sub_pageColor.red()).arg(sub_pageColor.green()).arg(sub_pageColor.blue());
	QString add_page_color = QString("background-color: rgb(%1, %2, %3);")
		.arg(add_pageColor.red()).arg(add_pageColor.green()).arg(add_pageColor.blue());
	QString handler_color = QString("background-color: rgb(%1, %2, %3);")
		.arg(handlerColor.red()).arg(handlerColor.green()).arg(handlerColor.blue());
	this->setStyleSheet(QString(
		"QSlider::sub-page:horizontal {" + sub_page_color + "}"
		"QSlider::add-page:horizontal {" + add_page_color + "}"
		"QSlider::groove:horizontal { background: transparent; height:4px; }"
		"QSlider::handle:horizontal { width:10px; border-radius:5px;"
		+ handler_color +
		"margin: -5px 0px -5px 0px; }"
	));
}

my_slider::~my_slider()
{

}
