#pragma once
#include <qpushbutton.h>
#include <qpropertyanimation.h>
#include "common_data.h"

class my_button :public QPushButton
{
	Q_OBJECT
		Q_PROPERTY(QColor color READ readColor WRITE setColor);

public:
	my_button(QWidget *parent = 0, QString name = 0);
	void set_styleSheet_color(QColor color_mouseIn, QColor color_mouseOut);
	void set_font_color(QColor color);

	void setColor(QColor color);
	QColor readColor();

protected:
	void enterEvent(QEvent *);
	void leaveEvent(QEvent *);
private:
	void set_mouseAnime();
	
	QColor mouseIn_color_ = ColorScale::Color_struct.colorB;
	QColor mouseOut_color_ = ColorScale::Color_struct.colorC;

	QPropertyAnimation *b_ani;
	QPropertyAnimation *e_ani;

	QColor b_color;
	QColor e_color;
};

