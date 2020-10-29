#pragma once
#include <qpropertyanimation.h>
#include <qtoolbutton.h>
#include "../../common_data.h"

class my_toolButton :public QToolButton
{
	Q_OBJECT
		Q_PROPERTY(QColor color READ readColor WRITE setColor);

public:
	my_toolButton(QWidget *parent = 0, QString name = 0, QString icon_root = 0);
	void set_styleSheet_color(QColor color_mouseIn, QColor color_mouseOut);
	void setColor(QColor color);
	QColor readColor();

protected:
	void enterEvent(QEvent *);
	void leaveEvent(QEvent *);
private:
	void Add_toolButton(QString icon_root);
	void set_mouseAnime();

	 QColor mouseIn_color_ = ColorScale::Color_struct.colorB;
	 QColor mouseOut_color_ = ColorScale::Color_struct.colorC;

	QPropertyAnimation *b_ani;
	QPropertyAnimation *e_ani;

	QColor b_color;
	QColor e_color;

	
};

