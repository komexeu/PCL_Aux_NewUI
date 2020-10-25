#pragma once
#include <qspinbox.h>

class my_spinBox :public QSpinBox
{
public:
	my_spinBox(QWidget *parent, QString name);
	~my_spinBox() {}
	void SetSliderStylesheet_default(const QColor &c);
};

