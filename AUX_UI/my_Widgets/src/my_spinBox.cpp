#include <my_spinBox.h>


my_spinBox::my_spinBox(QWidget *parent, QString name) :QSpinBox(parent)
{
	this->setObjectName(name);
	//this->setFocusPolicy(Qt::NoFocus);
	this->setRange(1, 400);
}

void my_spinBox::SetSliderStylesheet_default(const QColor &c) {
	this->setStyleSheet(QString(
		"QSpinBox {border-radius: 3px;}"
		"QSpinBox {background-color: rgb(255, 255, 255);}"
		"QSpinBox::up-button {background-color: rgb(%1, %2, %3);}"
		"QSpinBox::up-arrow {image:url(:/AUX_UI/my_source/Triangle4.png);width:10px;height:10px; }"
		"QSpinBox::down-button {background-color: rgb(%1, %2, %3);}"
		"QSpinBox::down-arrow {image:url(:/AUX_UI/my_source/Triangle5.png);width:10px;height:10px; }"
	).arg(c.red()).arg(c.green()).arg(c.blue()));
}