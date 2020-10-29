#include <my_toolButton.h>

my_toolButton::my_toolButton(QWidget *parent, QString name, QString icon_root)
	:QToolButton(parent) {
	this->setObjectName(name);
	this->setFocusPolicy(Qt::NoFocus);
	Add_toolButton(icon_root);
}

void my_toolButton::Add_toolButton(QString icon_root) {
	QIcon the_icon;
	the_icon.addFile(icon_root, QSize(), QIcon::Normal, QIcon::Off);
	this->setIcon(the_icon);
}

void my_toolButton::setColor(QColor color) {
	setStyleSheet(QString(
		"background-color: rgb(%1, %2, %3);"
		"font-size: 16px;"
		"border:0px;").arg(color.red()).arg(color.green()).arg(color.blue()));
	b_color = color;
	update();
}
QColor my_toolButton::readColor() {
	return b_color;
}

void my_toolButton::enterEvent(QEvent *) {
	b_ani->start();
}
void my_toolButton::leaveEvent(QEvent *) {
	e_ani->start();
}

void my_toolButton::set_styleSheet_color(QColor color_mouseIn, QColor color_mouseOut) {
	mouseIn_color_ = color_mouseIn;
	mouseOut_color_ = color_mouseOut;
	set_mouseAnime();
	this->setStyleSheet(QString(
		"background-color: rgb(%1, %2, %3);"
		"font-size: 16px;"
		"border:0px;").arg(mouseOut_color_.red()).arg(mouseOut_color_.green()).arg(mouseOut_color_.blue()));
}

void my_toolButton::set_mouseAnime() {
	b_ani = new QPropertyAnimation(this, "color");
	b_ani->setStartValue(mouseOut_color_);
	b_ani->setEndValue(mouseIn_color_);
	b_ani->setDuration(300);

	e_ani = new QPropertyAnimation(this, "color");
	e_ani->setStartValue(mouseIn_color_);
	e_ani->setEndValue(mouseOut_color_);
	e_ani->setDuration(300);
}