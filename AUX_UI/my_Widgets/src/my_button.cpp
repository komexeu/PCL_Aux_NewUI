#include <my_button.h>

my_button::my_button(QWidget *parent, QString name) :QPushButton(parent) {
	this->setObjectName(name);
	this->setFocusPolicy(Qt::NoFocus);
	this->setText(name);
	QFont font;
	font.setFamily(QString::fromUtf8("Taipei Sans TC Beta"));
	this->setFont(font);
	this->setAutoFillBackground(true);
	this->setFlat(false);
}

void my_button::setColor(QColor color) {
	setStyleSheet(QString(
		"background-color: rgb(%1, %2, %3);"
		"font-size: 16px;"
		"border-radius: 3px;"
		"border:0px;").arg(color.red()).arg(color.green()).arg(color.blue()));
	b_color = color;
	update();
}
QColor my_button::readColor() {
	return b_color;
}

void my_button::enterEvent(QEvent *) {
	b_ani->start();
}
void my_button::leaveEvent(QEvent *) {
	e_ani->start();
}

void my_button::set_styleSheet_color(QColor color_mouseIn, QColor color_mouseOut) {
	mouseIn_color_ = color_mouseIn;
	mouseOut_color_ = color_mouseOut;
	set_mouseAnime();
	this->setStyleSheet(QString(
		"background-color: rgb(%1, %2, %3);"
		"font-size: 16px;"
		"border-radius: 3px;"
		"border:0px;").arg(mouseOut_color_.red()).arg(mouseOut_color_.green()).arg(mouseOut_color_.blue()));
}

void my_button::set_font_color(QColor color) {
	QPalette pal;
	pal.setColor(QPalette::ButtonText, QColor(color.red(), color.green(), color.blue()));
	this->setPalette(pal);
}

void my_button::set_mouseAnime() {
	b_ani = new QPropertyAnimation(this, "color");
	b_ani->setStartValue(mouseOut_color_);
	b_ani->setEndValue(mouseIn_color_);
	b_ani->setDuration(300);

	e_ani = new QPropertyAnimation(this, "color");
	e_ani->setStartValue(mouseIn_color_);
	e_ani->setEndValue(mouseOut_color_);
	e_ani->setDuration(300);
}