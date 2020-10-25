#pragma once
#include <qlabel.h>
#include <qtreewidget.h>
#include <QGroupBox>
#include <QVector>

#include <qpropertyanimation.h>
#include <qformlayout.h>

class my_foldGroupBox : public QGroupBox
{
	Q_OBJECT

public:
	enum State
	{
		STATE_FOLD,
		STATE_EXPAND
	};

public:
	my_foldGroupBox(QWidget *parent = nullptr, State state = STATE_FOLD);
	my_foldGroupBox(const QString &title, QWidget *parent = nullptr, State state = STATE_FOLD);

private Q_SLOTS:
	void onChecked(bool checked);

public:
	void addWidget(QWidget *widget);
	void addWidget(int row, QFormLayout::ItemRole role,QWidget *widget);
	State getState() const;

private:
	QVector<QWidget*> children_;
	State state_;
	QPropertyAnimation *animate_;
	QFormLayout *groupbox_layout;
};



