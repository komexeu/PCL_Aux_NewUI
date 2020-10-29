#include <my_foldGroupBox.h>
#include <qdebug.h>

my_foldGroupBox::my_foldGroupBox(QWidget* parent, State state)
	: QGroupBox(parent)
{
	//---------------
	animate_ = new QPropertyAnimation(this, "minimumHeight");

	QFont font;
	font.setFamily(QString::fromUtf8("Taipei Sans TC Beta"));
	this->setFont(font);
	setStyleSheet(QString(
		"QGroupBox#my_foldGroupBox::indicator{width:15px;height: 15px;}"
		"QGroupBox#my_foldGroupBox::indicator:checked{image:url(:/AUX_UI/my_source/Triangle2.png);}"
		"QGroupBox#my_foldGroupBox::indicator:unchecked{image:url(:/AUX_UI/my_source/Triangle1.png);}"
		"QGroupBox#my_foldGroupBox{border:none;}"
		"QGroupBox{color: rgb(255, 255, 255);}"
	));

	setObjectName("my_foldGroupBox");
	setCheckable(true);
	this->setFocusPolicy(Qt::NoFocus);
	state_ = state;
	if (state_ == STATE_FOLD)
	{
		setChecked(false);
	}
	connect(this, SIGNAL(clicked(bool)), this, SLOT(onChecked(bool)));
}

my_foldGroupBox::my_foldGroupBox(const QString& title, QWidget* parent, State state)
	: QGroupBox(title, parent)
{
	groupbox_layout = new QFormLayout(this);
	this->setLayout(groupbox_layout);
	groupbox_layout->setContentsMargins(15, 3, 1, 3);
	//groupbox_layout->setRowWrapPolicy(QFormLayout::WrapAllRows);

	animate_ = new QPropertyAnimation(this, "minimumHeight");

	QFont font;
	font.setFamily(QString::fromUtf8("Taipei Sans TC Beta"));
	this->setFont(font);
	setStyleSheet(QString(
		"QGroupBox#my_foldGroupBox::indicator{width:15px;height: 15px;}"
		"QGroupBox#my_foldGroupBox::indicator:checked{image:url(:/AUX_UI/my_source/Triangle2.png);}"
		"QGroupBox#my_foldGroupBox::indicator:unchecked{image:url(:/AUX_UI/my_source/Triangle1.png);}"
		"QGroupBox#my_foldGroupBox{border:none;}"
		"QGroupBox{color: rgb(255, 255, 255);}"
	));

	setObjectName("my_foldGroupBox");
	setCheckable(true);
	//必須在setCheckable後面，詳見 https://dreamswork.github.io/qt4/qgroupbox_8cpp_source.html#l00485
	this->setFocusPolicy(Qt::NoFocus);
	state_ = state;
	if (state_ == STATE_FOLD)
	{
		setChecked(false);
	}
	connect(this, SIGNAL(clicked(bool)), this, SLOT(onChecked(bool)));
}

void my_foldGroupBox::addWidget(QWidget* widget)
{
	if (widget != nullptr)
	{
		groupbox_layout->addWidget(widget);

		if (state_ == STATE_FOLD)
		{
			widget->setVisible(false);
		}
		children_.push_back(widget);
	}
}

void my_foldGroupBox::addWidget(int row, QFormLayout::ItemRole role, QWidget* widget)
{
	if (widget != nullptr)
	{
		groupbox_layout->setWidget(row, role, widget);

		if (state_ == STATE_FOLD)
		{
			widget->setVisible(false);
		}
		children_.push_back(widget);
	}
}

void my_foldGroupBox::onChecked(bool checked)
{
	if (checked)
	{
		for (auto iter = children_.begin(); iter != children_.end(); ++iter)
		{
			(*iter)->setVisible(true);
		}
		state_ = STATE_EXPAND;
	}
	else
	{
		for (auto iter = children_.begin(); iter != children_.end(); ++iter)
		{
			(*iter)->setVisible(false);
		}
		state_ = STATE_FOLD;
		animate_->setDuration(300);
		animate_->setStartValue(this->geometry().height());
		animate_->setEndValue(this->sizeHint().height());
		animate_->start();
	}
}

my_foldGroupBox::State my_foldGroupBox::getState() const
{
	return state_;
}
