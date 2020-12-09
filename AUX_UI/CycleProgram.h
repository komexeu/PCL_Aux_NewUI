#pragma once
#include <qdialog.h>
#include <qtimer.h>
#include<qprogressbar.h>
#include <qlayout.h>

class CycleProgram : public QProgressBar
{
	Q_OBJECT
private:
	int currentValue;
	int updateInterval;
	int MaxValue;
	bool Inverted = false;
	bool Work_done = true;
	QTimer Timer;
public:
	CycleProgram(QWidget* parent) {
		currentValue = MaxValue = updateInterval = 0;
		this->setRange(0, 100);
		connect(&Timer, SIGNAL(timeout()), this, SLOT(UpdateSlot()));
		this->setTextVisible(false);
		parent->layout()->addWidget(this);
	}
	void Start(int interval = 30, int maxValue = 100) {
		Work_done = false;
		updateInterval = interval;
		MaxValue = maxValue;
		Timer.start(updateInterval);
		this->setRange(0, MaxValue);
		this->setValue(0);
	}
	void Stop() {
		Work_done = true;
	}
private slots:
	void UpdateSlot() {
		if (Work_done)
		{
			Inverted = false;
			currentValue = 0;
			this->setValue(currentValue);
			this->setInvertedAppearance(Inverted);
			Timer.stop();
			return;
		}

		if (!Inverted)
			currentValue++;
		else
			currentValue--;

		if (currentValue == MaxValue || currentValue == 0) {
			Inverted = !Inverted;
			this->setInvertedAppearance(Inverted);
		}
		this->setValue(currentValue);
	}
};