#pragma once
#include <qdialog.h>
#include <qtimer.h>
#include<qprogressbar.h>
#include <qlayout.h>

class CycleProgress : public QDialog
{
	Q_OBJECT
private:
	int currentValue;
	int updateInterval;
	int MaxValue;
	bool Inverted = false;
	QTimer Timer;
	QProgressBar* progressbar;
public:
	CycleProgress(QString title) {
		this->setWindowTitle(title);
		progressbar = new QProgressBar(this);
		currentValue = MaxValue = updateInterval = 0;
		progressbar->setRange(0, 100);
		connect(&Timer, SIGNAL(timeout()), this, SLOT(UpdateSlot()));
		progressbar->setTextVisible(false);
		QHBoxLayout* layout = new QHBoxLayout;
		layout->addWidget(progressbar);
		setLayout(layout);
	}
	void Start(int interval = 30, int maxValue = 100) {
		updateInterval = interval;
		MaxValue = maxValue;
		Timer.start(updateInterval);
		progressbar->setRange(0, MaxValue);
		progressbar->setValue(0);
	}
	void Stop() {
		Timer.stop();
		this->hide();
	}
private slots:
	void UpdateSlot() {
		if (!Inverted)
			currentValue++;
		else
			currentValue--;

		if (currentValue == MaxValue || currentValue == 0) {
			Inverted = !Inverted;
			progressbar->setInvertedAppearance(Inverted);
		}
		progressbar->setValue(currentValue);
	}
};