#pragma once
#include <qdialog.h>
#include <qtimer.h>
#include<qprogressbar.h>
#include <qlayout.h>

class CycleProgress : public QDialog
{
	Q_OBJECT
private:
	int m_CurrentValue;
	int m_UpdateInterval;
	int m_MaxValue;
	bool Inverted=false;
	QTimer m_Timer;
	QProgressBar* progressbar;
public:
	CycleProgress() {
		progressbar = new QProgressBar(this);
		m_CurrentValue = m_MaxValue = m_UpdateInterval = 0;
		progressbar->setRange(0, 100);
		connect(&m_Timer, SIGNAL(timeout()), this, SLOT(UpdateSlot()));
		progressbar->setTextVisible(false);
		QHBoxLayout* layout = new QHBoxLayout;
		layout->addWidget(progressbar);
		setLayout(layout);
	}
	void Start(int interval = 100, int maxValue = 100) {
		m_UpdateInterval = interval;
		m_MaxValue = maxValue;
		m_Timer.start(m_UpdateInterval);
		progressbar->setRange(0, m_MaxValue);
		progressbar->setValue(0);
	}
	void Stop() {
		m_Timer.stop();
		this->hide();
	}
private slots:
	void UpdateSlot() {
		
		if (!Inverted)
			m_CurrentValue++;
		else
			m_CurrentValue--;

		if (m_CurrentValue == m_MaxValue|| m_CurrentValue == 0) {
			Inverted = !Inverted;
			progressbar->setInvertedAppearance(Inverted);
		}
		progressbar->setValue(m_CurrentValue);
	}
};