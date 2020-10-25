#include "AUX_UI.h"
#include <QtWidgets/QApplication> 
#include <vtkOutputWindow.h>

using namespace std;

int main(int argc, char *argv[])
{
	vtkObject::GlobalWarningDisplayOff();
    QApplication a(argc, argv);
    AUX_UI w;
    w.show();
    return a.exec();
}
