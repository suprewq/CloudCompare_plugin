#ifndef EXAMPLEDLG_H
#define EXAMPLEDLG_H


#include "ui_dialog.h"
#include "ccOverlayDialog.h"
#include "ccPickingListener.h"

#include <QDialog>
#include <QMainWindow>
#include <iostream>

using namespace std;

namespace Ui {
class Dialog;
}

class ccGLWindow;
class ccPickingHub;

class ExampleDlg :public ccOverlayDialog,
        public ccPickingListener,
        public Ui::Dialog
{
    Q_OBJECT

public:
    explicit ExampleDlg( ccPickingHub* pickingHub, QWidget *parent = 0);

    //inherited from ccOverlayDialog
    bool linkWith(ccGLWindow* win) override;
    bool start() override;
    void stop(bool state) override;

    //! Inherited from ccPickingListener
    void onItemPicked(const PickedItem& pi) override;

signals:
    void pick_signal(const ccPickingListener::PickedItem& picked);

protected:

    ccPickingHub* m_pickingHub;
};

#endif
