#include "ExampleDlg.h"

#include <ccLog.h>
#include <ccPointCloud.h>
#include <ccPickingHub.h>

ExampleDlg::ExampleDlg(ccPickingHub* pickingHub, QWidget *parent/*=0*/)
    : ccOverlayDialog(parent)
    , m_pickingHub( pickingHub )
    , Ui::Dialog()
{
    setupUi(this);
}

bool ExampleDlg::linkWith(ccGLWindow* win)
{
    if (win == m_associatedWin)
    {
        //nothing to do
        return false;
    }
    ccGLWindow* oldWin = m_associatedWin;

    //just in case
    if (m_pickingHub)
    {
        m_pickingHub->removeListener(this);
    }

    if (!ccOverlayDialog::linkWith(win))
    {
        return false;
    }

    //if the dialog is already linked to a window, we must disconnect the 'point picked' signal
    if (oldWin)
    {
        oldWin->disconnect(this);
    }

    return true;
}

bool ExampleDlg::start()
{
    if (!m_pickingHub)
    {
        ccLog::Error("[Point picking] No associated display!");
        return false;
    }

    //activate "point picking mode" in associated GL window
    if (!m_pickingHub->addListener(this, true, true, ccGLWindow::POINT_PICKING))
    {
        ccLog::Error("Picking mechanism already in use. Close the tool using it first.");
        return false;
    }

    //the user must not close this window!
    m_associatedWin->setUnclosable(true);
    m_associatedWin->redraw(true, false);

    ccOverlayDialog::start();

    return true;
}

void ExampleDlg::stop(bool state)
{
    if (m_pickingHub)
    {
        //deactivate "point picking mode" in all GL windows
        m_pickingHub->removeListener(this);

        if ( m_associatedWin != nullptr )
        {
            m_associatedWin->setUnclosable(false);
            m_associatedWin->redraw(true, false);
        }
    }

    ccOverlayDialog::stop(state);
}

void ExampleDlg::onItemPicked(const PickedItem& pi)
{
    if (m_processing && pi.entity)
    {
        emit pick_signal(pi);
    }
}





