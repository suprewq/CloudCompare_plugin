//##########################################################################
//#                                                                        #
//#                CLOUDCOMPARE PLUGIN: ExamplePlugin                      #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                             COPYRIGHT: XXX                             #
//#                                                                        #
//##########################################################################

// First:
//	Replace all occurrences of 'ExamplePlugin' by your own plugin class name in this file.
//	This includes the resource path to info.json in the constructor.

// Second:
//	Open ExamplePlugin.qrc, change the "prefix" and the icon filename for your plugin.
//	Change the name of the file to <yourPluginName>.qrc

// Third:
//	Open the info.json file and fill in the information about the plugin.
//	 "type" should be one of: "Standard", "GL", or "I/O" (required)
//	 "name" is the name of the plugin (required)
//	 "icon" is the Qt resource path to the plugin's icon (from the .qrc file)
//	 "description" is used as a tootip if the plugin has actions and is displayed in the plugin dialog
//	 "authors", "maintainers", and "references" show up in the plugin dialog as well

#include <QtGui>
#include <QApplication>
#include <QMainWindow>
#include <qmainwindow.h>

#include "ExamplePlugin.h"
#include "ExampleDlg.h"

#include <iostream>

using namespace std;

static int s_pickedPointsStartIndex = 0;
static const char s_pickedPointContainerName[] = "Picked point list";
static const char s_pickedPlaneContainerName[] = "Picked plane list";
static const char s_pickedguidaoContainerName[] = "Picked guidao plane";
static const char s_defaultLabelBaseName[] = "Plane #";

ExamplePlugin::ExamplePlugin(QObject* parent)
    : QObject(parent)
    , ccStdPluginInterface( ":/CC/plugin/ExamplePlugin/info.json" )
    , m_action( nullptr )
    , m_pickingHub( nullptr )
{
    m_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    m_plane_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    m_tree.reset(new pcl::search::KdTree<pcl::PointXYZ>);
    m_normal.reset(new pcl::PointCloud<pcl::Normal>);
}

void ExamplePlugin::onNewSelection( const ccHObject::Container &selectedEntities )
{
    if ( m_action == nullptr )
    {
        return;
    }
    m_action->setEnabled(selectedEntities.size()==1 && selectedEntities[0]->isA(CC_TYPES::POINT_CLOUD));
    if(selectedEntities.size() == 1 && selectedEntities[0]->isA(CC_TYPES::POINT_CLOUD))
    {
        ccHObject* entity = selectedEntities[0];
        if (!entity->isKindOf(CC_TYPES::POINT_CLOUD) && !entity->isKindOf(CC_TYPES::MESH))
        {
            ccLog::Error("Select a cloud or a mesh");
            return;
        }

        if (!entity->isVisible() || !entity->isEnabled())
        {
            ccLog::Error("Entity must be visible!");
            return;
        }

        m_associatedEntity = entity;
        m_root = entity->getParent();
        ccLog::Print(m_root->getName());

        init_point_cloud(0.01, 10);
    }
}

QList<QAction *> ExamplePlugin::getActions()
{
    // default action (if it has not been already created, this is the moment to do it)
    if ( !m_action )
    {
        // Here we use the default plugin name, description, and icon,
        // but each action should have its own.
        m_action = new QAction( getName(), this );
        m_action->setToolTip( getDescription() );
        m_action->setIcon( getIcon() );

        // Connect appropriate signal
        connect(m_action, SIGNAL(triggered()), this, SLOT(doAction()));

    }

    return { m_action };
}

void ExamplePlugin::doAction()
{
    if (!m_app )
    {
        // The application interface should have already been initialized when the plugin is loaded
        Q_ASSERT( false );
        return;
    }

    ccGLWindow* win =  m_app->getActiveGLWindow();

    if (!win)
    {
        m_app->dispToConsole( "[ExamplePlugin] Could not find valid 3D window.",
                              ccMainAppInterface::WRN_CONSOLE_MESSAGE );
        return;
    }

    if(!m_eDlg)
    {
        m_eDlg = new ExampleDlg(m_app->pickingHub(), (QWidget*)(win));

        ExampleDlg::connect(m_eDlg->revertToolButton, SIGNAL(clicked(bool)), this, SLOT(removeLastEntry()));
        ExampleDlg::connect(m_eDlg->validToolButton, SIGNAL(clicked(bool)), this, SLOT(applyAndExit()));
        ExampleDlg::connect(m_eDlg->cancelToolButton, SIGNAL(clicked(bool)), this, SLOT(cancelAndExit()));
        ExampleDlg::connect(m_eDlg->saveToolButton, SIGNAL(clicked(bool)), this, SLOT(saveAndExit()));
        ExampleDlg::connect(m_eDlg->markerSizeBox,	static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
                            this,	&ExamplePlugin::markerSizeChanged);
        ExampleDlg::connect(m_eDlg->curvatureBox,	static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
                            this,	&ExamplePlugin::curvatureSizeChanged);
        ExampleDlg::connect(m_eDlg->normalBox,	static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox   ::valueChanged),
                            this,	&ExamplePlugin::normalSizeChanged);
        ExampleDlg::connect(m_eDlg->neborBox,	static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
                            this,	&ExamplePlugin::neborSizeChanged);

        connect(m_eDlg, &ccOverlayDialog::processFinished, this, &ExamplePlugin::deactivatePlanePickingMode);
        connect(m_eDlg, SIGNAL(pick_signal(const ccPickingListener::PickedItem&)),
                this, SLOT(processPickedPoint(const ccPickingListener::PickedItem)));

        m_app->registerOverlayDialog(m_eDlg, Qt::TopRightCorner);

        curvature_threshold_ = m_eDlg->curvatureBox->value();
        normal_thrashold_ = m_eDlg->normalBox->value();
        nebor_size_ = m_eDlg->neborBox->value();;

    }

    m_eDlg->linkWith(win);
    linkWithEntity();
    m_app->freezeUI(true);
    m_app->disableAllBut(win);

    if(!m_eDlg->start())
        ExamplePlugin::deactivatePlanePickingMode(false);
    else
        m_app->registerOverlayDialog(m_eDlg, Qt::TopRightCorner);

}

void ExamplePlugin::init_point_cloud(double mlsRadiusSize, int kSeachSize)
{
    ccPointCloud* cc_cloud = static_cast<ccPointCloud*>(m_associatedEntity);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>);

    CCcloudToPCLcloud(cc_cloud, m_cloud);

    m_tree->setInputCloud(m_cloud);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal>ne;
    ne.setInputCloud(m_cloud);
    ne.setSearchMethod(m_tree);
    ne.setKSearch(kSeachSize);
    ne.compute(*m_normal);
}

unsigned ExamplePlugin::getPickedPoints(std::vector<cc2DLabel*>& pickedPoints)
{
    pickedPoints.clear();

    if (m_orderedLabelsContainer)
    {
        //get all labels
        ccHObject::Container labels;
        unsigned count = m_orderedLabelsContainer->filterChildren(labels, false, CC_TYPES::LABEL_2D);

        try
        {
            pickedPoints.reserve(count);
        }
        catch (const std::bad_alloc&)
        {
            ccLog::Error("Not enough memory!");
            return 0;
        }
        for (unsigned i = 0; i < count; ++i)
        {
            if (labels[i]->isA(CC_TYPES::LABEL_2D)) //Warning: cc2DViewportLabel is also a kind of 'CC_TYPES::LABEL_2D'!
            {
                cc2DLabel* label = static_cast<cc2DLabel*>(labels[i]);
                if (label->isVisible() && label->size() == 1)
                {
                    pickedPoints.push_back(label);
                }
            }
        }
    }

    return static_cast<unsigned>(pickedPoints.size());
}

unsigned ExamplePlugin::getPickedPlanes(std::vector<ccPointCloud*>& planeClouds)
{
    planeClouds.clear();

    if (m_planeCloudContainer)
    {
        //get all labels
        ccHObject::Container labels;
        unsigned count = m_planeCloudContainer->filterChildren(labels, false, CC_TYPES::POINT_CLOUD);

        try
        {
            planeClouds.reserve(count);
        }
        catch (const std::bad_alloc&)
        {
            ccLog::Error("Not enough memory!");
            return 0;
        }
        for (unsigned i = 0; i < count; ++i)
        {
            if (labels[i]->isA(CC_TYPES::POINT_CLOUD))
            {
                ccPointCloud* label = static_cast<ccPointCloud*>(labels[i]);
                if (label->isVisible())
                {
                    planeClouds.push_back(label);
                }
            }
        }
    }

    return static_cast<unsigned>(planeClouds.size());
}

void ExamplePlugin::linkWithEntity()
{
    m_lastPreviousID = 0;

    if (m_root)
    {
        m_orderedLabelsContainer = nullptr;
        m_planeCloudContainer = nullptr;
        ccHObject::Container groups;
        m_root->filterChildren(groups, true, CC_TYPES::HIERARCHY_OBJECT);

        for (ccHObject::Container::const_iterator it = groups.begin(); it != groups.end(); ++it)
        {
            if(m_orderedLabelsContainer == nullptr)
            {
                if((*it)->getName() == s_pickedPointContainerName)
                {
                    m_orderedLabelsContainer = *it;
                }
            }

            if(m_planeCloudContainer == nullptr)
            {
                if((*it)->getName() == s_pickedPlaneContainerName)
                {
                    m_planeCloudContainer = *it;
                }
            }
        }

        std::vector<ccPointCloud*> previousPickedPlanes;
        unsigned count = getPickedPlanes(previousPickedPlanes);
        for (unsigned i = 0; i < count; ++i)
        {
            m_lastPreviousID = std::max(m_lastPreviousID, previousPickedPlanes[i]->getUniqueID());
        }
    }

    updateList();
}

void ExamplePlugin::deactivatePlanePickingMode(bool state)
{
    //we enable all GL windows
    m_associatedEntity = nullptr;
    m_root = nullptr;
    linkWithEntity();

    m_app->enableAll();

    m_app->freezeUI(false);

    m_app->updateUI();
}

void ExamplePlugin::removeLastEntry()
{
    if (!m_associatedEntity)
        return;

    //    //get all labels
    std::vector<cc2DLabel*> labels;
    unsigned count = getPickedPoints(labels);

    std::vector<ccPointCloud*> plane_labels;
    unsigned plane_count = getPickedPlanes(plane_labels);

    if (count == 0)
        return;

    if(plane_count == 0)
        return;

    if(plane_count != count)
        return;

    ccHObject* lastPoint = labels.back();

    if(lastPoint->getUniqueID() < m_lastPreviousID)
    {
        lastPoint->setEnabled(false);
        m_toBeDeleted.push_back(lastPoint);
    }
    else
    {
        if (!m_toBeAdded.empty())
        {
            assert(m_toBeAdded.back() == lastPoint);
            m_toBeAdded.pop_back();
        }

        if (m_planeCloudContainer)
        {
            if (lastPoint->getParent())
            {
                lastPoint->getParent()->removeDependencyWith(lastPoint);
                lastPoint->removeDependencyWith(lastPoint->getParent());
            }
            m_app->removeFromDB(lastPoint);
        }
        else
            m_root->detachChild(lastPoint);
    }

    ccHObject* lastPlane = plane_labels.back();

    if (lastPlane->getUniqueID() <= m_lastPreviousID)
    {
        //old label: hide it and add it to the 'to be deleted' list (will be restored if process is cancelled)
        lastPlane->setEnabled(false);
        m_planeBeDeleted.push_back(lastPlane);
    }
    else
    {
        if (!m_planeBeAdded.empty())
        {
            assert(m_planeBeAdded.back() == lastPlane);
            m_planeBeAdded.pop_back();
        }

        if (m_planeCloudContainer)
        {
            if (lastPlane->getParent())
            {
                lastPlane->getParent()->removeDependencyWith(lastPlane);
                lastPlane->removeDependencyWith(lastPlane->getParent());
            }
            m_app->removeFromDB(lastPlane);
        }
        else
            m_root->detachChild(lastPlane);
    }

    updateList();

    if (m_app->getActiveGLWindow())
        m_app->getActiveGLWindow()->redraw();
}

void ExamplePlugin::applyAndExit()
{
    if (!m_toBeDeleted.empty())
    {
        for(auto & child : m_toBeDeleted)
        {
            m_app->dbRootObject()->removeChild(child);
        }
    }

    if (!m_planeBeDeleted.empty())
    {
        for(auto & child : m_planeBeDeleted)
        {
            m_app->dbRootObject()->removeChild(child);
        }
    }

    m_toBeDeleted.resize(0);
    m_toBeAdded.resize(0);
    m_planeBeDeleted.resize(0);
    m_planeBeAdded.resize(0);

    m_associatedEntity = nullptr;
    m_orderedLabelsContainer = nullptr;
    m_planeCloudContainer = nullptr;

    updateList();

    m_eDlg->stop(true);
}

void ExamplePlugin::cancelAndExit()
{
    if (m_orderedLabelsContainer)
    {
        if (!m_toBeAdded.empty())
        {
            for(auto & child : m_toBeAdded)
            {
                m_app->removeFromDB(child);
            }
        }

        for (auto & object : m_toBeDeleted)
        {
            object->prepareDisplayForRefresh();
            object->setEnabled(true);
        }

        if (m_orderedLabelsContainer->getChildrenNumber() == 0)
        {
            m_app->removeFromDB(m_orderedLabelsContainer);
            m_orderedLabelsContainer = nullptr;
        }
    }

    if (m_planeCloudContainer)
    {
        //Restore previous state
        if (!m_planeBeAdded.empty())
        {
            for(auto & child : m_planeBeAdded)
            {
                m_app->removeFromDB(child);
            }
        }

        for (auto & object : m_planeBeDeleted)
        {
            object->prepareDisplayForRefresh();
            object->setEnabled(true);
        }

        if (m_planeCloudContainer->getChildrenNumber() == 0)
        {
            m_app->removeFromDB(m_planeCloudContainer);
            m_planeCloudContainer = nullptr;
        }
    }

    m_toBeDeleted.resize(0);
    m_toBeAdded.resize(0);
    m_planeBeDeleted.resize(0);
    m_planeBeAdded.resize(0);

    m_associatedEntity = nullptr;
    m_orderedLabelsContainer = nullptr;
    m_planeCloudContainer = nullptr;

    m_app->refreshAll();
    updateList();
    m_eDlg->stop(false);

}

void ExamplePlugin::saveAndExit()
{
    std::vector<ccPointCloud*> plane_labels;
    unsigned plane_count = getPickedPlanes(plane_labels);


    if(plane_count == 0)
        return;

    ccPointCloud* sum_cloud = new ccPointCloud;
    sum_cloud->clear();
    for(auto p : plane_labels)
    {
        sum_cloud->append(p, sum_cloud->size());
    }

    sum_cloud->setVisible(true);
    sum_cloud->setName(QString("all_planes"));
    sum_cloud->showColors(true);
    sum_cloud->setPointSize(5);

    if (!m_allPlaneCloudContainer)
    {
        m_allPlaneCloudContainer = new ccHObject(s_pickedguidaoContainerName);
        m_root->addChild(m_allPlaneCloudContainer);
        m_app->addToDB(m_allPlaneCloudContainer, false, true, false, false);
    }

    assert(m_allPlaneCloudContainer);
    m_allPlaneCloudContainer->addChild(sum_cloud);
    m_app->addToDB(sum_cloud, false, true, false, false);


    if(m_orderedLabelsContainer)
    {
        m_orderedLabelsContainer->removeAllChildren();
        m_app->removeFromDB(m_orderedLabelsContainer);
        m_orderedLabelsContainer = nullptr;
    }
    if(m_planeCloudContainer)
    {
        m_planeCloudContainer->removeAllChildren();
        m_app->removeFromDB(m_planeCloudContainer);
        m_planeCloudContainer = nullptr;
    }

    m_toBeDeleted.resize(0);
    m_toBeAdded.resize(0);
    m_planeBeDeleted.resize(0);
    m_planeBeAdded.resize(0);

    m_associatedEntity = nullptr;
    m_allPlaneCloudContainer = nullptr;

    m_app->refreshAll();
    updateList();
    m_eDlg->stop(false);

}

void ExamplePlugin::markerSizeChanged(int size)
{
    if (size < 1 || !m_app->getActiveGLWindow())
        return;

    //display parameters
    ccGui::ParamStruct guiParams = m_app->getActiveGLWindow()->getDisplayParameters();

    if (guiParams.labelMarkerSize != static_cast<unsigned>(size))
    {
        guiParams.labelMarkerSize = static_cast<unsigned>(size);
        m_app->getActiveGLWindow()->setDisplayParameters
                (guiParams,m_app->getActiveGLWindow()->hasOverridenDisplayParameters());
        m_app->getActiveGLWindow()->redraw();
    }
}

void ExamplePlugin::curvatureSizeChanged(double size)
{
    if(size != curvature_threshold_)
    {
        curvature_threshold_ = size;
    }
}

void ExamplePlugin::normalSizeChanged(double size)
{
    if(size != normal_thrashold_)
    {
        normal_thrashold_ = size;
    }
}

void ExamplePlugin::neborSizeChanged(int size)
{
    if(size != nebor_size_)
    {
        nebor_size_ = size;
    }
}

void ExamplePlugin::updateList()
{
    //get all labels
    std::vector<cc2DLabel*> labels;
    const int count = static_cast<int>( getPickedPoints(labels) );

    std::vector<ccPointCloud*> plane_labels;
    const int plane_count = static_cast<int>( getPickedPlanes(plane_labels) );

    const int oldRowCount = m_eDlg->tableWidget->rowCount();

    m_eDlg->revertToolButton->setEnabled(count);
    m_eDlg->validToolButton->setEnabled(count);
    m_eDlg->saveToolButton->setEnabled(count);
    m_eDlg->countLineEdit->setText(QString::number(count));
    m_eDlg->tableWidget->setRowCount(count);


    if ( count == 0 || plane_count > count)
    {
        return;
    }

    // If we have any new rows, create QTableWidgetItems for them
    if ( (count - oldRowCount) > 0 )
    {
        for ( int i = oldRowCount; i < count; ++i )
        {
            m_eDlg->tableWidget->setVerticalHeaderItem( i, new QTableWidgetItem );

            for ( int j = 0; j < 5; ++j )
            {
                m_eDlg->tableWidget->setItem( i, j, new QTableWidgetItem );
            }
        }
    }

    for ( int i = 0; i < count; ++i )
    {
        cc2DLabel* label = labels[static_cast<unsigned int>( i )];

        const cc2DLabel::PickedPoint& PP = label->getPickedPoint(0);

        CCVector3d Pd =  CCVector3d::fromArray(PP.getPointPosition().u);

        //point index in list
        m_eDlg->tableWidget->verticalHeaderItem( i )->setText( QStringLiteral( "%1" ).arg( i ) );

        //update name as well
        if (	label->getUniqueID() > m_lastPreviousID
                ||	label->getName().startsWith(s_defaultLabelBaseName) ) //DGM: we don't change the name of old labels that have a non-default name
        {
            label->setName(s_defaultLabelBaseName + QString::number(i));
        }

        //point absolute index (in cloud)
        m_eDlg->tableWidget->item( i, 0 )->setText( QStringLiteral( "%1" ).arg( PP.index ) );

        for ( int j = 0; j < 3; ++j )
        {
            m_eDlg->tableWidget->item( i, j + 1 )->setText( QStringLiteral( "%1" ).arg( Pd.u[j], 0, 'f', 4 ) );
        }

    }

    for( int i = 0; i < plane_count; i++)
    {
        ccPointCloud* cloud = plane_labels[static_cast<unsigned int>(i)];
        m_eDlg->tableWidget->item(count - 1, 4)->setText(QStringLiteral( "%1" ).arg( unsigned(cloud->size())));
    }

    m_eDlg->tableWidget->scrollToBottom();
}

void ExamplePlugin::processPickedPoint(const ccPickingListener::PickedItem& picked)
{
    if (!picked.entity || picked.entity != m_associatedEntity || !m_app)
        return;

    cc2DLabel* newLabel = new cc2DLabel();
    if (picked.entity->isKindOf(CC_TYPES::POINT_CLOUD))
    {
        newLabel->addPickedPoint(static_cast<ccGenericPointCloud*>(picked.entity), picked.itemIndex, picked.entityCenter);
    }
    else
    {
        delete newLabel;
        assert(false);
        return;
    }
    newLabel->setVisible(true);
    newLabel->setDisplayedIn2D(false);
    newLabel->displayPointLegend(true);
    newLabel->setCollapsed(true);
    ccGenericGLDisplay* display = m_associatedEntity->getDisplay();
    if (display)
    {
        newLabel->setDisplay(display);
        QSize size = display->getScreenSize();
        newLabel->setPosition(	static_cast<float>(picked.clickPoint.x() + 20) / size.width(),
                                static_cast<float>(picked.clickPoint.y() + 20) / size.height() );
    }

    //add default container if necessary
    if (!m_orderedLabelsContainer)
    {
        m_orderedLabelsContainer = new ccHObject(s_pickedPointContainerName);
        m_root->addChild(m_orderedLabelsContainer);
        m_orderedLabelsContainer->setDisplay(display);
        m_app->addToDB(m_orderedLabelsContainer, false, true, false, false);
    }
    assert(m_orderedLabelsContainer);
    m_orderedLabelsContainer->addChild(newLabel);
    m_app->addToDB(newLabel, false, true, false, false);
    m_toBeAdded.push_back(newLabel);

    //automatically send the new point coordinates to the clipboard
    QClipboard* clipboard = QApplication::clipboard();
    if (clipboard)
    {
        CCVector3 P = newLabel->getPickedPoint(0).getPointPosition();
        int indexInList = static_cast<int>(m_orderedLabelsContainer->getChildrenNumber()) - 1;
        clipboard->setText(QString("CC_POINT_#%0(%1;%2;%3)").arg(indexInList).arg(P.x, 0, 'f', 4).arg(P.y, 0, 'f', 4).arg(P.z, 0, 'f', 4));
    }

    const cc2DLabel::PickedPoint& pick_point = newLabel->getPickedPoint(0);
    computer_region_growing(pick_point.index);

    updateList();

    if (m_app->getActiveGLWindow())
        m_app->getActiveGLWindow()->redraw();
}

void ExamplePlugin::CCcloudToPCLcloud(ccPointCloud* m_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &pclCloud)
{
    int num = m_cloud->size();
    for (int i = 0; i < num; i++)
    {
        pcl::PointXYZ pointT;
        pointT.x = (m_cloud->getPoint(i))->x;
        pointT.y = (m_cloud->getPoint(i))->y;
        pointT.z = (m_cloud->getPoint(i))->z;
        pclCloud->push_back(pointT);
    }
}

void ExamplePlugin::PCLcloudToCCcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &pclCloud, ccPointCloud* m_cloud)
{
    int num = pclCloud->points.size();
    m_cloud->reserve(static_cast<unsigned>(num));
    for (int i = 0; i < num; i++)
    {
        CCVector3 P11(pclCloud->points[i].x, pclCloud->points[i].y, pclCloud->points[i].z);
        m_cloud->addPoint(P11);
        m_cloud->resizeTheRGBTable(true);
        m_cloud->setPointColor(i, ccColor::Rgb(255, 0, 0));
    }
}

void ExamplePlugin::computer_region_growing(int index_point)
{
    if(m_cloud->points.empty())
        return ;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(m_cloud);
    float normal_threshold_real = cosf(normal_thrashold_ / 180.0 * M_PI);
    queue<int> seed;           //种子点队列
    vector<int> point_label;   //搜索标记板
    vector<int> nebor_index;   //邻家索引
    vector<float> nebor_distance;  //邻家距离
    vector<int> all_index;

    seed.push(index_point);
    point_label.resize(m_cloud->points.size(), -1); //未搜索为-1, 已搜索为0
    point_label.at(index_point) = 0;

    while(!seed.empty())
    {
        int curr_seed = seed.front();
        seed.pop();
        int num_nebor = 0;
        tree->nearestKSearch(m_cloud->points.at(curr_seed), nebor_size_, nebor_index, nebor_distance);

        while(num_nebor < nebor_index.size())
        {
            int index_nebor = nebor_index[num_nebor];
            if(point_label.at(index_nebor) != -1)
            {
                num_nebor++;
                continue;
            }

            bool is_a_seed = false;
            Eigen::Map<Eigen::Vector3f> send_curr = m_normal->points.at(curr_seed).getNormalVector3fMap();
            Eigen::Map<Eigen::Vector3f> seed_nebor = m_normal->points.at(index_nebor).getNormalVector3fMap();

            float dot_normal = fabsf(send_curr.dot(seed_nebor));

            if (dot_normal < normal_threshold_real ||
                    m_normal->points[index_nebor].curvature > curvature_threshold_)
            {
                is_a_seed = false;
            }
            else
            {
                is_a_seed = true;
            }
            if (!is_a_seed)
            {
                num_nebor++;
                continue;
            }
            all_index.push_back(index_nebor);
            point_label[index_nebor] = 0;
            if (is_a_seed)
            {
                seed.push(index_nebor);
            }
            num_nebor++;
        }
    }

    ccLog::Print( "cc index size " + QString::number(all_index.size()));

    ccPointCloud* cc_plane_cloud = new ccPointCloud;
    if(!all_index.empty())
    {
        m_plane_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*m_cloud, all_index, *m_plane_cloud);
        PCLcloudToCCcloud(m_plane_cloud, cc_plane_cloud);
    }
    else
    {
        vector<int> temp;
        temp.push_back(index_point);
        m_plane_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*m_cloud, temp, *m_plane_cloud);
        PCLcloudToCCcloud(m_plane_cloud, cc_plane_cloud);
    }

    if (!m_planeCloudContainer)
    {
        m_planeCloudContainer = new ccHObject(s_pickedPlaneContainerName);
        m_root->addChild(m_planeCloudContainer);
        m_app->addToDB(m_planeCloudContainer, false, true, false, false);
    }

    int indexInList = static_cast<int>(m_planeCloudContainer->getChildrenNumber());
    cc_plane_cloud->setVisible(true);
    cc_plane_cloud->setName(QString::fromLocal8Bit("plane_") + QString::number(indexInList));//子目录
    cc_plane_cloud->showColors(true);
    cc_plane_cloud->setPointSize(5);

    assert(m_planeCloudContainer);
    m_planeCloudContainer->addChild(cc_plane_cloud);
    m_app->addToDB(cc_plane_cloud, false, true, false, false);
    m_planeBeAdded.push_back(cc_plane_cloud);


}
