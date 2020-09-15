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

#pragma once

#include "ccMainAppInterface.h"
#include "ccStdPluginInterface.h"

//GUI
#include <ui_dialog.h>
#include "ExampleDlg.h"

//Local
#include "ccOverlayDialog.h"
#include "ccPickingListener.h"

//qCC_db
#include <ccPointCloud.h>
#include <cc2DLabel.h>
#include <ccHObject.h>
#include <ccPickingHub.h>

//qCC_io
#include <FileIOFilter.h>

#include <qmdiarea.h>

//PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <pcl/features/boundary.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>


class ccGLWindow;
class ccPointCloud;
class ccHObject;
class ccPickingHub;

class ExamplePlugin :public QObject,
        public ccStdPluginInterface
{
    Q_OBJECT
    Q_INTERFACES( ccPluginInterface ccStdPluginInterface )
    Q_PLUGIN_METADATA( IID "cccorp.cloudcompare.plugin.Example" FILE "../info.json" )

public:
    explicit ExamplePlugin( QObject* parent = nullptr );

    // Inherited from ccStdPluginInterface
    void onNewSelection( const ccHObject::Container &selectedEntities ) override;
    QList<QAction *> getActions() override;

    void linkWithEntity();

public slots:

    void processPickedPoint(const ccPickingListener::PickedItem& picked);

protected slots:

    void doAction();

    void removeLastEntry();

    void applyAndExit();

    void cancelAndExit();

    void saveAndExit();

protected:

    unsigned getPickedPoints(std::vector<cc2DLabel*>& pickedPoints);

    unsigned getPickedPlanes(std::vector<ccPointCloud*>& planeClouds);

    void updateList();

    ExampleDlg *m_eDlg = nullptr;

    QAction* m_action;

    //! Associated cloud or mesh
    ccHObject* m_associatedEntity;

    //! Last existing label unique ID on load
    unsigned m_lastPreviousID;
    //! Ordered labels container
    ccHObject* m_orderedLabelsContainer;
    //! Existing picked points that the user wants to delete (for proper "cancel" mechanism)
    ccHObject::Container m_toBeDeleted;
    //! New picked points that the user has selected (for proper "cancel" mechanism)
    ccHObject::Container m_toBeAdded;


    ccHObject* m_root;
    ccHObject* m_planeCloudContainer;
    ccHObject::Container m_planeBeAdded;
    ccHObject::Container m_planeBeDeleted;

    ccHObject* m_allPlaneCloudContainer;


    ccPickingHub* m_pickingHub;

    pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;
    pcl::PointCloud<pcl::Normal>::Ptr m_normal;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_plane_cloud;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr m_tree;

    float curvature_threshold_;
    float normal_thrashold_;
    int nebor_size_;

private:

    void deactivatePlanePickingMode(bool state);

    void init_point_cloud(double mlsRadiusSize, int kSeachSize);

    void markerSizeChanged(int size);
    void curvatureSizeChanged(double size);
    void normalSizeChanged(double size);
    void neborSizeChanged(int size);

    void CCcloudToPCLcloud(ccPointCloud* m_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &pclCloud);
    void PCLcloudToCCcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &pclCloud, ccPointCloud* m_cloud);

    void smoothCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr &output,
                     double seach_r);

    void computerNormal(int k_nebor_size);

    void computer_region_growing(int index_point);


};
