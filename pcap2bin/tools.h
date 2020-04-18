#ifndef TOOLS_H
#define TOOLS_H

#include <common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <LasOperator.h>

using namespace std;

//typedef pcl::PointXYZI PointT;
typedef PointType PointT;

void getFiles(std::string path, std::vector<std::string> &files)
{
    struct dirent *ptr;
    DIR *dir;
    dir = opendir(path.c_str());

    while((ptr=readdir(dir)) != NULL)
    {
        if(ptr->d_name[0] == '.') //跳过.和.. 
            continue; 
        //files.push_back(ptr->d_name); //文件名
        //std::string temp = path;
        files.push_back( (path + "/").append(ptr->d_name)); //文件路径
    }
    closedir(dir);
    std::sort(files.begin(), files.end());
}

void showCloud(pcl::visualization::PCLVisualizer *viewer, pcl::PointCloud<PointT>::Ptr cloud, std::string cloud_id)
{
    if(viewer->contains("cloud"))
        viewer->removePointCloud("cloud");
    if(viewer->contains("cloud_id"))
        viewer->removeShape("cloud_id");
    pcl::visualization::PointCloudColorHandlerGenericField<PointT> cloud_handle(cloud, "intensity");
    //pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_handle(cloud, 255, 0, 0);
    viewer->addPointCloud(cloud, cloud_handle, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    viewer->addText(cloud_id, 10, 10, "cloud_id");
    viewer->spinOnce(1);
}

// 暂停view, 注意viewer
void waitForSpace(bool &isPause, pcl::visualization::PCLVisualizer *viewer)
{
   isPause = true;
   while(isPause)
      viewer->spinOnce();
}
#endif