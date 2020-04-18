#include <dirent.h>
#include <string>
#include <iostream>
#include <algorithm>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

pcl::visualization::PCLVisualizer viewer("Visualization");

bool isPause = true; //全局变量进行控制


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

void showCloud(pcl::visualization::PCLVisualizer *viewer, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::string cloud_id)
{
    if(viewer->contains("cloud"))
        viewer->removePointCloud("cloud");
    if(viewer->contains("cloud_id"))
        viewer->removeShape("cloud_id");
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> cloud_handle(cloud, "intensity");
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

// 继续执行view
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing){
    if(event.getKeySym() == "space" && event.keyDown()){
        isPause = false;
    }
}

void readLabels(std::string file, std::vector<float> &labels)
{
	labels.clear();

	std::ifstream file_input(file.c_str());
	float label;
	while(file_input >> label)
		labels.push_back(label);
	file_input.close();
}
/**
 * 显示添加语义标签后的pcd点云, 空格键控制暂停与显示
 */
int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzi(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader reader;
    std::vector<std::string> NameList;
	std::vector<float> labels;
    std::string imgDir = argv[1];
    std::string otDir = "./output/";
    
    getFiles(imgDir, NameList);
    
	std::cout << "init viewer..." << std::endl;
	if(&viewer)  
        viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*)NULL);
    viewer.setBackgroundColor(1.0, 1.0, 1.0);
	viewer.setPosition(500,250);
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();

	std::cout << "finished init." << std::endl;
	for (int i = 0; i < NameList.size(); i++)
	{	
		if(NameList[i].substr(NameList[i].rfind('.')+1, 3) != "pcd")
            continue;
		std::string label_file = NameList[i].substr(0, NameList[i].size()-4) + ".labels";
        std::cout << "pcd:" <<  NameList[i] << ", label: " << label_file << std::endl;
		reader.read(NameList[i], *cloud_xyz);
		readLabels(label_file, labels);
	
		cloud_xyzi->clear();
		cloud_xyzi->height = 1;

		for (int j = 0; j < cloud_xyz->size(); j++) {
			pcl::PointXYZI p;
			pcl::copyPoint(cloud_xyz->points[j], p);
			p.intensity = labels[j];
			cloud_xyzi->points.push_back(p);
		}
		cloud_xyzi->width = cloud_xyzi->points.size();

		std::string id = NameList[i].substr(NameList[i].rfind('/')+1, NameList[i].rfind('.')-NameList[i].rfind('/'));
		showCloud(&viewer, cloud_xyzi, id);
		waitForSpace(isPause, &viewer);

		cout << "success count : " << i << endl;
	}
	while(!viewer.wasStopped()){
        viewer.spinOnce(10);
    }
	return 0;
}