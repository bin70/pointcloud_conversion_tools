#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

pcl::visualization::PCLVisualizer viewer("Visualization");

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

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PCDReader reader;
    std::vector<std::string> NameList;
   	std::string imgDir = argv[1];
    std::string otDir = "./output/";
    
    getFiles(imgDir, NameList);
    
	for (int i = 0; i < NameList.size(); i++)
	{	
        std::cout << NameList[i] << std::endl;
		reader.read(NameList[i], *cloud);
		char p[10];
		sprintf(p, "%d", i + 1);
		std::ofstream file(otDir + "kitti_bin" + std::string(p) + ".bin", std::ofstream::binary);

		for (int j = 0; j < cloud->size(); j++) {

			file.write((char*)& cloud->at(j).x, sizeof(cloud->at(j).x));
			file.write((char*)& cloud->at(j).y, sizeof(cloud->at(j).y));
			file.write((char*)& cloud->at(j).z, sizeof(cloud->at(j).z));
			file.write((char *)& cloud->at(j).intensity, sizeof(cloud->at(j).intensity));
		}
		cout << "success count : " << i << endl;
		file.close();
	}
}