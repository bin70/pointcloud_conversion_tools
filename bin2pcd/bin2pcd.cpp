
#include <tools.h>

#define SAVE_CLOUD 1

pcl::visualization::PCLVisualizer viewer("Visualization");
bool isPause = true; //全局变量进行控制

// 继续执行view
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing){
    if(event.getKeySym() == "space" && event.keyDown()){
        isPause = false;
    }
}

std::vector<int> extractRoadLine(const std::vector<float>& intensity_){
  double mean, std, sum=0.0;
  for(int i=0; i<intensity_.size(); ++i)
    sum += intensity_[i];
  mean = sum / intensity_.size();

  sum = 0.0;
  for(int i=0; i<intensity_.size(); ++i)
    sum = sum + (intensity_[i]-mean)*(intensity_[i]-mean);
  std = sum / intensity_.size();
  
  std::vector<int> roadline_idx_;
  for(int i=0; i<intensity_.size(); ++i)
    if(intensity_[i] > mean+7*std) // && intensity_[i] < mean+5*std) 
      roadline_idx_.push_back(i);

  return roadline_idx_;
}


int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZI>);
    //pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud_(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PCDReader reader;
    std::vector<std::string> NameList;
    std::string imgDir = argv[1];
    if(argc < 3){
        printf("Usage ./bin2pcd fileDir las|xyz\n");
        return -1;
    }
    bool tolas = (strcmp(argv[2], "las") == 0);
    std::string otDir = (tolas)?"./output_las":"./output_xyz";
    
    getFiles(imgDir, NameList);

    if(&viewer)  
        viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*)NULL);
    viewer.setPosition(500,250);
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();

    for (int i = 0; i < NameList.size(); i++)
    {
        if(NameList[i].substr(NameList[i].rfind('.')+1, 3) != "bin")
            continue;
        
        std::ifstream input(NameList[i].c_str(), std::ios_base::binary);
        if (!input.good()) {
            std::cerr << "Cannot open file : " << NameList[i].c_str() << std::endl;
            return -1;
        }
        
        cloud_->clear();
        cloud_->height = 1;

        float max_intensity;
        for (int j = 0; input.good() && !input.eof(); j++) {
            pcl::PointXYZI point;
            input.read((char *)&point.x, 3 * sizeof(point.x));
            input.read((char *)&point.intensity, sizeof(point.intensity));
            max_intensity = std::max(point.intensity, max_intensity);
            
            //if(point.y > 5.0 || point.y < -5.0) continue;
            
            cloud_->points.push_back(point);
        }
        input.close();
        
        #if 0
  
        // 对强度做了一个线性归一化
        for (uint32_t j = 0; j < cloud_->points.size(); ++j)
            cloud_->points[j].intensity /= max_intensity;
        
        std::vector<float> intensity;
        for(int j=0; j<cloud_->points.size(); ++j)
            intensity.push_back(cloud_->points[j].intensity);
        std::vector<int> idx_ = extractRoadLine(intensity);
        
        temp_cloud_->clear();
        for(int j=0; j<idx_.size(); ++j)
            temp_cloud_->points.push_back(cloud_->points[ idx_[j] ]);
        *cloud_ = *temp_cloud_;
        #endif

        cloud_->width = cloud_->points.size();
        std::string id = NameList[i].substr(NameList[i].rfind('/')+1, 6);
        showCloud(&viewer, cloud_, id);
        
        #if SAVE_CLOUD
            //if(i >= 90 && i <= 100){
                if(tolas){
                    pcl::copyPointCloud(*cloud_, *cloud);
                    saveLasFile(otDir + "/" + id + ".las", cloud);
                }else{
                    pcl::io::savePCDFileASCII(otDir + "/" + id + ".xyz", *cloud_);
                }
            //}
        #else
            waitForSpace(isPause, &viewer);
        #endif
        cout << "success open : " << NameList[i] << endl;
        
    }
    while(!viewer.wasStopped()){
        viewer.spinOnce(10);
    }

    return 0;
}