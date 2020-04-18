
#include <common.h>
#include <PointCloudReader.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <tools.h>

//pcl::visualization::PCLVisualizer viewer("Visualization");

void calibPointCloud(ArgParser &args, PointCloud::Ptr &cloud)
{  
    Eigen::Matrix4f calib_T;
    if(args.getDataType() == 1 && args.processFlag!=2) //车载系统点云放平
    {
        calib_T <<  0.809794, -0.056804, 0.583958, 0,
                    0.047749, 0.998381, 0.030902, 0,
                    -0.584768, 0.002859, 0.811196, 0,
                    0.000000, 0.000000, 0.000000, 1.000000;
        pcl::transformPointCloud(*cloud, *cloud, calib_T);
    }
    else if (args.getDataType() == 0) // vlp16,上一代背包点云放平
    {  
        calib_T <<  0.99958,      0.01688,   -0.0235602, -0.000400302,
                    -0.0286371,     0.700434,    -0.713142,   -0.0264561,
                    0.00446452,     0.713517,     0.700623,    -0.236375,
                            0,            0,            0,            1;
        pcl::transformPointCloud(*cloud, *cloud, calib_T);
    }
    else if (args.getDataType() == 2)  // 新一代背包，vlp32水平头
    {

    }
}

void pcd2bin(PointCloud::Ptr &cloud, std::string binFile)
{
    std::ofstream bin(binFile, std::ofstream::binary);
    for (int j = 0; j < cloud->size(); j++) {
        bin.write((char*)& cloud->at(j).x, sizeof(cloud->at(j).x));
        bin.write((char*)& cloud->at(j).y, sizeof(cloud->at(j).y));
        bin.write((char*)& cloud->at(j).z, sizeof(cloud->at(j).z));
        bin.write((char *)& cloud->at(j).intensity, sizeof(cloud->at(j).intensity));
    }
    bin.close();
}
bool isPause = true; //全局变量进行控制
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing){
    if(event.getKeySym() == "space" && event.keyDown()){
        isPause = false;
    }
}

std::string unused;
bool readLine(std::ifstream &trajFile, Eigen::Matrix4d &tf)
{
    if(!(trajFile >> unused)) 
        return false;

    Eigen::Quaterniond q;
    Eigen::Vector3d v; 
    long frameID;
    double timestamp;
    trajFile >> frameID >> v(0) >> v(1) >> v(2)
             >> q.x() >> q.y() >> q.z() >> q.w()
             >> timestamp;
    
    tf = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d r = q.toRotationMatrix();
    tf.block(0,0,3,3) = r;
    tf.block(0,3,3,1) = v;

    return true;
}

int main(int argc, char **argv)
{
    ArgParser args;
    args.parse_arg(argc, argv);
    args.print();

    PointCloud::Ptr cloud(new PointCloud);
   	std::string otDir = "./output/velodyne/";
    CreateDir(otDir.c_str());
    
    PointCloudReader reader;
    reader.setup(args);

    long scanID = 0;
    
    #if 0
    viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*)NULL);
    viewer.setPosition(500,250);
    viewer.addCoordinateSystem(5.0);
    viewer.initCameraParameters();

    //std::ifstream trajFile(args.trajFile.c_str());
    
    //for(int i=0; i<4 && !trajFile.eof(); ++i)
    //    getline(trajFile, unused);
    //Eigen::Matrix4d tf;
    #endif

    while ( reader.readPointCloud(cloud) != -1)
    {
        if(!args.isMerg2Cloud) 
            calibPointCloud(args, cloud);
        else if(args.getDataType() == 2)
            pcl::transformPointCloud<PointType>(*cloud, *cloud, reader.getCalibMatrix().inverse());
        
        //readLine(trajFile, tf);
        //pcl::transformPointCloud<PointType>(*cloud, *cloud, tf);
        
        //showCloud(&viewer, cloud, "cloud"+std::to_string(scanID));
        //waitForSpace(isPause, &viewer);
        #if 1
        char p[10];
		sprintf(p, "%06ld", scanID++);
		pcd2bin(cloud, otDir + std::string(p) + ".bin");
        #endif
    } 
    return 0;
}