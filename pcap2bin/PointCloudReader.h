#ifndef POINT_CLOUD_READER_H
#define POINT_CLOUD_READER_H

#include <ArgParser.h>
#include <pcapFileReader.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>


class PointCloudReader{
public:
    PointCloudReader();

    int getNscans(){ return Nscans;}  //获取总线数
    int getFrameGap(){ return frameGap;}
    int getNumHaveRead() { return (frameNumber-startFrameID)/frameGap;}
    inline float getRateOfProgess() { return (float)(frameNumber-startFrameID)/(float)(endFrameID-startFrameID);}

    void setup(ArgParser args);
    void setFrameGap(int _frameGap){ frameGap = (_frameGap <= 0)? 1: _frameGap; }; // 设置隔帧数
    void setValidDistance(float _dis_ctrl = 40.0) {distanceControl = _dis_ctrl; } // 设置雷达数据有效范围
    void setVoxelGridLeafsize(float _lf) { vg_leafsize = _lf; }
    
    long long readPointCloudByFrameID(long long frameID, PointCloud::Ptr cloud);
    long long readPointCloud(PointCloud::Ptr cloud);
    long long readPointCloud201(PointCloud::Ptr cloud);
    long long getTotalFrame(){ if (!IsReadSuc()) return 0; return reader1.totalFrame(); }

    double getUSTime(PointCloud::Ptr frame);
    double getUSTimeByFrameID(long long frameID); //获取雷达帧(第一个点)的时间戳
    double getUSTimeByFrameID2(long long frameId); //获取雷达帧(点最多)的时间戳
    double getTimeByFrameID(long long frameID);
    double getTime(PointCloud::Ptr frame);
    double getEndTime(PointCloud::Ptr frame);
    bool isOutdoor();

    Eigen::Matrix4f getCalibMatrix(){ return transformation; }

    long long startFrameID;
    long long endFrameID;
    long long frameNumber;
    long long frameNumber2;
    
private:
    int getPointCloud(pcapReader &reader, long long &frameID, PointCloud::Ptr frame_point_cloud, int scanGap);
    int correctFrameOffset(long long &frameID, int &frameOffset);
    bool OpenPcap();
    bool isSkip();
    bool isEnd();
    void loadMatrixFile(std::string matrixPath);
    void VoxelGrid(PointCloud::Ptr cloud);
    bool IsReadSuc(){ return isReadSuc;}
    void PointCloudFilter(pcl::PointCloud<PointType>::Ptr cloudin, pcl::PointCloud<PointType>::Ptr cloudout, float Leafsize);
    void setPcapPath(std::string pc){ fileNamePcap = pc; }
    void setCalibrationPath(std::string ca){ calibrationPath = ca; }
    void setPcap2Path(std::string _pc){ fileNamePcap2 = _pc; }
    void setCalibration2Path(std::string _ca){ calibrationPath2 = _ca; }
    void setMergeFlag(bool _m){ isMerge2Cloud = _m; }
    void setScansGap(int scanGap);
    void setStartFrame(long long _s){ startFrameID = _s; frameNumber = _s; frameNumber2 = _s; }
    void setEndFrame(long long _e){ endFrameID = _e;}
    
    pcapReader reader1;
    pcapReader reader2;
    std::string fileNamePcap;
    std::string calibrationPath;
    std::string fileNamePcap2;
    std::string calibrationPath2;

    PointCloud::Ptr frame1;
    PointCloud::Ptr frame2;
    bool isMerge2Cloud;
    Eigen::Matrix4f transformation; // 双雷达外标定矩阵

    pcl::VoxelGrid<PointType> grid;
    veloFrame frame;
    bool isSkipStartMin; // 跳过开头数据少的帧
    int minFlamePointSize; // 有效帧的最少点数
    bool isReadSuc;
    float vg_leafsize;
    float distanceControl;
    int sparseThreshold;
    int Nscans;           //default 32
    int Nscans2;          //default 16
    int frameGap;
    int frameOffset;
    int minPcapSize = 1000;
    bool endFlag; // 读完最后一帧
    bool inited;
};
#endif
