#include <PointCloudReader.h>

PointCloudReader::PointCloudReader():frame1(new PointCloud), frame2(new PointCloud)
{
    frameNumber = 0;
    frameNumber2 = 0;
    frameGap = 1;
    distanceControl = 40.0;
    vg_leafsize = 0;
    isReadSuc = false;
    isMerge2Cloud = false;
    frameOffset = 0;
    isSkipStartMin = true;
    minFlamePointSize = 16000; //32线单帧最少点数
    minPcapSize = 1000; //pcap包最少帧数
    endFlag = false;
    inited = false;
}

/**
 * 设置读取类的必要参数
 */
void PointCloudReader::setup(ArgParser args)
{
    // 设置数据包路径
    setPcapPath(args.pcapFile); 
    setCalibrationPath(args.calibFile);
    // 开始与结束
    setStartFrame(args.startID); 
    setEndFrame(args.endID);
    // 默认32线
    Nscans = args.nscans; 
    // 隔几帧进行建图, 范围为1~3
    setFrameGap(args.frameGap);
    // 设置有效数据的范围 
    setValidDistance(args.valid_dis);

    // 另一个雷达头
    if(args.isMerg2Cloud)
    {
        setMergeFlag(true);
        setPcap2Path(args.pcap2File);
        setCalibration2Path("./resource/VLP-16.xml"); // 设置了202默认16线
        loadMatrixFile("./resource/autoCalibMatrix.txt"); // 外标定矩阵的默认路径
        Nscans2 = 16;  // 默认202为16线
        Nscans += Nscans2;
    }

    if (Nscans == 16)
      minFlamePointSize = 8000;

    if (!OpenPcap())
    {
      printf("Cannot open pcap file:%s!\n", fileNamePcap.c_str());
      exit(-1);
    }

    if (reader1.totalFrame() < minPcapSize)
    {
      printf("The pcap data is too small!\n");
      exit(-1);
    }
    std::cout << "Total Frame = " << (long int)getTotalFrame() << std::endl;
    inited = true;
}

/**
 * 读取点云, 包含纠正两雷达帧时间戳与合并的代码
 * @param[in] 点云指针
 * @param[out] 读取成功则返回帧编号，读取失败则返回-1;
 */
long long PointCloudReader::readPointCloud(PointCloud::Ptr cloud)
{
    if(!cloud){ 
        std::cout << "*cloud is point to NULL" << std::endl;
        return(-1);
    }
    else
        cloud->clear();

    while(!isEnd() && isSkip()) //用while是因为可能需要对齐时间戳
        ;
    if(vg_leafsize)
        VoxelGrid(frame1);
    *cloud = *frame1;
    return endFlag ? -1 : frameNumber-frameGap;
}


/**
 * 检查当前帧的有效性,无效则跳过
 * 同时调整两雷达数据的时间戳, 合并pcap2数据帧到pcap1数据帧中
 * 实际上这里变换的是-f指定的数据
 */
bool PointCloudReader::isSkip()
{  
    #if 0
    if(isMerge2Cloud)
    {  
      switch (correctFrameOffset(frameNumber, frameOffset))
      {
         case 0: //纠正了reader1.frameNumber,重新读取
            return true;
         case 1: // 合并两帧
            pcl::transformPointCloud<PointType>(*frame1, *frame1, transformation);
            *frame1 += *frame2;
            break;
         case -1: // 201丢帧,或两雷达时间差过大,不合并
            printf("Jump 201 frame\n");
            break;
      }
    }

    //if (frame1->points.size() > 79900) // featureextraction.h中设置的最大上限
    //    return true;
    
    if (isSkipStartMin) // 跳过数据包开头那些点数不足的帧
    {
        isSkipStartMin = (frame1->points.size() < minFlamePointSize);
        return isSkipStartMin;
    }
    #else
    pcl::transformPointCloud<PointType>(*frame1, *frame1, transformation);
    *frame1 += *frame2;
    #endif
    
    return false;
}

/**
 * 检查是否到达数据包尾部
 */
bool PointCloudReader::isEnd()
{
   if(getPointCloud(reader1, frameNumber, frame1, 0) == -1)     // pcap1 读完
   {
       endFlag = true;
       //std::cout << "frameNumber = " << frameNumber << ", pcap1 is end." << std::endl;
       return endFlag;
   }
      
   // 更新进度条
   long long frameID = frameNumber - frameGap;
   consoleProgress(int(1.0 * frameID / reader1.totalFrame() * 76));

   if (endFrameID > 0 && frameID > endFrameID) // 到达用户指定的结束帧
   {
       endFlag = true;
       return endFlag;
   }

   if (isMerge2Cloud) 
   {
      frameNumber2 = frameID + frameOffset;
      if (getPointCloud(reader2, frameNumber2, frame2, Nscans-Nscans2) == -1) // pcap2 读完
       {   
          endFlag = true;
          return endFlag;
      }
   }
   return false;
}

/**
 * 按给定帧号读取点云
 * @param[in] 指定帧号, 点云指针
 * @param[out] 读取成功则返回读取到的帧编号，读取失败则返回-1;
 */
long long PointCloudReader::readPointCloudByFrameID(long long frameID, PointCloud::Ptr cloud)
{
    frameNumber = frameID;
    return readPointCloud(cloud);
}

/**
 * 仅读取201的数据,用于建图
 */
long long PointCloudReader::readPointCloud201(PointCloud::Ptr cloud)
{
    if(!cloud) 
        return(-1);
    else
        cloud->clear();
    
    if(getPointCloud(reader1, frameNumber, cloud, 0) == -1)
        endFlag = true;
    
    if(isMerge2Cloud)
        pcl::transformPointCloud<PointType>(*cloud, *cloud, transformation);
            
    return endFlag ? -1 : frameNumber-frameGap;
}

/**
 * 获取雷达帧(第一个点)的时间戳, 以微秒为单位
 */
double PointCloudReader::getUSTime(PointCloud::Ptr frame)
{
    return getTime(frame)*1e6;
}

/**
 * 获取雷达帧(第一个点)的时间戳, 以秒为单位
 */
double PointCloudReader::getTime(PointCloud::Ptr frame)
{
    return frame->points[0].data_n[0] + frame->points[0].data_n[1];
}

/**
 * 获取雷达帧(最后一个点)的时间戳, 以秒为单位
 */
double PointCloudReader::getEndTime(PointCloud::Ptr frame)
{
    return frame->points.back().data_n[0] + frame->points.back().data_n[1];
}

/**
 * 获取指定帧号的雷达帧(第一个点)的时间戳, 以微秒为单位
 */
double PointCloudReader::getUSTimeByFrameID(long long frameID)
{
    if (!reader1.capture(frame, frameID))
        return -1;

    return frame.lines[0].pTimestamp[0];
}

/**
 * 获取指定帧号的雷达帧(第一个点)的时间戳, 以秒为单位
 */
double PointCloudReader::getTimeByFrameID(long long frameID)
{
    return getUSTimeByFrameID(frameID) / 1e6;
}
/**
 * 获取指定帧号的雷达帧(最多点)的时间戳, 以微秒为单位
 */
double PointCloudReader::getUSTimeByFrameID2(long long int frameId)
{
    double curTimeStamp;
    std::vector<std::pair<double, int>> timestampList;

    if (!reader1.capture(frame, frameId))
        return -1;

    // count each point's timestamp, select max counted as the frame's timestamp
    for (int n = 0; n < frame.numLine / 2; n++)
    {
        for (int i = 0; i < frame.lines[n].num / 100; i++)
        {
            curTimeStamp = frame.lines[n].pTimestamp[i];
            bool _flag = true;

            // find this timestamp in list
            for (std::vector<std::pair<double, int>>::iterator it = timestampList.begin(); it != timestampList.end(); ++it)
            {
                if (it->first == curTimeStamp)
                {
                    _flag = false;
                    it->second++;
                }
            }
            // if not exist, add new timstamp
            if (_flag)
            {
                std::pair<double, int> _tmp_node;
                _tmp_node.first = curTimeStamp;
                _tmp_node.second = 1;
                timestampList.push_back(_tmp_node);
            }
        }
    }

    // find a timestamp has most count
    int _max_conut = 0;
    for (std::vector<std::pair<double, int>>::iterator it = timestampList.begin(); it != timestampList.end(); ++it)
    {
        if (it->second > _max_conut)
        {
            _max_conut = it->second;
            curTimeStamp = it->first;
        }
    }
    return curTimeStamp;
}




////////////////////////////////////////////////////////////////////

/**
 * 检查pcap数据包是否能够打开
 */
bool PointCloudReader::OpenPcap()
{
    if (fileNamePcap == "" || calibrationPath == "")
        return false;
    isReadSuc = reader1.open(fileNamePcap, calibrationPath);
    if (isMerge2Cloud)
    {
        if (fileNamePcap2 == "" || calibrationPath2 == "")
            return false;
        isReadSuc = (isReadSuc && reader2.open(fileNamePcap2, calibrationPath2));
    }
    return isReadSuc;
}


/**
 * 纠正两个雷达帧的时间差小于半帧时间
 */
int PointCloudReader::correctFrameOffset(long long &frameID, int &offset)
{
   //202也可能掉帧，判断一下
   float frameTime = getEndTime(frame2) - getTime(frame2);
   
   if (frame2->points.size() < 8000 || frameTime > 0.06 || frameTime < 0.04)
   {
      std::cout << "Attention: 202可能存在掉帧或者数据不稳定情况!!!\n";
      return -1; //不纠正
   }

   // 以第一个点时间戳作为当前帧时间戳
   float timeOffset = getTime(frame2) - getTime(frame1);
   frameTime = 0.05; //正确的时间差

   //计算两个激光头之间的对应帧号差
   int _a = timeOffset / frameTime;
   // 让时间差位于半帧的时间之内
   _a += 1.9 * (timeOffset - _a * frameTime) / frameTime;
   offset -= _a;

   if (abs(_a) > 0)
   {
      if (TEST)
      {
        std::cout << "======================offset============================\n"
              << "FrameID = " << frameID << "\t"
              << "Offset = " << -_a << "\t"
              << "Sumoffset = " << offset << "\t "
              << "Timeoffset  = " << timeOffset << std::endl;
         //showTime(frame1, frameID, "202");
         //showTime(frame2, reader2.frameNumber - skipFrameNumber, "201");
         std::cout << "=============================================================\n";
         if (abs(_a) > 1 && frameID - startFrameID > 5)
         {
            std::cout << "\nAttention: Frame2's timestamp is changing more than 0.25s!!!\n";
            return -1;
         }
      }

      if (frameID + offset < 3)
         frameID += _a;
      return 0;
   }
   return 1;
}


/**
 * 按指定的frameID, 从pcap数据包中读取点云存入cloud
 */
int PointCloudReader::getPointCloud(pcapReader &reader, long long &frameID, PointCloud::Ptr cloud, int scanGap)
{
    cloud->clear();   
    if (!reader.capture(frame, frameID)){
        //std::cout << "frameID = " << frameID << std::endl;
        return -1;
    }
    float max_intensity = 0;
    for (int n = 0; n < frame.numLine; n++)
    {
        for (int i = 0; i < frame.lines[n].num; i++)
        {
            PointType pt;
            pt.x = (float)frame.lines[n].pPosition[i].x;
            pt.y = (float)frame.lines[n].pPosition[i].y;
            pt.z = (float)frame.lines[n].pPosition[i].z;
            float dist = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            if (distanceControl && (dist > distanceControl))
                continue;
            
            // pt.intensity = frame.lines[n].pIntensity[i] + 0.1f;
            pt.intensity = frame.lines[n].pIntensity[i]; 
            max_intensity = std::max(pt.intensity, max_intensity);
            

            double timestamp = frame.lines[n].pTimestamp[i] / 1e6; //秒为单位
            pt.data_n[0] = int(timestamp);           //整数部分为秒
            pt.data_n[1] = timestamp - pt.data_n[0]; //微秒
            pt.data_n[2] = n + scanGap;
            pt.data_n[3] = dist;
            cloud->points.push_back(pt);
        }
    }
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = true;
    
    #if 1
    for (uint32_t j = 0; j < cloud->points.size(); ++j)
        cloud->points[j].intensity /= max_intensity;
    #endif

    frameID = frameID+frameGap;
    return 0;
}

/**
 * 网格滤波器
 */
void PointCloudReader::VoxelGrid(PointCloud::Ptr cloud)
{
    PointCloud::Ptr cloud_filtered(new PointCloud);
    grid.setLeafSize(vg_leafsize, vg_leafsize, vg_leafsize);
    grid.setInputCloud(cloud);
    grid.filter(*cloud_filtered);
    *cloud = *cloud_filtered;

    // remove NAN points from the cloud
    std::vector<int> indices;
    PointCloud::Ptr cloud_remove_nan(new PointCloud);
    pcl::removeNaNFromPointCloud(*cloud, *cloud_remove_nan, indices);
    *cloud = *cloud_remove_nan;
}

/**
 * 异常点滤波
 */
void PointCloudReader::PointCloudFilter(pcl::PointCloud<PointType>::Ptr cloudin, pcl::PointCloud<PointType>::Ptr cloudout, float Leafsize)
{
    cloudout->clear();
    grid.setLeafSize(Leafsize, Leafsize, Leafsize);
    grid.setInputCloud(cloudin);
    grid.filter(*cloudout);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloudout, *cloudout, indices);
}

/**
 * 从文件读取标定矩阵
 */
void PointCloudReader::loadMatrixFile(std::string matrixPath)
{
   std::ifstream matrix_file(matrixPath);
   if (!matrix_file)
   {
      std::cout << "Cannot read " << matrixPath << std::endl;
      matrix_file.close();
      exit(-1);
   }
   for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j)
         matrix_file >> transformation(i, j);
   matrix_file.close();
}

/**
 * 检查是否为室外数据
 */
bool PointCloudReader::isOutdoor()
{
    if(!inited) 
        CATCH_ERROR("PointCloudReader hasn't be inited, use setup to init it.");
   
   float outdoorTreshhold = 0.035;
   PointCloud::Ptr frame(new PointCloud);
   float meanSize = 0;
   float sigmaSize = 0;
   std::vector<int> sizeIndices;
   if (endFrameID < 0)
      endFrameID = getTotalFrame()-1;

   for (long long i = startFrameID + 100; i < endFrameID - 50; i += 50)
   {
      readPointCloudByFrameID(i, frame);
      sizeIndices.push_back(frame->points.size());
      meanSize += sizeIndices.back();
   }

   meanSize /= sizeIndices.size();
   //std::cout << "meanSize = " << meanSize << std::endl;
   
   for (int i = 0; i < sizeIndices.size(); i++)
   {
      sigmaSize += pow(sizeIndices[i] - meanSize, 2);
   }
   sigmaSize /= sizeIndices.size();
   sigmaSize = sqrt(sigmaSize) / meanSize;
   //std::cout << "sigmaSize = " << sigmaSize << std::endl;

   if (sigmaSize > outdoorTreshhold && !TEST)
   {
      printf("WARNING: The program can't running in currunt environment!\n\n");
      return true;
   }
   else if (sigmaSize > outdoorTreshhold / 2)
   {
      printf("The mapping result may be not good in currunt environment!\n\n");
      return false;
   }
   else if (sigmaSize < 0)
   {
      printf("Isoutdoor:Something wrong with prob!\n");
      return true;
   }
   frameNumber = startFrameID;
}

