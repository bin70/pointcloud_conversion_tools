#pragma once

#ifndef FILE_FUNC_H
#define FILE_FUNC_H

#include <common.h>

///************************************************************************/
///* 创建文件夹，存在则不创建                                                  */
///************************************************************************/
inline int CreateDir(const char *sPathName)
{
   char DirName[256];
   strcpy(DirName, sPathName);
   int i, len = strlen(DirName);
   if (DirName[len - 1] != '/')
      strcat(DirName, "/");
   len = strlen(DirName);
   for (i = 1; i < len; i++)
   {
      if (DirName[i] == '/' || DirName[i] == '\\')
      {
         DirName[i] = 0;
         if (access(DirName, 0) != 0) //存在则返回0
         {
            if (mkdir(DirName, 0755) == -1)
            {
               perror("mkdir   error");
               return -1;
            }
         }
         DirName[i] = '/';
      }
   }

   return 0;
}

///************************************************************************/
///* 删除文件夹，不存在则不删除                                               */
///************************************************************************/
inline int DeleteDir(const char *path)
{
   std::string dirPath = path;
   if(access(path, 0) == 0)
   {
      std::string shell_delete_command = "rm -rf " + dirPath;
      system( shell_delete_command.c_str() );
   }
   return 0;
}

///************************************************************************/
///* 检查文件是否存在 
///************************************************************************/

inline bool CheckFileExist(const char *p)
{
   
   if ((access(p, 0)) == 0)
   {
      return true;
   }
   return false;
}

///************************************************************************/
///* 创建文件 
///************************************************************************/
inline int make_file(std::string _s)
{
   if(!CheckFileExist(_s.c_str()))
   {
      std::ofstream f(_s.c_str());
      f.close();
   }  
   return 0;
}

//*****************
//* 获取文件名
//*****************
inline std::string getFileName(std::string path)
    {
        std::string fullName = path.substr(path.rfind('/') + 1);
        return fullName.substr(0, fullName.find('.'));
    }
#endif