#ifndef AIKIT_COMMON_H
#define AIKIT_COMMON_H


typedef enum AIKIT_DATA_PTR_TYPE_E {
    AIKIT_DATA_PTR_MEM  = 0,  // 数据内存指针
    AIKIT_DATA_PTR_FILE = 1,  // 数据文件指针（FILE指针）
    AIKIT_DATA_PTR_PATH = 2   // 数据文件路径指针
} AIKIT_DATA_PTR_TYPE;


#endif  //AIKIT_COMMON_H