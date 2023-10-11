#ifndef AIKIT_BIZ_OBSOLETE_BUILDER_H
#define AIKIT_BIZ_OBSOLETE_BUILDER_H

#include "aikit_biz_type.h"

namespace AIKIT {

class AIKIT_ParamBuilder;

class AIKITAPI AIKIT_DataBuilderObsolete {
public:
    //设置SDK输入数据格式描述参数
    attribute_deprecated virtual AIKIT_DataBuilderObsolete* desc(const char* key, AIKIT_ParamBuilder* builder) = 0;

    attribute_deprecated virtual AIKIT_DataBuilderObsolete* text(const char* key, const char* data, uint32_t dataLen, uint32_t dataStatus) = 0;
    attribute_deprecated virtual AIKIT_DataBuilderObsolete* textPath(const char* key, const char* path)               = 0;
    attribute_deprecated virtual AIKIT_DataBuilderObsolete* textFile(const char* key, const FILE* file)               = 0;

    attribute_deprecated virtual AIKIT_DataBuilderObsolete* audio(const char* key, const char* data, uint32_t dataLen, uint32_t dataStatus) = 0;
    attribute_deprecated virtual AIKIT_DataBuilderObsolete* audioPath(const char* key, const char* path)               = 0;
    attribute_deprecated virtual AIKIT_DataBuilderObsolete* audioFile(const char* key, const FILE* file)               = 0;

    attribute_deprecated virtual AIKIT_DataBuilderObsolete* image(const char* key, const char* data, uint32_t dataLen, uint32_t dataStatus) = 0;
    attribute_deprecated virtual AIKIT_DataBuilderObsolete* imagePath(const char* key, const char* path)               = 0;
    attribute_deprecated virtual AIKIT_DataBuilderObsolete* imageFile(const char* key, const FILE* file)               = 0;

    attribute_deprecated virtual AIKIT_DataBuilderObsolete* video(const char* key, const char* data, uint32_t dataLen, uint32_t dataStatus) = 0;
    attribute_deprecated virtual AIKIT_DataBuilderObsolete* videoPath(const char* key, const char* path)               = 0;
    attribute_deprecated virtual AIKIT_DataBuilderObsolete* videoFile(const char* key, const FILE* file)               = 0;

};

} // namespace AIKIT

#endif