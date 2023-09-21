# AIKit-Spark
在sparkDemo.cpp中填入 appId,apiKey,apiSecret,uid

1.编译：
    sh build.sh
2.运行：
    sh run.sh

.
├── SDK                         // SDK头文件及动态库
│   ├── include                 // SDK所需头文件列表
│   │   ├── aikit_biz_api.h
│   │   ├── aikit_biz_builder.h
│   │   ├── aikit_biz_obsolete_builder.h
│   │   ├── aikit_biz_type.h
│   │   ├── aikit_common.h
│   │   ├── aikit_spark_api.h   // spark核心头文件
│   │   └── aikit_type.h
│   └── libs                    // SDK动态库目录
│       └── x64
│           └── libaikit.so
├── demo                        // demo目录
│   ├── build.sh                // demo编译脚本
│   ├── README.md          // demo运行说明
│   ├── run.sh                  // demo运行脚本
│   ├── sparkDemo.cpp           // demo源码
│   └── sparkDemo.h
└── docs                        //SDK相关文档