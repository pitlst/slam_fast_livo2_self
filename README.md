### 公司手持扫描仪的建图前端

#### 依赖

1. c++23标准
2. opencv
3. mvs海康相机库
4. 岚沃的mid360驱动
5. fastlivo2算法

构建
```bash
cmake -DCMAKE_BUILD_TYPE:STRING=Release -DCMAKE_C_COMPILER:FILEPATH=/usr/bin/gcc-16 .. -G Ninja
cmake --build . --config Release -j 20
```

