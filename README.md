# ALFLS Simulator

适用于Acoustic-Lens-Based Forward-Looking-Sonar的模拟器。

## FLS Simulation Review

### Blender (Tokyo University)

基于Blender。

-   通过Normal Map，避免一个面上过多的采样点累积到一个bin内导致过饱和
-   

### HoloOcean

Potokar E, Lay K, Norman K, et al. HoloOcean: Realistic sonar simulation[C]//2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2022: 8450-8456.

基于UE4。不同俯仰角聚合到一个bin点时采用平均。

-   创建了一次反射的多径
-   利用声阻抗性质编辑不同材料改变反射率
-   利用指数分布代替K分布模拟噪声
-   模拟了Oculus的bin-wise自动增益调节（出现黑色条带）



