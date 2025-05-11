# QRSim

QRsim(Quick Real Sim) 是一款使用虚幻引擎4开发的3D仿真平台，目前可以应用于无人系统的运动学控制3D仿真。该平台主要有两大特点，一是可以快速(Quick)验证控制算法的有效性，二是可以无缝迁移到实物平台中进行实物实验。目前该平台已经成功应用于无人艇集群的3D仿真及实物实验中。

QRsim目前采用蓝图(BluePrint)开发，图形化编程使得开发更加容易上手。

QRsim采用C/S结构，虚幻引擎平台用以针对收到的控制信息进行运动，并通过对自身和环境的感知返回给控制器新的状态信息；控制器则使用新的状态信息进行控制器的更新，并反馈给虚幻引擎平台。
这样的设计与实物中非常贴合，有利于快速移植到实物平台上。

## 工程结构

- **Config**: 虚幻引擎工程的默认配置；
- **Content**: 工程的主要内容，其中保留了初学者内容以便后续开发时有基础的物料可以使用；
- **QRSim.uproject**: 工程的核心配置文件，记录了项目的基本信息和设置。

更多项目结构可参考虚幻引擎官网：[目录结构](https://dev.epicgames.com/documentation/zh-cn/unreal-engine/directory-structure?application_version=4.27)

- **Controller_py**: 使用python语言实现的控制器算法，用以和虚幻引擎工程搭配使用，该部分可以使用各种编程语言实现。

## 安装和使用说明

开发版本：虚幻引擎4.27.2

### 安装
1. 安装虚幻引擎，并下载插件：**ObjectDeliverer**和**TCPSocket Plugin**
2. 克隆工程: `git clone https://github.com/adi-lee/QRSim.git`
3. 因本项目中采用了lfs，因此需等待lfs文件加载完毕，可使用`git lfs pull`来确定是否下载完成

### 使用说明
1. 双击打开`.uproject`文件；
2. 运行`Controller_py`文件中的`main.py`；
3. 点击虚幻引擎工程的“运行”，即可体验QRSim

## 功能
1. 键盘WSAD控制无人系统移动；
2. 键盘0~9可控制摄像机视角转换，以在不同角度观察；
3. 无人系统轨迹描绘和消除，tab键控制轨迹生成与否，`Q`按键消除现有轨迹；
4. 利用TCP与控制算法结合，测试控制算法有效性。



![](https://gitee.com/adi-lee/blogs_images/raw/master/imgs/UE2.gif)

![](https://gitee.com/adi-lee/blogs_images/raw/master/imgs/UEMultiUSVsEnclosingControl.gif)