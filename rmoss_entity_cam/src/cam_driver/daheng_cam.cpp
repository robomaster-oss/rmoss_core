/*
 * @Author: holakk
 * @Date: 2021-11-05 18:35:57
 * @LastEditors: holakk
 * @LastEditTime: 2021-11-10 20:32:17
 * @Description: file content
 */
#include "rmoss_entity_cam/daheng_cam.hpp"

#include <iostream>
#include <fstream>

namespace rmoss_entity_cam
{
    /**
     * @brief Construct a new Da Heng Cam:: Da Heng Cam object
     * 
     * @param camera_sn 相机序列号
     * @param node 相机节点智能指针
     * @param config_path 配置文件地址
     * @param lut_path Lut表地址
     */
    DaHengCam::DaHengCam(const std::string &camera_sn,
                         rclcpp::Node::SharedPtr node,
                         const std::string &config_path,
                         const std::string &lut_path,
                         const std::vector<double> &lut_detail)
        : _camera_sn(camera_sn), _node(node), _config_path(config_path), _lut_path(lut_path), _lut_detail(lut_detail)
    {
        _param[rmoss_cam::CamParamType::Height] = 0;
        _param[rmoss_cam::CamParamType::Width] = 0;
        _param[rmoss_cam::CamParamType::Fps] = 0;
        _param[rmoss_cam::CamParamType::AutoExposure] = 0; // 0 为手动曝光; 其余为自动曝光亮度目标值
        _param[rmoss_cam::CamParamType::Exposure] = 0;     // 曝光时间 单位 us (默认 2000us)
        _param[rmoss_cam::CamParamType::WhiteBalance] = 0;
        _param[rmoss_cam::CamParamType::RGain] = 0;
        _param[rmoss_cam::CamParamType::GGain] = 0;
        _param[rmoss_cam::CamParamType::BGain] = 0;
        _param[rmoss_cam::CamParamType::Gamma] = 0;
        _param[rmoss_cam::CamParamType::Contrast] = 0; // TODO
        _param[rmoss_cam::CamParamType::Saturation] = 0;
        _param[rmoss_cam::CamParamType::Hue] = 0; // TODO

        // 在起始位置调用 GXInitLib()进行初始化，申请资源
        GX_STATUS GXstatus = GXInitLib();
        if (GXstatus != GX_STATUS_SUCCESS)
        {
            RCLCPP_FATAL(
                this->_node->get_logger(),
                "ERROR:[%d] - Init GXIAPI failed!", GXstatus);
        }

        this->_is_open = false;
        this->_hDevice = nullptr;
    }
    /**
    * @brief Destroy the Da Heng Cam:: Da Heng Cam object
    * 解构函数，实现对图像处理对象内部所有动态内存的清理，以及相机列表指针清理，同时关闭GXLib
    */
    DaHengCam::~DaHengCam()
    {
        // 清理创建的内存
        if (this->_stClrImageProc.pProLut != NULL)
        {
            delete[] this->_stClrImageProc.pProLut;
            this->_stClrImageProc.pProLut = NULL;
        }
        if (this->_stClrImageProc.parrCC != NULL)
        {
            delete[] this->_stClrImageProc.parrCC;
            this->_stClrImageProc.parrCC = NULL;
        }

        GX_STATUS GXstatus = GXCloseLib();
        if (GXstatus != GX_STATUS_SUCCESS)
        {
            RCLCPP_FATAL(
                this->_node->get_logger(),
                "ERROR:[%d] - Close GXIAPI failed!", GXstatus);
        }
    }
    /**
     * @brief 开启相机
     * 
     * @return true 开启成功
     * @return false 开启失败
     */
    bool DaHengCam::open()
    {
        if (this->_is_open)
        {
            return true;
        }
        GX_STATUS GXstatus = GX_STATUS_SUCCESS;
        GX_OPEN_PARAM stOpenParam;
        uint32_t nDeviceNum = 0;
        stOpenParam.openMode = GX_OPEN_SN;
        stOpenParam.accessMode = GX_ACCESS_EXCLUSIVE;
        stOpenParam.pszContent = (char *)this->_camera_sn.c_str();

        // 获取相机列表
        GXstatus = GXUpdateDeviceList(&nDeviceNum, 1000);
        if (GXstatus != GX_STATUS_SUCCESS)
        {
            RCLCPP_FATAL(
                this->_node->get_logger(),
                "ERROR:[%d] - Found camera failed!", GXstatus);
            return false;
        }
        if (nDeviceNum == 0)
        {
            RCLCPP_FATAL(
                this->_node->get_logger(),
                "ERROR: No camera was found!");
            return false;
        }

        GX_DEVICE_BASE_INFO *pBaseinfo = new GX_DEVICE_BASE_INFO[nDeviceNum];
        size_t nSize = nDeviceNum * sizeof(GX_DEVICE_BASE_INFO);
        //获取所有设备的基础信息
        GXstatus = GXGetAllDeviceBaseInfo(pBaseinfo, &nSize);

        for (size_t i = 0; i < nSize; i++)
        {
            GX_DEVICE_BASE_INFO tempBaseInfo = *(pBaseinfo + i);
            if (tempBaseInfo.szSN == this->_camera_sn)
            {
                this->_cameraInfo = tempBaseInfo;
                GXstatus = GXOpenDevice(&stOpenParam, &this->_hDevice);
                if (GXstatus != GX_STATUS_SUCCESS)
                {
                    RCLCPP_FATAL(
                        this->_node->get_logger(),
                        "ERROR:[%d] - Open camera failed!", GXstatus);
                    return false;
                }
            }
        }

        if (this->_cameraInfo.szSN != this->_camera_sn)
        {
            RCLCPP_FATAL(
                this->_node->get_logger(),
                "ERROR:[%d] - Open camera SN wrong!", GXstatus);
            return false;
        }

        delete[] pBaseinfo;

        // 相机打开完成，载入配置
        if (!this->load_config(this->_config_path))
        {
            RCLCPP_FATAL(
                this->_node->get_logger(),
                "ERROR: Load Config file failed!");
            return false;
        }

        if (!this->load_lut(this->_lut_path))
        {
            RCLCPP_FATAL(
                this->_node->get_logger(),
                "ERROR: Load Lut.. file failed!");
            return false;
        }
        //开采
        GXstatus = GXStreamOn(this->_hDevice);
        if (GXstatus != GX_STATUS_SUCCESS)
        {
            RCLCPP_FATAL(
                this->_node->get_logger(),
                "ERROR:[%d] - GXStreamOn open failed!", GXstatus);
            return false;
        }
        this->_is_open = true;
        return true;
    }

    /**
     * @brief 关闭相机
     * 
     * @return true 关闭成功
     * @return false 关闭失败
     */

    bool DaHengCam::close()
    {
        if (!this->_is_open)
        {
            return true;
        }
        //停采
        GX_STATUS GXstatus = GXStreamOff(this->_hDevice);
        if (GXstatus != GX_STATUS_SUCCESS)
        {
            RCLCPP_FATAL(
                this->_node->get_logger(),
                "ERROR:[%d] - Close Image Stream failed!", GXstatus);
            return false;
        }

        GXstatus = GXCloseDevice(this->_hDevice);
        if (GXstatus != GX_STATUS_SUCCESS)
        {
            RCLCPP_FATAL(
                this->_node->get_logger(),
                "ERROR:[%d] - Close camera failed!", GXstatus);
            return false;
        }
        this->_is_open = false;
        return true;
    }

    /**
     * @brief 相机是否开启
     * 
     * @return true 开启
     * @return false 没有开启
     */
    bool DaHengCam::is_open()
    {
        return this->_is_open;
    }

    /**
     * @brief 捕获图片
     * 
     * @param image CV::MAT格式传入的图片
     * @return true 成功
     * @return false 失败
     */
    bool DaHengCam::grab_image(cv::Mat &image)
    {
        GX_STATUS GXstatus = GX_STATUS_SUCCESS;

        //调用 GXDQBuf 取一帧图像
        GXstatus = GXDQBuf(this->_hDevice, &this->_pFrameBuffer, 1000);
        if (GXstatus != GX_STATUS_SUCCESS)
        {
            RCLCPP_FATAL(
                this->_node->get_logger(),
                "ERROR:[%d] - GXDQBuf get failed!", this->_pFrameBuffer->nStatus);
            return false;
        }
        if (this->_pFrameBuffer->nStatus != GX_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_FATAL(
                this->_node->get_logger(),
                "ERROR:[%d] - FrameBuffer set failed!", this->_pFrameBuffer->nStatus);
            return false;
        }
        image = cv::Mat(this->_pFrameBuffer->nHeight, this->_pFrameBuffer->nWidth, CV_8UC3);
        VxInt32 DXstatus = DxRaw8ImgProcess(this->_pFrameBuffer->pImgBuf, image.data, this->_pFrameBuffer->nWidth, this->_pFrameBuffer->nHeight,
                                            &this->_stClrImageProc);
        if (DXstatus != DX_OK)
        {
            RCLCPP_FATAL(
                this->_node->get_logger(),
                "ERROR:[%d] - Raw2RGB failed!", DXstatus);
            return false;
        }
        //调用 GXQBuf 将图像 buf 放回库中继续采图
        GXstatus = GXQBuf(this->_hDevice, this->_pFrameBuffer);
        if (GXstatus != GX_STATUS_SUCCESS)
        {
            RCLCPP_FATAL(
                this->_node->get_logger(),
                "ERROR:[%d] - FrameBuffer set failed!", this->_pFrameBuffer->nStatus);
            return false;
        }

        return true;
    }

    /**
     * @brief 捕获图片和图片时间戳
     * 
     * @param image CV::MAT格式传入的图片
     * @param timestamp_ms 时间戳（ms）
     * @return true 成功
     * @return false 失败
     */
    bool DaHengCam::grab_image(cv::Mat &image, double &timestamp_ms)
    {
        bool result = this->grab_image(image);
        if (!result)
        {
            return false;
        }
        // TODO: 确定时间戳类型和具体情况, 相机内部获取为纳秒级
        timestamp_ms = this->_pFrameBuffer->nTimestamp / 1000000;
        return true;
    }

    /**
     * @brief 加载Config文件
     * 
     * @param path Config文件路径
     * @return true 成功
     * @return false 失败
     */
    bool DaHengCam::load_config(const std::string &path)
    {
        GX_STATUS GXstatus = GX_STATUS_SUCCESS;
        if (path != "")
        {
            GXstatus = GXImportConfigFile(this->_hDevice, path.c_str());
            GXError(GXstatus, "ERROR:[%d] - Import config file failed!", GXstatus);
        }
        else
        {
            // TODO:手动配置参数
            GXstatus = GXSetFloat(this->_hDevice, GX_FLOAT_EXPOSURE_TIME,
                                  2000.);
            GXError(GXstatus, "ERROR:[%d] - Sensor set expose time error!", GXstatus);
        }
        return true;
    }

    /**
     * @brief 加载Lut文件
     * 
     * @param path Lut表路径
     * @return true 成功
     * @return false 失败
     */
    bool DaHengCam::load_lut(const std::string &path)
    {
        VxInt32 DXstatus = DX_OK;
        // 获取查找表长度
        this->_stClrImageProc.bAccelerate = false;
        this->_stClrImageProc.bDefectivePixelCorrect = false;
        this->_stClrImageProc.bDenoise = false;
        this->_stClrImageProc.bFlip = true;
        this->_stClrImageProc.bSharpness = false;
        this->_stClrImageProc.fSharpFactor = 1.5;         // 锐化强度因子[]
        this->_stClrImageProc.cvType = RAW2RGB_NEIGHBOUR; //RAW2RGB使用的算法
        this->_stClrImageProc.emLayOut = BAYERRG;         // RAW图像格式

        DXstatus = DxGetLut(0, 1, 0, NULL,
                            &this->_stClrImageProc.nLutLength);
        if (DXstatus != DX_OK)
        {
            RCLCPP_FATAL(
                this->_node->get_logger(),
                "ERROR:[%d] - Get lut length failed!", DXstatus);
            return false;
        }
        // 申请lut表空间
        this->_stClrImageProc.pProLut = new VxUint8[this->_stClrImageProc.nLutLength];
        if (this->_stClrImageProc.pProLut == NULL)
        {
            RCLCPP_FATAL(
                this->_node->get_logger(),
                "ERROR: Lut array failed!");
            return false;
        }

        // 获取色彩调节数组长度
        this->_stClrImageProc.nCCBufLength = sizeof(VxInt16) * 9;
        // 申请色彩调节数组空间
        this->_stClrImageProc.parrCC = new VxInt16[this->_stClrImageProc.nCCBufLength];
        if (this->_stClrImageProc.parrCC == NULL)
        {
            if (this->_stClrImageProc.pProLut != NULL)
            {
                delete[] this->_stClrImageProc.pProLut;
                this->_stClrImageProc.pProLut = NULL;
            }
            RCLCPP_FATAL(
                this->_node->get_logger(),
                "ERROR: Color adjustment array failed!");
            return false;
        }
        // GXIAPI获取颜色矫正值
        int64_t nColorCorrectionParam = 0;
        GX_STATUS GxStatus = GXGetInt(this->_hDevice, GX_INT_COLOR_CORRECTION_PARAM,
                                      &nColorCorrectionParam);
        if (GxStatus != GX_STATUS_SUCCESS)
        {
            RCLCPP_FATAL(
                this->_node->get_logger(),
                "ERROR:[%d] - Get Color correction value failed!", GxStatus);
            return false;
        }
        DXstatus = DxCalcCCParam(nColorCorrectionParam, 80,
                                 this->_stClrImageProc.parrCC, this->_stClrImageProc.nCCBufLength);
        if (DXstatus != DX_OK)
        {
            RCLCPP_FATAL(
                this->_node->get_logger(),
                "ERROR:[%d] - Compute Color adjustment array failed!", DXstatus);
            return false;
        }

        if (path != "")
        {
            std::ifstream inFile(path, std::ios::in | std::ios::binary); //二进制读方式打开
            if (!inFile)
            {
                RCLCPP_FATAL(
                    this->_node->get_logger(),
                    "ERROR: Read lut file failed!");
                return false;
            }
            // TODO: 确定Lut表长度和载入方式正确: 生成的Lut表和getLut接口不一致
            RCLCPP_FATAL(
                this->_node->get_logger(),
                "ERROR:[%d] Read lut file failed!", this->_stClrImageProc.nLutLength);
            inFile.read((char *)&this->_stClrImageProc.pProLut, sizeof(uint8_t) * this->_stClrImageProc.nLutLength);
            inFile.close();
        }
        else
        {
            // 手动配置lut表参数 默认采用
            // 对比度调节参数[-50-100] 伽马调节参数[0.1-10] 亮度调节参数[-150-150]
            DXstatus = DxGetLut(this->_lut_detail[0], this->_lut_detail[1], this->_lut_detail[2], this->_stClrImageProc.pProLut,
                                &this->_stClrImageProc.nLutLength);
            if (DXstatus != DX_OK)
            {
                if (this->_stClrImageProc.pProLut != NULL)
                {
                    delete[] this->_stClrImageProc.pProLut;
                    this->_stClrImageProc.pProLut = NULL;
                }
                if (this->_stClrImageProc.parrCC != NULL)
                {
                    delete[] this->_stClrImageProc.parrCC;
                    this->_stClrImageProc.parrCC = NULL;
                }
                return false;
            }
        }
        return true;
    }

    // TODO: 加入参数修改
    bool DaHengCam::set_parameter(rmoss_cam::CamParamType type, int value)
    {
        if (_param.find(type) != _param.end())
        {
            _param[type] = value;
            return true;
        }
        else
        {
            return false;
        }
    }

    bool DaHengCam::get_parameter(rmoss_cam::CamParamType type, int & value)
    {
        if (_param.find(type) != _param.end())
        {
            value = _param[type];
            return true;
        }
        else
        {
            return false;
        }
    }

    std::string DaHengCam::error_message()
    {
        // TODO: error_message
        return std::string("");
    }

} // namespace rm_cam
