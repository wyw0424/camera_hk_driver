#include<iostream>
#include<stdio.h>
#include<cstdlib>
#include"HCNetSDK.h"
#include <LinuxPlayM4.h>

#include "ros/ros.h"
#include "yidamsg/Image_data.h"
#include "yidamsg/motor_control.h"
#include "yidamsg/CameraChange.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"

#include <unistd.h>
#include <opencv2/opencv.hpp>
#include"convert.h"
#include "jpeglib.h"
#include "jerror.h"
#include <setjmp.h>
#include<pthread.h>
#include<time.h>
#include "iniFile.h"
#include <jpeglib.h>


using namespace std;

ros::Publisher g_pub;
image_transport::Publisher pub;

int g_lPort = 0;
char* g_rgbBuffer1 = NULL;
char* g_jpegBuffer = NULL;
long flag, lUserID1, lRealPlayHandle1;
pthread_mutex_t g_mutex=PTHREAD_MUTEX_INITIALIZER;
typedef struct my_error_mgr * my_error_ptr;

bool camera_change_callback(yidamsg::CameraChange::Request  &req, yidamsg::CameraChange::Response &res);
void jpeg2rgb(unsigned char*jpeg_buffer, int jpeg_size, unsigned char* rgb_buffer, int width, int height);
int rgb2jpeg(unsigned char* rgb_buffer, int width, int height, int quality, unsigned char** jpeg_buffer, unsigned long* jpeg_size);

void my_error_exit(j_common_ptr cinfo);
void CALLBACK DecCBFun(int nPort, char* pBuf, int  nSize, FRAME_INFO* pFrameInfo, int nReserved1, int nReserved2);
void CALLBACK g_RealDataCallBack_V30(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer1, DWORD dwBufSize, void* dwUser);
bool YV12ToBGR24_Table(unsigned char* pYUV,unsigned char* pBGR24,int width,int height);

int main(int argc, char **argv)
{
    IniFile ini("Device.ini");
    unsigned int dwSize = 0;
    char sSection[16] = "DEVICE";

    char *sIP1 = ini.readstring(sSection, "ip1", "error", dwSize);
    int iPort = ini.readinteger(sSection, "port", 0);

    char *sUserName = ini.readstring(sSection, "username", "error", dwSize);
    char *sPassword1 = ini.readstring(sSection, "password1", "error", dwSize);
    char *sTopic1 = ini.readstring(sSection, "topic1", "error", dwSize);
    char *rosNode = ini.readstring(sSection, "node", "error", dwSize);
    int iChannel = ini.readinteger(sSection, "channel", 0);
    int num = ini.readinteger(sSection, "number", 0);
    
    sIP1 = argv[1];
    //sIP1 = "192.168.1.64";
    iPort = 8000;
    sUserName = "admin";
    sPassword1 = "123qweasd";
    sTopic1 = argv[2];
    //sTopic1 = "camera1";

    num = 1;	
    lUserID1 = -1;
    lRealPlayHandle1 = -1;
    ros::init(argc, argv, argv[3]);
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    pub = it.advertise(sTopic1, 1);

	

//     ros::ServiceServer service = n.advertiseService("camera_change", camera_change_callback);
    ros::Rate loop_rate(20);

    NET_DVR_Init();
    char c = 'q';
    NET_DVR_DEVICEINFO_V30 struDeviceInfo = {0};
    char* sJpegPicBuffer1 = new char[1920*960*3];
    assert(sJpegPicBuffer1);
    g_jpegBuffer = new char[1920*960*3];
    g_rgbBuffer1 = sJpegPicBuffer1;


    NET_DVR_CLIENTINFO ClientInfo = {0};
    ClientInfo.hPlayWnd     = 0;
    ClientInfo.lChannel     = 1;  //channel NO.

    ClientInfo.lLinkMode    = 0;
    ClientInfo.sMultiCastIP = NULL;

    lUserID1 = NET_DVR_Login_V30(sIP1, iPort, sUserName, sPassword1, &struDeviceInfo);


    if (lUserID1 < 0)
    {
        printf("ip1: %s---Login error,  %d\n", sIP1, NET_DVR_GetLastError());
        return 0;
    }
    lRealPlayHandle1 = NET_DVR_RealPlay_V30(lUserID1, &ClientInfo, g_RealDataCallBack_V30, NULL, 0);//
    char ch= 'a';
    pthread_mutex_init(&g_mutex,NULL);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    NET_DVR_Cleanup();

}

bool YV12ToBGR24_Table(unsigned char* pYUV,unsigned char* pBGR24,int width,int height)
{
    //cout << "YV12toBGR24" << endl;
    if (width < 1 || height < 1 || pYUV == NULL || pBGR24 == NULL)
        return false;
        const long len = width * height;
         unsigned char* yData = pYUV;
         unsigned char* vData = &yData[len];
         unsigned char* uData = &vData[len >> 2];
         int bgr[3];
         int yIdx,uIdx,vIdx,idx;
         int rdif,invgdif,bdif;
         for (int i = 0;i < height;i++)
            {
                for (int j = 0;j < width;j++)
                {
                    yIdx = i * width + j;
                    vIdx = (i/2) * (width/2) + (j/2);
                    uIdx = vIdx;
                    rdif = Table_fv1[vData[vIdx]];
                    invgdif = Table_fu1[uData[uIdx]] + Table_fv2[vData[vIdx]];
                    bdif = Table_fu2[uData[uIdx]];
                    bgr[0] = yData[yIdx] + bdif;
                    bgr[1] = yData[yIdx] - invgdif;
                    bgr[2] = yData[yIdx] + rdif;
                    for (int k = 0;k < 3;k++)
                    {
                       idx = (i * width + j) * 3 + k;
                       if(bgr[k] >= 0 && bgr[k] <= 255)
                       pBGR24[idx] = bgr[k];
                       else
                       pBGR24[idx] = (bgr[k] < 0)?0:255;
                    }
                }
            }
           return true;
}

int rgb2jpeg(unsigned char* rgb_buffer, int width, int height, int quality, unsigned char** jpeg_buffer, unsigned long* jpeg_size)
{
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    int row_stride = 0;
    JSAMPROW row_pointer[1];

    if (jpeg_buffer == NULL)
    {
        printf("you need a pointer for jpeg buffer.\n");
        return -1;
    }

    cinfo.err = jpeg_std_error(&jerr);

    jpeg_create_compress(&cinfo);

    jpeg_mem_dest(&cinfo, jpeg_buffer, jpeg_size);

    cinfo.image_width = width;
    cinfo.image_height = height;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;

    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, 1);  // todo 1 == true
    jpeg_start_compress(&cinfo, TRUE);
    row_stride = width * cinfo.input_components;

    while (cinfo.next_scanline < cinfo.image_height)
    {
        row_pointer[0] = &rgb_buffer[cinfo.next_scanline * row_stride];
        jpeg_write_scanlines(&cinfo, row_pointer, 1);
    }

    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);

    return 0;
}

void  jpeg2rgb(unsigned char*jpeg_buffer, int jpeg_size, unsigned char* rgb_buffer, int width, int height)
{
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;
    int row_stride;
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&cinfo);//初始化JPEG对象
    jpeg_mem_src(&cinfo, jpeg_buffer, jpeg_size);
    (void) jpeg_read_header(&cinfo, TRUE);
    cinfo.out_color_space = JCS_RGB;//JCS_GRAYSCALE;
    (void) jpeg_start_decompress(&cinfo);
    row_stride = cinfo.output_width * cinfo.output_components;
    JSAMPROW row_pointer[1];
    while (cinfo.output_scanline < cinfo.output_height)
    {
       row_pointer[0] = &rgb_buffer[(cinfo.output_height - cinfo.output_scanline-1)*cinfo.image_width*cinfo.num_components];
       jpeg_read_scanlines(&cinfo,row_pointer, 1);
    }
    (void) jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
}

void my_error_exit(j_common_ptr cinfo)
{

    my_error_ptr myerr = (my_error_ptr) cinfo->err;

    (*cinfo->err->output_message) (cinfo);

//    longjmp(myerr->setjmp_buffer, 1);
}


bool camera_change_callback(yidamsg::CameraChange::Request  &req, yidamsg::CameraChange::Response &res)
{
    if(req.flag==1)
    {
        flag = lRealPlayHandle1;
    }
    else
    {
//         flag = lRealPlayHandle2;
    }
    res.status = true;
    return true;
}

void  CALLBACK DecCBFun(int nPort, char* pBuf, int  nSize, FRAME_INFO* pFrameInfo, int nReserved1, int nReserved2)
{
    //cout << "DecCBFun1" << endl;
    yidamsg::Image_data msg;
    if(pFrameInfo->nType == T_YV12)
    {
        YV12ToBGR24_Table((unsigned char*)pBuf,(unsigned char*)g_rgbBuffer1 ,pFrameInfo->nWidth, pFrameInfo->nHeight);
        cv::Mat m1(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC3, g_rgbBuffer1);
        cv::Mat m2;
        cv::resize(m1, m2, cv::Size(320, 240));
        char test[20] = {0};
        //sprintf(test, "Frame:---%d", pFrameInfo->dwFrameNum);
        cv::imshow("camera1", m2);
        cv::waitKey(1);
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", m2).toImageMsg();
	pub.publish(msg);

//         long unsigned lSize = 0;
//         rgb2jpeg((unsigned char*)g_rgbBuffer,pFrameInfo->nWidth,pFrameInfo->nHeight,50, (unsigned char**)&g_jpegBuffer,&lSize);
//         msg.width = pFrameInfo->nWidth;
//         msg.height = pFrameInfo->nHeight;

//         if(lSize<200000)
//         {
//             memcpy((void*)&msg.pImgBuf,g_jpegBuffer,lSize);
//             msg.length = lSize;
//             msg.id = pFrameInfo->dwFrameNum;
//             g_pub.publish(msg);
//         }
//        printf("花费时间：%d秒-------DecCBFun总花费时间:%d毫秒,step1:%d,step2:%d,step3:%d,第%d帧\n",(clock() - m)/CLOCKS_PER_SEC,(clock()-t)*1000/CLOCKS_PER_SEC, step1*1000/CLOCKS_PER_SEC,step2*1000/CLOCKS_PER_SEC,step3*1000/CLOCKS_PER_SEC,pFrameInfo->dwFrameNum);
    }

}

void CALLBACK g_RealDataCallBack_V30(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer1, DWORD dwBufSize, void* dwUser)
{
    //cout << "RealDataCallBack" << " " << lRealHandle << " " << lRealPlayHandle1 << endl;
    switch (dwDataType)
    {
	case NET_DVR_SYSHEAD: 
	    if (!PlayM4_GetPort(&g_lPort))  
	    {
		    break;
	    }
	    //第一次回调的是系统头，将获取的播放库port号赋值给全局port，下次回调数据时即使用此port号播放
	    
	    if (dwBufSize > 0)
	    {
		if (!PlayM4_SetStreamOpenMode(g_lPort, STREAME_REALTIME))  //设置实时流播放模式
		{
			break;
		}

		if (!PlayM4_OpenStream(g_lPort, pBuffer1, dwBufSize, SOURCE_BUF_MAX)) //打开流接口
		{
			break;
		}

		if (!PlayM4_Play(g_lPort, NULL)) //播放开始
		{
			break;
		}
		PlayM4_SetDecCallBack(g_lPort,DecCBFun);

			    
	    }
	case NET_DVR_STREAMDATA:
	    if (dwBufSize > 0 && g_lPort != -1)
	    {
		    if (!PlayM4_InputData(g_lPort, pBuffer1, dwBufSize))
		    {
			break;
		    }
	    }
// 		    printf("时间戳：%d毫秒----------花费总时间：%d毫秒\n", clock()*1000/CLOCKS_PER_SEC, (clock() - start)*1000/CLOCKS_PER_SEC);
    }

}
