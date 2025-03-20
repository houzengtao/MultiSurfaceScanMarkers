#include<opencv2/opencv.hpp>   
#include "librealsense2/rs.hpp" 
#include "example.hpp"  
#include <librealsense2/rsutil.h>
#include "cv-helpers.hpp"
#include <algorithm>            // std::min, std::max
#include <iostream>

#include "stdafx.h"
#include "Com.h"
#include "windows.h"
#include "time.h"
#include "stdio.h"
#include "JY901.h"

using namespace cv;
using namespace std;
using namespace rs2;
void on_mouse(int EVENT, int x, int y, int flags, void* userdata);
float cx = 323.305, cy = 237.231;//D415相机像素640*480@30帧时的参数:中心平移量，单位像素，通过例子程序sensorcontrol获得:Principal point
float fx = 616.924, fy = 616.523;//D415相机像素640*480@30帧时的参数：物理焦距f的倍数，通过例子程序sensorcontrol获得:Focal length
Point startpoint, endpoint;
const int width = 640;
const int height = 480;
const int fps = 30;
bool flag = false;
bool Enterflag = false;
int main()
{
	char chrBuffer[2000];
	unsigned short usLength = 0, usCnt = 0;
	unsigned long ulBaund = 115200, ulComNo = 6;
	signed char cResult = 1;
	float xvalue=0.0, yvalue=0.0, zvalue=0.0;//xvalue绕x轴旋转角度；其它同此
	printf("请输入串口号:\r\nCom = ");
	scanf("%ld", &ulComNo);
//	printf("请输入波特率:(9600、115200或其他)\r\nBaud = ");
//	scanf("%ld", &ulBaund);
	printf("等待打开串口%d...\r\n", ulComNo);
	cResult = OpenCOMDevice(ulComNo, ulBaund);
	
	rs2::align align_to(RS2_STREAM_COLOR);
	//【1】从摄像头读入视频  
	//VideoCapture capture(2);//若测试摄像头有没有打开，0是默认camera，realsense相机的彩色camera是2（看设备管理器，从上往下，第一个为0，第三个为2）
	//if(!capture.isOpened())   {cout<< "cannot open the camera.";cin.get();return -1;} 
	Mat frame; //定义一个Mat变量，用于存储每一帧的图像
	Mat dst;//输出图像 
	Mat bgr;//灰度值归一化  
	Mat hsv; // HSV图像 
	float markerupixel[2] = { 0.0,0.0 }; // From pixel
	float markerupixel2[2] = { 0.0,0.0 };
	float markerupixel3[2] = { 0.0,0.0 };
	float markerudist = 0.0;
	float markerudist2 = 0.0;
	float markerudist3 = 0.0;
	//色相  
	int hmin = 175;
	int hmin_Max = 300;
	int hmax = 300;
	int hmax_Max = 300;
	//饱和度  
	int smin = 90;
	int smin_Max = 255;
	int smax = 255;
	int smax_Max = 255;
	//亮度  
	int vmin = 80;
	int vmin_Max = 255;
	int vmax = 255;
	int vmax_Max = 255;
	FILE *fp;//想用FILE指针，需要在C/C++预编译器中的定义栏加上：_CRT_SECURE_NO_WARNINGS
	FILE *fp2;//颜色文件的
	fp = fopen("pointcloud.txt", "w");
	fp2 = fopen("color.txt", "w");
	// Declare pointcloud object, for calculating pointclouds and texture mappings
	//pointcloud pc = rs2::context().operator std::shared_ptr<rs2_context>//create_pointcloud();
	rs2::pointcloud pc;
	// We want the points object to be persistent so we can display the last cloud when a frame drops
	rs2::points points;
	// Create a Pipeline, which serves as a top-level API for streaming and processing frames
	//pipeline p;
	rs2::pipeline p;//Pipeline
	rs2::config pipe_config;
	pipe_config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
	pipe_config.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
	//	pipe_config.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);
	pipe_config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);

	rs2::pipeline_profile profile = p.start(pipe_config);
	frameset aligned_set2;
	// start the pipeline
	//p.start();
	startpoint.x = 0;
	startpoint.y = 0;
	endpoint.x = 0;
	endpoint.y = 0;
	while (1)
	{
		usLength = CollectUARTData(ulComNo, chrBuffer);
		if (usLength>0)
		{
			JY901.CopeSerialData(chrBuffer, usLength);
		}
		Sleep(10);
		xvalue = (float)JY901.stcAngle.Angle[0] / 32768 * 180;
		yvalue = (float)JY901.stcAngle.Angle[1] / 32768 * 180;
		zvalue = (float)JY901.stcAngle.Angle[2] / 32768 * 180;
//		printf("Angle:%.3f %.3f %.3f\r\n", xvalue, yvalue, zvalue);
		// Block program until frames arrive
		//frameset frames = p.wait_for_frames();
		// Try to get a frame of a depth image
		//depth_frame depth = frames.get_depth_frame();
		frameset data = p.wait_for_frames();
		// Make sure the frameset is spatialy aligned 
		// (each pixel in depth image corresponds to the same pixel in the color image)
		frameset aligned_set = align_to.process(data);
		//aligned_set2 = aligned_set;
		depth_frame depth = aligned_set.get_depth_frame();
		// The frameset might not contain a depth frame, if so continue until it does
		if (!depth) continue;

		auto color = aligned_set.get_color_frame();
		auto color_mat = frame_to_mat(color);
		frame = color_mat;
		// Tell pointcloud object to map to this color frame
	//	pc.map_to(color);
		// Generate the pointcloud and texture mappings
	//	points = pc.calculate(depth);
		//capture >> frame;  //读取当前帧   


		if (frame.empty())
		{
			printf("--(!) No captured frame -- Break!");
			//break;                  
		}
		else
		{

			setMouseCallback("Rect读取", on_mouse, &frame);

			dst = Mat::zeros(frame.size(), CV_32FC3);
			frame.convertTo(bgr, CV_32FC3, 1.0 / 255, 0);
			cvtColor(bgr, hsv, CV_BGR2HSV);
			Mat mask;
			inRange(hsv, Scalar(hmin, smin / float(smin_Max), vmin / float(vmin_Max)), Scalar(hmax, smax / float(smax_Max), vmax / float(vmax_Max)), mask);
			//inRange输出的mask是黑白色图片
			for (int r = 0; r < bgr.rows; r++)
			 {
			 for (int c = 0; c < bgr.cols; c++)
			  {
			  if (mask.at<uchar>(r, c) == 255)
			    {
			    dst.at<Vec3f>(r, c) = bgr.at<Vec3f>(r, c);//dst是只剩下蓝色区域的 彩图
			    }
			  }
			 }
			//输出图像
			//imshow("HSV处理后", mask);//二位图
			imshow("HSV处理后", dst);//只剩下蓝色区域的彩色图
			
			vector<vector<Point> > contours;
			vector<Vec4i> hierarchy;
			findContours(mask, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
			Mat contoursImage(mask.rows, mask.cols, CV_8U, Scalar(0));//Scalar(0)等同于Scalar(0,0,0)黑色
			int markercounter = 0;
			for (int i = 0; i < contours.size(); i++)
			{
				double area = 0, perimeter = 0;
				area = fabs(contourArea(contours[i], false));
				perimeter = arcLength(contours[i], false);
				if (area > 100 && area < 300)
				{
					
					if (3.14 * 4 * (area / perimeter)*(area / perimeter) > area*0.6)//圆度大于0.6比较稳定,0.7以上不稳定
					 {
						markercounter++;
						drawContours(contoursImage, contours, i, Scalar(255, 255, 255), 1);
						Rect R = boundingRect(Mat(contours[i]));
						rectangle(contoursImage, R, Scalar(255), 1, 1, 0);
						rectangle(frame, R, Scalar(255), 1, 1, 0);
						Point pcenter;					
//						float upoint[3]; // From point (in 3D)
						pcenter.x = (R.tl().x + R.br().x) / 2;
						pcenter.y = (R.tl().y + R.br().y) / 2;				
						//画实心点
						circle(contoursImage, pcenter, 3, Scalar(255, 0, 0), -1); //第五个参数我设为-1，表明这是个实点。
						if (markercounter == 1)
						{
							markerupixel[0] = pcenter.x;
							markerupixel[1] = pcenter.y;
							markerudist = depth.get_distance(markerupixel[0], markerupixel[1]);
						}
						else if(markercounter ==2)
						{
							if (abs(pcenter.x -markerupixel[0])>10)//避免重复
							{
								markerupixel2[0] = pcenter.x;
								markerupixel2[1] = pcenter.y;
								markerudist2 = depth.get_distance(markerupixel2[0], markerupixel2[1]);
							}
						}
						else if (markercounter == 3)
					   {
							if (abs(pcenter.x - markerupixel[0])>10)//避免重复
							{
								markerupixel3[0] = pcenter.x;
								markerupixel3[1] = pcenter.y;
								markerudist3 = depth.get_distance(markerupixel3[0], markerupixel3[1]);
							}
					   }
//						std::cout << "upixel:" << markerupixel[0]/640.0 << "," << markerupixel[1]/480.0 << "," << markerudist << "\n";
					 }
				}
			}
			markercounter = 0;
			imshow("识别轮廓图", contoursImage); //显示当前帧
			if (flag)
			  {
				rectangle(frame, startpoint, endpoint, Scalar::all(0), 2, 8, 0);
			  }

			imshow("Rect读取", frame); //显示当前帧  

		}
/*
		// Get the depth frame's dimensions
		//float width = depth.get_width();
		//float height = depth.get_height();
		float width = startpoint.x + endpoint.x;
		float height = startpoint.y + endpoint.y;
		// Query the distance from the camera to the object in the center of the image
		float dist_to_center = depth.get_distance(width / 2, height / 2);
		// Print the distance 
		//std::cout << "The camera is facing an object " << dist_to_center << " meters away \r";
*/
		char keyc = waitKey(30); //等待回车键
		if (keyc == 13) //回车键的ascall码是13
			Enterflag = true;
		if (flag && Enterflag)//画框，左键down后，flag变为false,左键up后flag变为true，如果再按回车键就保存框内的图片
		{
			
			float upixel[2]; // From pixel
			float upoint[3], worldpoint[3] = {0,0,0}, worldpoint2[3] = { 0,0,0 }, worldpoint3[3] = { 0,0,0 }; // point (in 3D)
			uchar* ptr;

			if (markerudist>0.0)
			{
				//计算mark点的空间坐标，以米为单位
				worldpoint[0] = (markerupixel[0] - cx)*markerudist / fx;
				worldpoint[1] = (markerupixel[1] - cy)*markerudist / fy;
				worldpoint[2] = markerudist + 0.006;// 0.006是小球半径，+ 0.006得到的是球心z坐标
				fprintf(fp, "M,%.3f,%.3f,%.3f\n", worldpoint[0], worldpoint[1], worldpoint[2]);
				fprintf(fp2, "M,%.3f,%.3f,%.3f\n", markerupixel[0], markerupixel[1], markerudist);//为了跟点云文件对应，这句在颜色文件中占个坑位而已，没有其他用，所以用原来的语句随便写的，没改
			}
			if (markerudist2 > 0.0)
			{
				//计算mark2点的空间坐标，以米为单位									
				worldpoint2[0] = (markerupixel2[0] - cx)*markerudist2 / fx;
				worldpoint2[1] = (markerupixel2[1] - cy)*markerudist2 / fy;
				worldpoint2[2] = markerudist2 + 0.006;// 0.006是小球半径，+ 0.006得到的是球心z坐标
				fprintf(fp, "N,%.3f,%.3f,%.3f\n", worldpoint2[0], worldpoint2[1], worldpoint2[2]);
				fprintf(fp2, "N,%.3f,%.3f,%.3f\n", markerupixel2[0], markerupixel2[1], markerudist2);//为了跟点云文件对应，这句在颜色文件中占个坑位而已，没有其他用，所以用原来的语句随便写的，没改

			}			
			if (markerudist3 > 0.0)
			{
				//计算mark3点的空间坐标，以米为单位									
				worldpoint3[0] = (markerupixel3[0] - cx)*markerudist3 / fx;
				worldpoint3[1] = (markerupixel3[1] - cy)*markerudist3 / fy;
				worldpoint3[2] = markerudist3 + 0.006;// 0.006是小球半径，+ 0.006得到的是球心z坐标
				fprintf(fp, "O,%.3f,%.3f,%.3f\n", worldpoint3[0], worldpoint3[1], worldpoint3[2]);
				fprintf(fp2, "O,%.3f,%.3f,%.3f\n", markerupixel3[0], markerupixel3[1], markerudist3);//为了跟点云文件对应，这句在颜色文件中占个坑位而已，没有其他用，所以用原来的语句随便写的，没改

			}
			fprintf(fp, "A,%.3f,%.3f,%.3f\n", xvalue, yvalue, zvalue);
			fprintf(fp2, "A,%.3f,%.3f,%.3f\n", xvalue, yvalue, zvalue);//为了跟点云文件对应，句在颜色文件中占个坑位而已，没有其他用，
			//depth_frame depth2 = aligned_set2.get_depth_frame();
			//rs2_intrinsics intr = depth2.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data	
			for (int V = startpoint.y; V <= endpoint.y; V++)//一行一行扫描，从上往下输出
			{
				for (int U = startpoint.x; U <= endpoint.x; U++)
				{
					upixel[0] = U / 640.0;
					upixel[1] = V / 480.0;
			
					auto udist = depth.get_distance(U, V);
					if (udist == 0.0) continue;
					//通用的3D点计算公式，经证实，跟rs2_deproject_pixel_to_point得到的3D点坐标完全相同，也就是说realsense内部函数用的也是此公式
                    //计算此像素点的空间坐标
					worldpoint[0] = (U - cx)*udist / fx;
					worldpoint[1] = (V - cy)*udist / fy;
					worldpoint[2] = udist;
					if (udist>0.2 && udist<2)// 手术对象只能放在在距离镜头0.2到2m之间，可根据实际需求更改手术对象的提取范围
					{
						//					fprintf(fp, "UV坐标(%f,%f) ", upixel[0], upixel[1]);//这里不能用%d，会全成0.
						//					fprintf(fp, " 对应三维(%f,%f,%f)\n", upoint[0], upoint[1], upoint[2]);
						//fprintf(fp, "D,%f,%f,%f\n", upixel[0], upixel[1], udist);
						fprintf(fp, "D,%f,%f,%f\n", worldpoint[0], worldpoint[1], worldpoint[2]);
						//获得该点RGB值即彩色纹理	
						IplImage* img = &(IplImage)frame;
						ptr = cvPtr2D(img, V, U, NULL);//其中y代表y轴（第y行），即height；x代表x轴（第x列），即width
						fprintf(fp2, "D,%f,%f,%f\n", ptr[2] / 255.0, ptr[1] / 255.0, ptr[0] / 255.0);//OPENCV的顺序BGR,所以注意这样排才得到R,G,B
					}
				}
			}
			flag = false;
			Enterflag = false;
		}
		//char c = cvWaitKey(33);//获取键盘输入，也是控制视频的播放速度
		char c = waitKey(10); //延时30ms,上边那句测试过也可以
		if (c == 27) //ESC退出
			break;
	}
/*	if (flag && Enterflag)//跟上段一样也成功了，因tex_coords[i].u和tex_coords[i].v在0.0到1.0之间并是像素UV的单位化，且坐标系跟像素的走向相同
	{
		uchar* ptr;
		auto vertices = points.get_vertices();              // get vertices
		auto tex_coords = points.get_texture_coordinates(); // texture coordinates,它是已经归一的坐标，跟U,V走向一致
		for (int i = 0; i < points.size(); i++)//跟像素走向一致：从左上角开始逐行扫描，然后朝下结束于右下角点
		{
			if (640 * tex_coords[i].u > startpoint.x && 640 * tex_coords[i].u < endpoint.x)
			{
				if (480 * tex_coords[i].v > startpoint.y && 480 * tex_coords[i].v < endpoint.y)
				{
					if (vertices[i].z > 0.2&&vertices[i].z < 2)
					{
						int U = 640 * tex_coords[i].u;
						int V = 480 * tex_coords[i].v;
						fprintf(fp, "UV坐标(%d,%d)", U, V);
						fprintf(fp, "纹理三维(%f,%f,%f) ", tex_coords[i].u, tex_coords[i].v, vertices[i].z);//纹理三维其实就是像素三维
						fprintf(fp, " 对应三维(%f,%f,%f)\n", vertices[i].x, vertices[i].y, vertices[i].z);
						//获得该点RGB值即彩色纹理	
						IplImage* img = &(IplImage)frame;
						ptr = cvPtr2D(img, V, U, NULL);//其中y代表y轴（第y行），即height；x代表x轴（第x列），即width
						fprintf(fp2, "%f,%f,%f\n", ptr[2] / 255.0, ptr[1] / 255.0, ptr[0] / 255.0);//OPENCV的顺序BGR,所以注意这样排才得到R,G,B
					}
				}
			}
		}
		flag = false;
		Enterflag = false;
	}*/


	fclose(fp);
	fclose(fp2);
	cvDestroyWindow("HSV处理后");
	cvDestroyWindow("Rect读取");
//	destroyAllWindows();//销毁全部窗口---destroyWindow（）销毁特定窗口
	return 0;
}
void on_mouse(int EVENT, int x, int y, int flags, void* userdata)
{
	Mat hh;
	hh = *(Mat*)userdata;
	Point p(x, y);
	switch (EVENT)
	{
	case EVENT_LBUTTONDOWN:
	{
		printf("b=%d\t", hh.at<Vec3b>(p)[0]);
		printf("g=%d\t", hh.at<Vec3b>(p)[1]);
		printf("r=%d\n", hh.at<Vec3b>(p)[2]);
		printf("DOWN x=%d\t", x);
		printf("DOWN y=%d\n", y);
		startpoint.x = x;
		startpoint.y = y;
		flag = false;
	}
	case EVENT_LBUTTONUP:
	{
		printf("b=%d\t", hh.at<Vec3b>(p)[0]);
		printf("g=%d\t", hh.at<Vec3b>(p)[1]);
		printf("r=%d\n", hh.at<Vec3b>(p)[2]);
		printf("UP x=%d\t", x);
		printf("UP y=%d\n", y);
		endpoint.x = x;
		endpoint.y = y;
		flag = true;
	}
	break;

  }
}