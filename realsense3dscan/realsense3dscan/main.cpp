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
float cx = 323.305, cy = 237.231;//D415�������640*480@30֡ʱ�Ĳ���:����ƽ��������λ���أ�ͨ�����ӳ���sensorcontrol���:Principal point
float fx = 616.924, fy = 616.523;//D415�������640*480@30֡ʱ�Ĳ�����������f�ı�����ͨ�����ӳ���sensorcontrol���:Focal length
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
	float xvalue=0.0, yvalue=0.0, zvalue=0.0;//xvalue��x����ת�Ƕȣ�����ͬ��
	printf("�����봮�ں�:\r\nCom = ");
	scanf("%ld", &ulComNo);
//	printf("�����벨����:(9600��115200������)\r\nBaud = ");
//	scanf("%ld", &ulBaund);
	printf("�ȴ��򿪴���%d...\r\n", ulComNo);
	cResult = OpenCOMDevice(ulComNo, ulBaund);
	
	rs2::align align_to(RS2_STREAM_COLOR);
	//��1��������ͷ������Ƶ  
	//VideoCapture capture(2);//����������ͷ��û�д򿪣�0��Ĭ��camera��realsense����Ĳ�ɫcamera��2�����豸���������������£���һ��Ϊ0��������Ϊ2��
	//if(!capture.isOpened())   {cout<< "cannot open the camera.";cin.get();return -1;} 
	Mat frame; //����һ��Mat���������ڴ洢ÿһ֡��ͼ��
	Mat dst;//���ͼ�� 
	Mat bgr;//�Ҷ�ֵ��һ��  
	Mat hsv; // HSVͼ�� 
	float markerupixel[2] = { 0.0,0.0 }; // From pixel
	float markerupixel2[2] = { 0.0,0.0 };
	float markerupixel3[2] = { 0.0,0.0 };
	float markerudist = 0.0;
	float markerudist2 = 0.0;
	float markerudist3 = 0.0;
	//ɫ��  
	int hmin = 175;
	int hmin_Max = 300;
	int hmax = 300;
	int hmax_Max = 300;
	//���Ͷ�  
	int smin = 90;
	int smin_Max = 255;
	int smax = 255;
	int smax_Max = 255;
	//����  
	int vmin = 80;
	int vmin_Max = 255;
	int vmax = 255;
	int vmax_Max = 255;
	FILE *fp;//����FILEָ�룬��Ҫ��C/C++Ԥ�������еĶ��������ϣ�_CRT_SECURE_NO_WARNINGS
	FILE *fp2;//��ɫ�ļ���
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
		//capture >> frame;  //��ȡ��ǰ֡   


		if (frame.empty())
		{
			printf("--(!) No captured frame -- Break!");
			//break;                  
		}
		else
		{

			setMouseCallback("Rect��ȡ", on_mouse, &frame);

			dst = Mat::zeros(frame.size(), CV_32FC3);
			frame.convertTo(bgr, CV_32FC3, 1.0 / 255, 0);
			cvtColor(bgr, hsv, CV_BGR2HSV);
			Mat mask;
			inRange(hsv, Scalar(hmin, smin / float(smin_Max), vmin / float(vmin_Max)), Scalar(hmax, smax / float(smax_Max), vmax / float(vmax_Max)), mask);
			//inRange�����mask�Ǻڰ�ɫͼƬ
			for (int r = 0; r < bgr.rows; r++)
			 {
			 for (int c = 0; c < bgr.cols; c++)
			  {
			  if (mask.at<uchar>(r, c) == 255)
			    {
			    dst.at<Vec3f>(r, c) = bgr.at<Vec3f>(r, c);//dst��ֻʣ����ɫ����� ��ͼ
			    }
			  }
			 }
			//���ͼ��
			//imshow("HSV�����", mask);//��λͼ
			imshow("HSV�����", dst);//ֻʣ����ɫ����Ĳ�ɫͼ
			
			vector<vector<Point> > contours;
			vector<Vec4i> hierarchy;
			findContours(mask, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
			Mat contoursImage(mask.rows, mask.cols, CV_8U, Scalar(0));//Scalar(0)��ͬ��Scalar(0,0,0)��ɫ
			int markercounter = 0;
			for (int i = 0; i < contours.size(); i++)
			{
				double area = 0, perimeter = 0;
				area = fabs(contourArea(contours[i], false));
				perimeter = arcLength(contours[i], false);
				if (area > 100 && area < 300)
				{
					
					if (3.14 * 4 * (area / perimeter)*(area / perimeter) > area*0.6)//Բ�ȴ���0.6�Ƚ��ȶ�,0.7���ϲ��ȶ�
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
						//��ʵ�ĵ�
						circle(contoursImage, pcenter, 3, Scalar(255, 0, 0), -1); //�������������Ϊ-1���������Ǹ�ʵ�㡣
						if (markercounter == 1)
						{
							markerupixel[0] = pcenter.x;
							markerupixel[1] = pcenter.y;
							markerudist = depth.get_distance(markerupixel[0], markerupixel[1]);
						}
						else if(markercounter ==2)
						{
							if (abs(pcenter.x -markerupixel[0])>10)//�����ظ�
							{
								markerupixel2[0] = pcenter.x;
								markerupixel2[1] = pcenter.y;
								markerudist2 = depth.get_distance(markerupixel2[0], markerupixel2[1]);
							}
						}
						else if (markercounter == 3)
					   {
							if (abs(pcenter.x - markerupixel[0])>10)//�����ظ�
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
			imshow("ʶ������ͼ", contoursImage); //��ʾ��ǰ֡
			if (flag)
			  {
				rectangle(frame, startpoint, endpoint, Scalar::all(0), 2, 8, 0);
			  }

			imshow("Rect��ȡ", frame); //��ʾ��ǰ֡  

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
		char keyc = waitKey(30); //�ȴ��س���
		if (keyc == 13) //�س�����ascall����13
			Enterflag = true;
		if (flag && Enterflag)//�������down��flag��Ϊfalse,���up��flag��Ϊtrue������ٰ��س����ͱ�����ڵ�ͼƬ
		{
			
			float upixel[2]; // From pixel
			float upoint[3], worldpoint[3] = {0,0,0}, worldpoint2[3] = { 0,0,0 }, worldpoint3[3] = { 0,0,0 }; // point (in 3D)
			uchar* ptr;

			if (markerudist>0.0)
			{
				//����mark��Ŀռ����꣬����Ϊ��λ
				worldpoint[0] = (markerupixel[0] - cx)*markerudist / fx;
				worldpoint[1] = (markerupixel[1] - cy)*markerudist / fy;
				worldpoint[2] = markerudist + 0.006;// 0.006��С��뾶��+ 0.006�õ���������z����
				fprintf(fp, "M,%.3f,%.3f,%.3f\n", worldpoint[0], worldpoint[1], worldpoint[2]);
				fprintf(fp2, "M,%.3f,%.3f,%.3f\n", markerupixel[0], markerupixel[1], markerudist);//Ϊ�˸������ļ���Ӧ���������ɫ�ļ���ռ����λ���ѣ�û�������ã�������ԭ����������д�ģ�û��
			}
			if (markerudist2 > 0.0)
			{
				//����mark2��Ŀռ����꣬����Ϊ��λ									
				worldpoint2[0] = (markerupixel2[0] - cx)*markerudist2 / fx;
				worldpoint2[1] = (markerupixel2[1] - cy)*markerudist2 / fy;
				worldpoint2[2] = markerudist2 + 0.006;// 0.006��С��뾶��+ 0.006�õ���������z����
				fprintf(fp, "N,%.3f,%.3f,%.3f\n", worldpoint2[0], worldpoint2[1], worldpoint2[2]);
				fprintf(fp2, "N,%.3f,%.3f,%.3f\n", markerupixel2[0], markerupixel2[1], markerudist2);//Ϊ�˸������ļ���Ӧ���������ɫ�ļ���ռ����λ���ѣ�û�������ã�������ԭ����������д�ģ�û��

			}			
			if (markerudist3 > 0.0)
			{
				//����mark3��Ŀռ����꣬����Ϊ��λ									
				worldpoint3[0] = (markerupixel3[0] - cx)*markerudist3 / fx;
				worldpoint3[1] = (markerupixel3[1] - cy)*markerudist3 / fy;
				worldpoint3[2] = markerudist3 + 0.006;// 0.006��С��뾶��+ 0.006�õ���������z����
				fprintf(fp, "O,%.3f,%.3f,%.3f\n", worldpoint3[0], worldpoint3[1], worldpoint3[2]);
				fprintf(fp2, "O,%.3f,%.3f,%.3f\n", markerupixel3[0], markerupixel3[1], markerudist3);//Ϊ�˸������ļ���Ӧ���������ɫ�ļ���ռ����λ���ѣ�û�������ã�������ԭ����������д�ģ�û��

			}
			fprintf(fp, "A,%.3f,%.3f,%.3f\n", xvalue, yvalue, zvalue);
			fprintf(fp2, "A,%.3f,%.3f,%.3f\n", xvalue, yvalue, zvalue);//Ϊ�˸������ļ���Ӧ��������ɫ�ļ���ռ����λ���ѣ�û�������ã�
			//depth_frame depth2 = aligned_set2.get_depth_frame();
			//rs2_intrinsics intr = depth2.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data	
			for (int V = startpoint.y; V <= endpoint.y; V++)//һ��һ��ɨ�裬�����������
			{
				for (int U = startpoint.x; U <= endpoint.x; U++)
				{
					upixel[0] = U / 640.0;
					upixel[1] = V / 480.0;
			
					auto udist = depth.get_distance(U, V);
					if (udist == 0.0) continue;
					//ͨ�õ�3D����㹫ʽ����֤ʵ����rs2_deproject_pixel_to_point�õ���3D��������ȫ��ͬ��Ҳ����˵realsense�ڲ������õ�Ҳ�Ǵ˹�ʽ
                    //��������ص�Ŀռ�����
					worldpoint[0] = (U - cx)*udist / fx;
					worldpoint[1] = (V - cy)*udist / fy;
					worldpoint[2] = udist;
					if (udist>0.2 && udist<2)// ��������ֻ�ܷ����ھ��뾵ͷ0.2��2m֮�䣬�ɸ���ʵ��������������������ȡ��Χ
					{
						//					fprintf(fp, "UV����(%f,%f) ", upixel[0], upixel[1]);//���ﲻ����%d����ȫ��0.
						//					fprintf(fp, " ��Ӧ��ά(%f,%f,%f)\n", upoint[0], upoint[1], upoint[2]);
						//fprintf(fp, "D,%f,%f,%f\n", upixel[0], upixel[1], udist);
						fprintf(fp, "D,%f,%f,%f\n", worldpoint[0], worldpoint[1], worldpoint[2]);
						//��øõ�RGBֵ����ɫ����	
						IplImage* img = &(IplImage)frame;
						ptr = cvPtr2D(img, V, U, NULL);//����y����y�ᣨ��y�У�����height��x����x�ᣨ��x�У�����width
						fprintf(fp2, "D,%f,%f,%f\n", ptr[2] / 255.0, ptr[1] / 255.0, ptr[0] / 255.0);//OPENCV��˳��BGR,����ע�������Ųŵõ�R,G,B
					}
				}
			}
			flag = false;
			Enterflag = false;
		}
		//char c = cvWaitKey(33);//��ȡ�������룬Ҳ�ǿ�����Ƶ�Ĳ����ٶ�
		char c = waitKey(10); //��ʱ30ms,�ϱ��Ǿ���Թ�Ҳ����
		if (c == 27) //ESC�˳�
			break;
	}
/*	if (flag && Enterflag)//���϶�һ��Ҳ�ɹ��ˣ���tex_coords[i].u��tex_coords[i].v��0.0��1.0֮�䲢������UV�ĵ�λ����������ϵ�����ص�������ͬ
	{
		uchar* ptr;
		auto vertices = points.get_vertices();              // get vertices
		auto tex_coords = points.get_texture_coordinates(); // texture coordinates,�����Ѿ���һ�����꣬��U,V����һ��
		for (int i = 0; i < points.size(); i++)//����������һ�£������Ͻǿ�ʼ����ɨ�裬Ȼ���½��������½ǵ�
		{
			if (640 * tex_coords[i].u > startpoint.x && 640 * tex_coords[i].u < endpoint.x)
			{
				if (480 * tex_coords[i].v > startpoint.y && 480 * tex_coords[i].v < endpoint.y)
				{
					if (vertices[i].z > 0.2&&vertices[i].z < 2)
					{
						int U = 640 * tex_coords[i].u;
						int V = 480 * tex_coords[i].v;
						fprintf(fp, "UV����(%d,%d)", U, V);
						fprintf(fp, "������ά(%f,%f,%f) ", tex_coords[i].u, tex_coords[i].v, vertices[i].z);//������ά��ʵ����������ά
						fprintf(fp, " ��Ӧ��ά(%f,%f,%f)\n", vertices[i].x, vertices[i].y, vertices[i].z);
						//��øõ�RGBֵ����ɫ����	
						IplImage* img = &(IplImage)frame;
						ptr = cvPtr2D(img, V, U, NULL);//����y����y�ᣨ��y�У�����height��x����x�ᣨ��x�У�����width
						fprintf(fp2, "%f,%f,%f\n", ptr[2] / 255.0, ptr[1] / 255.0, ptr[0] / 255.0);//OPENCV��˳��BGR,����ע�������Ųŵõ�R,G,B
					}
				}
			}
		}
		flag = false;
		Enterflag = false;
	}*/


	fclose(fp);
	fclose(fp2);
	cvDestroyWindow("HSV�����");
	cvDestroyWindow("Rect��ȡ");
//	destroyAllWindows();//����ȫ������---destroyWindow���������ض�����
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