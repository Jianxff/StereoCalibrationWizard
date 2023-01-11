#ifndef __SGBM_UTIL_HPP__
#define __SGBM_UTIL_HPP__
#include <opencv2/opencv.hpp>
#include "sgbm_type.hpp"

static void fillBlank(cv::Mat& image)
{
    const int width = image.cols;
    const int height = image.rows;
    short* data = (short*)image.data;
    cv::Mat integralMap = cv::Mat::zeros(height, width, CV_64F);
    cv::Mat ptsMap = cv::Mat::zeros(height, width, CV_32S);
    double* integral = (double*)integralMap.data;
    int* ptsIntegral = (int*)ptsMap.data;
    //memset(integral, 0, sizeof(double) * width * height);
    //memset(ptsIntegral, 0, sizeof(int) * width * height);
    for (int i = 0; i < height; i++)
    {
        int id1 = i * width;
        for (int j = 0; j < width; j++)
        {
            int id2 = id1 + j;
            if (data[id2] > 1e-3)
            {
                integral[id2] = data[id2];
                ptsIntegral[id2] = 1;
            }
        }
    }

    // 积分区间
    for (int i = 0; i < height; i++)
    {
        int id1 = i * width;
        for (int j = 1; j < width; j++)
        {
            int id2 = id1 + j;
            integral[id2] += integral[id2 - 1];
            ptsIntegral[id2] += ptsIntegral[id2 - 1];
        }
    }

    for (int i = 1; i < height; i++)
    {
        int id1 = i * width;
        for (int j = 0; j < width; j++)
        {
            int id2 = id1 + j;
            integral[id2] += integral[id2 - width];
            ptsIntegral[id2] += ptsIntegral[id2 - width];
        }
    }

    int wnd;
    double dWnd = 2;
    while (dWnd > 1)
    {
        wnd = int(dWnd);
        dWnd /= 2;
        for (int i = 0; i < height; i++)
        {
            int id1 = i * width;
            for (int j = 0; j < width; j++)
            {
                int id2 = id1 + j;
                int left = j - wnd - 1;
                int right = j + wnd;
                int top = i - wnd - 1;
                int bot = i + wnd;
                left = std::max(0, left);
                right = std::min(right, width - 1);
                top = std::max(0, top);
                bot = std::min(bot, height - 1);
                int dx = right - left;
                int dy = (bot - top) * width;
                int idLeftTop = top * width + left;
                int idRightTop = idLeftTop + dx;
                int idLeftBot = idLeftTop + dy;
                int idRightBot = idLeftBot + dx;
                int ptsCnt = ptsIntegral[idRightBot] + ptsIntegral[idLeftTop] - (ptsIntegral[idLeftBot] + ptsIntegral[idRightTop]);
                double sumGray = integral[idRightBot] + integral[idLeftTop] - (integral[idLeftBot] + integral[idRightTop]);
                if (ptsCnt <= 0)
                {
                    continue;
                }
                data[id2] = float(sumGray / ptsCnt);
            }
        }
        int s = wnd / 2 * 2 + 1;
        s = s > 200 ? 200 : s;
        cv::GaussianBlur(image, image, cv::Size(s, s), s, s);
    }
}

// void fillBlank(int width,int height,SGBMOption& option,cv::Mat& img_left,cv::Mat& img_right,cv::Mat& disp)
// {
//     std::vector<std::pair<int, int>> occlusions;
// 	std::vector<std::pair<int, int>> mismatches;


//     /* LRCheck */
//     for (int i = 0; i < height; i++) {
//         for (int j = 0; j < width; j++) {
//             // 左影像视差值
//             auto& disp_data = img_left.data[i * width + j];
//             if(disp_data == std::numeric_limits<float>::infinity()){
//                 mismatches.emplace_back(i, j);
//                 continue;
//             }

//             // 根据视差值找到右影像上对应的同名像素
//             const auto col_right = static_cast<int>(j - disp_data + 0.5);
            
//             if(col_right >= 0 && col_right < width) {
//                 // 右影像上同名像素的视差值
//                 const auto& disp_r = img_right.data[i * width + col_right];
                
//                 // 判断两个视差值是否一致（差值在阈值内）
//                 if (abs(disp_data - disp_r) > 1.0f) {
//                     // 区分遮挡区和误匹配区
//                     // 通过右影像视差算出在左影像的匹配像素，并获取视差disp_rl
//                     // if(disp_rl > disp) 
//                     //		pixel in occlusions
//                     // else 
//                     //		pixel in mismatches
//                     const int col_rl = static_cast<int>(col_right + disp_r + 0.5);
//                     if(col_rl > 0 && col_rl < width){
//                         const auto& disp_l = img_left.data[i*width + col_rl];
//                         if(disp_l > disp_data) {
//                             occlusions.emplace_back(i, j);
//                         }
//                         else {
//                             mismatches.emplace_back(i, j);
//                         }
//                     }
//                     else{
//                         mismatches.emplace_back(i, j);
//                     }

//                     // 让视差值无效
//                     disp = std::numeric_limits<float>::infinity();
//                 }
//             }
//             else{
//                 // 通过视差值在右影像上找不到同名像素（超出影像范围）
//                 disp = std::numeric_limits<float>::infinity();
//                 mismatches.emplace_back(i, j);
//             }
//         }
//     }

//     std::vector<float> disp_collects;

//     // 定义8个方向
//     const float pi = 3.1415926f;
//     float angle1[8] = { pi, 3 * pi / 4, pi / 2, pi / 4, 0, 7 * pi / 4, 3 * pi / 2, 5 * pi / 4 };
//     float angle2[8] = { pi, 5 * pi / 4, 3 * pi / 2, 7 * pi / 4, 0, pi / 4, pi / 2, 3 * pi / 4 };
//     float *angle = angle1;
//     // 最大搜索行程，没有必要搜索过远的像素
//     const int max_search_length = std::max(abs(option.num_disparity), abs(option.min_disparity));

//     float* disp_ptr = (float*)disp.data;
//     for (int k = 0; k < 3; k++) {
//         // 第一次循环处理遮挡区，第二次循环处理误匹配区
//         auto& trg_pixels = (k == 0) ? occlusions : mismatches;
//         if (trg_pixels.empty()) {
//             continue;
//         }
//         std::vector<float> fill_disps(trg_pixels.size());
//         std::vector<std::pair<int, int>> inv_pixels;
//         if (k == 2) {
//             //  第三次循环处理前两次没有处理干净的像素
//             for (int i = 0; i < height; i++) {
//                 for (int j = 0; j < width; j++) {
//                     if (disp_ptr[i * width + j] == std::numeric_limits<float>::infinity()) {
//                         inv_pixels.emplace_back(i, j);
//                     }
//                 }
//             }
//             trg_pixels = inv_pixels;
//         }

//         // 遍历待处理像素
//         for (auto n = 0u; n < trg_pixels.size(); n++) {
//             auto& pix = trg_pixels[n];
//             const int y = pix.first;
//             const int x = pix.second;

//             if (y == height / 2) {
//                 angle = angle2; 
//             }

//             // 收集8个方向上遇到的首个有效视差值
//             disp_collects.clear();
//             for (int s = 0; s < 8; s++) {
//                 const float ang = angle[s];
//                 const float sina = float(sin(ang));
//                 const float cosa = float(cos(ang));
//                 for (int m = 1; m < max_search_length; m++) {
//                     const int yy = lround(y + m * sina);
//                     const int xx = lround(x + m * cosa);
//                     if (yy<0 || yy >= height || xx<0 || xx >= width) {
//                         break;
//                     }
//                     const auto& disp = *(disp_ptr + yy*width + xx);
//                     if (disp != std::numeric_limits<float>::infinity()) {
//                         disp_collects.push_back(disp);
//                         break;
//                     }
//                 }
//             }
//             if(disp_collects.empty()) {
//                 continue;
//             }

//             std::sort(disp_collects.begin(), disp_collects.end());

//             // 如果是遮挡区，则选择第二小的视差值
//             // 如果是误匹配区，则选择中值
//             if (k == 0) {
//                 if (disp_collects.size() > 1) {
//                     fill_disps[n] = disp_collects[1];
//                 }
//                 else {
//                     fill_disps[n] = disp_collects[0];
//                 }
//             }
//             else{
//                 fill_disps[n] = disp_collects[disp_collects.size() / 2];
//             }
//         }
//         for (auto n = 0u; n < trg_pixels.size(); n++) {
//             auto& pix = trg_pixels[n];
//             const int y = pix.first;
//             const int x = pix.second;
//             disp_ptr[y * width + x] = fill_disps[n];
//         }
//     }
// }
#endif;