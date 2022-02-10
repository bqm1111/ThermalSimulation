#include "utils.h"
void writeTofile(std::string filename, double *arr, int length)
{
    std::ofstream file;
    file.open(filename.c_str());
    for(int i = 0; i < length; i++)
    {
        file << std::to_string(arr[i]) + "\n";
    }
    file.close();
}

void writeTofile(std::string filename, double *arr, int line, int numPerline)
{
    std::ofstream file;
    file.open(filename.c_str());
    for(int i = 0; i < line; i++)
    {
        for(int j = 0; j < numPerline; j++)
        {
            file << std::to_string(arr[i * numPerline + j]) + " ";
        }
        file << "\n";
    }
    file.close();

}

bool readFromFile(std::string filename, double *arr, int numLine, int numPerLine)
{
    std::cout << "---------> Loading file " << filename << std::endl;
    std::string line;
    std::ifstream file(filename);

    int lineIdx = 0;
    if(file.good())
    {
        while(getline(file, line) && lineIdx < numLine)
        {
            size_t pos = 0;
            std::string delimeter = " ";
            int cnt= 0;
            while((pos = line.find(' ')) != std::string::npos)
            {
                if(line.substr(0, pos).length() == 0)
                {
                    line.erase(0, pos + delimeter.length());
                    continue;
                }
                arr[numPerLine * lineIdx + cnt] = std::stof(line.substr(0, pos));
                //                std::cout <<"idx = " << numPerLine * lineIdx + cnt << ":" << line.substr(0, pos) << "\n";
                line.erase(0, pos + delimeter.length());
                cnt++;
            }
            arr[numPerLine * lineIdx + cnt] = std::stof(line);
            //            std::cout << "idx = " << numPerLine * lineIdx + cnt << ":" << line << std::endl;
            lineIdx++;
        }

        file.close();
        return true;
    }
    else
    {
        std::cout << "Unable to open file " << filename << std::endl;
        file.close();
        return false;
    }

    return true;
}

void printObjStatus(ObjStatus * obj, int numLine)
{
    for(int i = 0; i < numLine; i++)
    {
        printf("%.8f  %.8f  %.2f  %.4f  %.4f  %.4f\n",   obj[i].gps.latitude ,
               obj[i].gps.longtitude,
               obj[i].gps.height,
               obj[i].angle.roll,
               obj[i].angle.pitch,
               obj[i].angle.yaw );
    }
}

void printSeekerInfo(SeekerInfo * obj, int numLine)
{
    for(int i = 0; i < numLine; i++)
    {
        printf("%.4f  %.4f\n",   obj[i].azimuth ,
               obj[i].elevation);
    }

}

void genRandomMat(cv::Mat src)
{
    cv::RNG rng( 0xFFFFFFFF );

    for(int y = 0; y < src.rows; y++)
    {
        for(int x = 0; x < src.cols; x++)
        {
            src.at<double>(y, x) = rng.uniform(1, 10);
        }
    }
}

__device__ void CheckPoint(int idx)
{
    printf("Check point %d\n", idx);
}
void printMat(double * data, int width, int height)
{
    for(int y = 0; y < height; y++)
    {
        for(int x = 0; x < width; x++)
        {
            printf("%f\t", data[y * width + x]);
        }
        printf("\n");
    }
}

//void printMat(double * data, int width, int height)
//{
//    for(int y = 0; y < height; y++)
//    {
//        for(int x = 0; x < width; x++)
//        {
//            printf("%f\t", data[y * width + x]);
//        }
//        printf("\n");
//    }
//}

void printDevicePtr(double *arr, int width, int height)
{
    double* h_ptr = (double*)malloc(width * height * sizeof(double));
    gpuErrChk(cudaMemcpy(h_ptr, arr, width * height * sizeof(double), cudaMemcpyDeviceToHost));
    printMat(h_ptr, width, height);
    free(h_ptr);
}

void dev2Host(double * dst, double *src, int size)
{
    gpuErrChk(cudaMemcpy(dst, src, size, cudaMemcpyDeviceToHost));
}

void host2Dev(double* dst, double *src, int size)
{
    gpuErrChk(cudaMemcpy(dst, src, size, cudaMemcpyHostToDevice));
}

bool checkEqual(std::string name, double * gt, double *infer, int width, int height, bool show)
{
    std::cout << name;
    for(int y = 0; y < height; y++)
    {
        for(int x = 0; x < width; x++)
        {
            int idx = y * width + x;
            if(gt[idx] - infer[idx] > 0.0001)
            {
                std::cout << ": !!! DO NOT MATCH !!!" << std::endl;
                std::cout << "Error in line " << idx << std::endl;
                if(show)
                {
                    std::cout << "GT = " << std::endl;
                    printMat(gt, width, height);
                    std::cout << "INFER = " << std::endl;
                    printMat(infer, width, height);
                }
                return false;
            }
        }
    }
    std::cout << ": *** MATCH ***" << std::endl;
    return true;
}


//bool checkEqual(std::string name, double * gt, double *infer, int width, int height, bool show)
//{
//    std::cout << name;
//    for(int y = 0; y < height; y++)
//    {
//        for(int x = 0; x < width; x++)
//        {
//            int idx = y * width + x;
//            if(gt[idx] - infer[idx] > 0.0001)
//            {
//                std::cout << ": !!! DO NOT MATCH !!!" << std::endl;
//                if(show)
//                {
//                    std::cout << "GT = " << std::endl;
//                    printMat(gt, width, height);
//                    std::cout << "INFER = " << std::endl;
//                    printMat(infer, width, height);
//                }
//                return false;
//            }
//        }
//    }
//    std::cout << ": *** MATCH ***" << std::endl;
//    return true;
//}

