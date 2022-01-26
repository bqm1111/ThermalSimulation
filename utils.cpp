#include "utils.h"

bool readFromFile(std::string filename, float *arr, int numLine, int numPerLine)
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
            src.at<float>(y, x) = rng.uniform(1, 10);
        }
    }
}

__device__ void CheckPoint(int idx)
{
    printf("Check point %d\n", idx);
}
void printMat(float * data, int width, int height)
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

void printDevicePtr(float *arr, int width, int height)
{
    float* h_ptr = (float*)malloc(width * height * sizeof(float));
    gpuErrChk(cudaMemcpy(h_ptr, arr, width * height * sizeof(float), cudaMemcpyDeviceToHost));
    printMat(h_ptr, width, height);
    free(h_ptr);
}

void dev2Host(float * dst, float *src, int size)
{
    gpuErrChk(cudaMemcpy(dst, src, size, cudaMemcpyDeviceToHost));
}

void host2Dev(float* dst, float *src, int size)
{
    gpuErrChk(cudaMemcpy(dst, src, size, cudaMemcpyHostToDevice));
}

