#include "utils.h"

float deg2rad(float deg)
{
    return deg * M_PI / 180.0;
}

float rad2deg(float rad)
{
    return rad * 180.0 / M_PI;
}

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
