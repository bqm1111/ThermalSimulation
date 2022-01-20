#include "Simulator.h"

Simulator::Simulator(int fps, int duration, int batch_size):
    m_fps(fps), m_duration(duration), m_batch_size(batch_size)
{
    // camera parameter
    m_width = 640;
    m_height = 480;
    m_fov = 3.7;
    m_fov_pixel = m_width / 2 / tan(deg2rad(m_fov / 2));
    m_current_img_id = 0;

    // thermal parameter
    m_ocean_coeff= 4.5468;
    m_horizon_coeff  = 40.0641;
    m_sky_coeff   = 1.0555;
    m_solar_coeff = 27.7758;
    m_object_coeff = 8.5245;

    // filename
    if(fps == 30)
    {
        m_missile_data_filename = "../data/Missile_30hz.txt";
        m_target_data_filename = "../data/Target_30hz.txt";
        m_seeker_data_filename = "../data/Seeker_30hz.txt";
    }
    else if(fps == 100)
    {
        m_missile_data_filename = "../data/Missile_100hz.txt";
        m_target_data_filename = "../data/Target_100hz.txt";
        m_seeker_data_filename = "../data/Seeker_100hz.txt";
    }
    else
    {
        std::cout << "UNSUPPORTED DATA FOR "<< fps << " FPS" << std::endl;
    }
    m_ship_surfaces_data_filename = "../data/Data_faces.txt";
    m_ship_vertices_data_filename = "../data/Data_vertices.txt";

    // ship parameter
    m_ship.num_surfaces = 4563;
    m_ship.num_vertices = 2295;

    m_totalFrame = m_duration * m_fps;
}

void Simulator::loadData()
{
    ObjStatus *missile_hData, *target_hData;
    SeekerInfo *seeker_hData;
    float3 *ship_surfaces_hData, *ship_vertices_hData;

    missile_hData = (ObjStatus*)malloc(m_totalFrame * sizeof(ObjStatus));
    target_hData = (ObjStatus*)malloc(m_totalFrame * sizeof(ObjStatus));
    seeker_hData = (SeekerInfo*)malloc(m_totalFrame * sizeof(SeekerInfo));
    ship_surfaces_hData = (float3*)malloc(m_ship.num_surfaces * sizeof(float3));
    ship_vertices_hData = (float3*)malloc(m_ship.num_vertices * sizeof(float3));

    readFromFile(m_missile_data_filename, (float*)missile_hData, m_totalFrame, 6);
    readFromFile(m_target_data_filename, (float*)target_hData, m_totalFrame, 6);
    readFromFile(m_seeker_data_filename, (float*)seeker_hData, m_totalFrame, 2);
    readFromFile(m_ship_surfaces_data_filename, (float*)ship_surfaces_hData, m_ship.num_surfaces, 3);
    readFromFile(m_ship_vertices_data_filename, (float*)ship_vertices_hData, m_ship.num_vertices, 3);

    gpuErrChk(cudaMemcpy(m_missile, missile_hData, m_totalFrame * sizeof(ObjStatus), cudaMemcpyHostToDevice));
    gpuErrChk(cudaMemcpy(m_target, target_hData, m_totalFrame * sizeof(ObjStatus), cudaMemcpyHostToDevice));
    gpuErrChk(cudaMemcpy(m_seeker, seeker_hData, m_totalFrame * sizeof(SeekerInfo), cudaMemcpyHostToDevice));
    gpuErrChk(cudaMemcpy(m_ship.surfaces, ship_surfaces_hData, m_ship.num_surfaces * sizeof(float3), cudaMemcpyHostToDevice));
    gpuErrChk(cudaMemcpy(m_ship.vertices, ship_vertices_hData, m_ship.num_vertices * sizeof(float3), cudaMemcpyHostToDevice));

    std::cout << "Load source data: DONE !!!" << std::endl;
    free(missile_hData);
    free(target_hData);
    free(seeker_hData);
    free(ship_surfaces_hData);
    free(ship_vertices_hData);
}

void Simulator::init()
{
    int grid_size = m_ship.num_surfaces * m_batch_size * PIXEL_GRID_SIZE * PIXEL_GRID_SIZE;

    // Allocate source data
    gpuErrChk(cudaMalloc((void**)&m_missile, m_totalFrame * sizeof(ObjStatus)));
    gpuErrChk(cudaMalloc((void**)&m_target, m_totalFrame * sizeof(ObjStatus)));
    gpuErrChk(cudaMalloc((void**)&m_seeker, m_totalFrame * sizeof(SeekerInfo)));
    gpuErrChk(cudaMallocManaged((void**)&m_ship.surfaces, m_ship.num_surfaces * sizeof(float3)));
    gpuErrChk(cudaMallocManaged((void**)&m_ship.vertices, m_ship.num_vertices *sizeof(float3)));
    gpuErrChk(cudaMallocManaged((void**)&m_ship.gps, m_ship.num_vertices *sizeof(GPS)));
    gpuErrChk(cudaMallocManaged((void**)&m_ship.imgPos, m_ship.num_vertices *sizeof(float2)));
    gpuErrChk(cudaMallocManaged((void**)&m_ship.surface_gps, m_ship.num_surfaces * sizeof(GPS3)));
    gpuErrChk(cudaMallocManaged((void**)&m_ship.surface_imgPos, m_ship.num_surfaces * sizeof(float6)));


    // Allocate core data for rendering image
    gpuErrChk(cudaMalloc((void**)&m_ray, grid_size * sizeof(RayInfo)));
    gpuErrChk(cudaMalloc((void**)&m_partialRadiance, m_batch_size * PIXEL_GRID_SIZE * PIXEL_GRID_SIZE * sizeof(float)));
    gpuErrChk(cudaMalloc((void**)&m_renderedImg, m_width * m_height * sizeof(unsigned char)));

    // Allocate memory for transformation matrices
    gpuErrChk(cudaMalloc((void**)&m_Rb2c_cur, 9 * sizeof(float)));
    gpuErrChk(cudaMalloc((void**)&m_Rb2c_prev, 9 * sizeof(float)));
    gpuErrChk(cudaMalloc((void**)&m_Ri2b_missile_cur, 9 * sizeof(float)));
    gpuErrChk(cudaMalloc((void**)&m_Ri2b_missile_prev, 9 * sizeof(float)));
    gpuErrChk(cudaMalloc((void**)&m_Ri2b_target, 9 * sizeof(float)));
    gpuErrChk(cudaMalloc((void**)&m_Re2i_missile, 9 * sizeof(float)));
    gpuErrChk(cudaMalloc((void**)&m_Re2i_target, 9 * sizeof(float)));
}

Simulator::~Simulator()
{
    // Free source data
    gpuErrChk(cudaFree(m_missile));
    gpuErrChk(cudaFree(m_target));
    gpuErrChk(cudaFree(m_seeker));
    gpuErrChk(cudaFree(m_ship.surfaces));
    gpuErrChk(cudaFree(m_ship.vertices));
    gpuErrChk(cudaFree(m_ship.gps));
    gpuErrChk(cudaFree(m_ship.imgPos));
    gpuErrChk(cudaFree(m_ship.surface_gps));
    gpuErrChk(cudaFree(m_ship.surface_imgPos));

    // Free core data for rendering image
    gpuErrChk(cudaFree(m_ray));
    gpuErrChk(cudaFree(m_partialRadiance));
    gpuErrChk(cudaFree(m_renderedImg));

    // Free memory for transformation matrices
    gpuErrChk(cudaFree(m_Rb2c_cur));
    gpuErrChk(cudaFree(m_Rb2c_prev));
    gpuErrChk(cudaFree(m_Ri2b_missile_cur));
    gpuErrChk(cudaFree(m_Ri2b_missile_prev));
    gpuErrChk(cudaFree(m_Ri2b_target));
    gpuErrChk(cudaFree(m_Re2i_missile));
    gpuErrChk(cudaFree(m_Re2i_target));
}

// Reorganizing input data to avoid uncoalesed data access pattern.
// Each element in surface_gps contain 3 gps data of 3 vertices contructing that surface
void Simulator::calcSurfaceData()
{
    for(int i = 0; i < m_ship.num_surfaces; i++)
    {
        m_ship.surface_gps[i].x = m_ship.gps[(int)(m_ship.surfaces->x)];
        m_ship.surface_gps[i].y = m_ship.gps[(int)(m_ship.surfaces->y)];
        m_ship.surface_gps[i].z = m_ship.gps[(int)(m_ship.surfaces->z)];

        m_ship.surface_imgPos[i].x = m_ship.imgPos[(int)(m_ship.surfaces->x)];
        m_ship.surface_imgPos[i].y = m_ship.imgPos[(int)(m_ship.surfaces->y)];
        m_ship.surface_imgPos[i].z = m_ship.imgPos[(int)(m_ship.surfaces->z)];
    }
}

void Simulator::run()
{
    for(int i = 0; i < m_fps * m_duration; i++)
    {
        ObjStatus * missile_cur, *target_cur, *missile_prev, *target_prev;
        SeekerInfo * seeker_cur, *seeker_prev;
        gpuErrChk(cudaMalloc((void**)&missile_cur, sizeof(ObjStatus)));
        gpuErrChk(cudaMalloc((void**)&target_cur, sizeof(ObjStatus)));
        gpuErrChk(cudaMalloc((void**)&missile_prev, sizeof(ObjStatus)));
        gpuErrChk(cudaMalloc((void**)&target_prev, sizeof(ObjStatus)));
        gpuErrChk(cudaMalloc((void**)&seeker_cur, sizeof(SeekerInfo)));
        gpuErrChk(cudaMalloc((void**)&seeker_prev, sizeof(SeekerInfo)));

        gpuErrChk(cudaMemcpy(missile_cur, m_missile + m_current_img_id, sizeof(ObjStatus), cudaMemcpyDeviceToDevice));
        gpuErrChk(cudaMemcpy(target_cur, m_target + m_current_img_id, sizeof(ObjStatus), cudaMemcpyDeviceToDevice));
        gpuErrChk(cudaMemcpy(seeker_cur, m_seeker + m_current_img_id, sizeof(SeekerInfo), cudaMemcpyDeviceToDevice));

        if(m_current_img_id > 0)
        {
            gpuErrChk(cudaMemcpy(missile_prev, m_missile + m_current_img_id - 1, sizeof(ObjStatus), cudaMemcpyDeviceToDevice));
            gpuErrChk(cudaMemcpy(target_prev, m_target + m_current_img_id - 1, sizeof(ObjStatus), cudaMemcpyDeviceToDevice));
            gpuErrChk(cudaMemcpy(seeker_prev, m_seeker + m_current_img_id - 1, sizeof(SeekerInfo), cudaMemcpyDeviceToDevice));
        }
        else
        {
            gpuErrChk(cudaMemcpy(missile_prev, m_missile + m_current_img_id, sizeof(ObjStatus), cudaMemcpyDeviceToDevice));
            gpuErrChk(cudaMemcpy(target_prev, m_target + m_current_img_id, sizeof(ObjStatus), cudaMemcpyDeviceToDevice));
            gpuErrChk(cudaMemcpy(seeker_prev, m_seeker + m_current_img_id, sizeof(SeekerInfo), cudaMemcpyDeviceToDevice));
        }
        printf("****** Start rendering image #%d ******\n", m_current_img_id + 1);
        calcTranformationMatrices(missile_cur, missile_prev, target_cur, target_prev,
                                  seeker_cur, seeker_prev);
        printf("Calculate Transformation Matrices: DONE !!!\n");
        convertToImage(missile_cur, target_cur);
        printf("Convert To Image: DONE!!!\n");
        calcSurfaceData();
        printf("Calculate SurfaceData: DONE!!!\n");

        for(int offset = 0; offset < m_width * m_height / m_batch_size; offset++)
        {
            printf("Rendering image part %d\n", offset);

            getExeTime("calcDistance Time = ", calcDistance(offset, missile_cur));
            getExeTime("calcRadiance Time = ", calcRadiance(offset));
            getExeTime("calcRenderPartialImg Time = ", renderPartialImg(offset));
        }
        cv::Mat img(m_height, m_width, CV_8UC1);
        gpuErrChk(cudaMemcpy(img.data, m_renderedImg, m_width * m_height * sizeof(unsigned char), cudaMemcpyDeviceToHost));
        cv::imwrite("../img/" + std::string(std::to_string(m_current_img_id)) + ".jpg", img);
//        cv::waitKey(2);
        m_current_img_id++;
        gpuErrChk(cudaFree(missile_cur));
        gpuErrChk(cudaFree(target_cur));
        gpuErrChk(cudaFree(missile_prev));
        gpuErrChk(cudaFree(target_prev));
        gpuErrChk(cudaFree(seeker_cur));
        gpuErrChk(cudaFree(seeker_prev));
    }
}
