#include "Simulator.h"

Simulator::Simulator(int fps, int duration, int batch_size):
    m_fps(fps), m_duration(duration), m_batch_size(batch_size)
{
    // camera parameter
    m_width = 640;
    m_height = 480;
    m_fov = 3.7;
    m_fov_pixel = m_width / 2 / tan(m_fov / 2 * M_PI / 180);
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
    m_ship_surfaces_data_filename = "../test_data/ship_data_face.txt";
    m_ship_vertices_data_filename = "../test_data/ship_data_vertices.txt";

    // ship parameter
    m_ship.num_surfaces = 4563;
    m_ship.num_vertices = 2295;

    m_totalFrame = m_duration * m_fps;
}

void Simulator::loadData()
{
    ObjStatus *missile_hData, *target_hData;
    SeekerInfo *seeker_hData;
    double3 *ship_surfaces_hData, *ship_vertices_hData;

    missile_hData = (ObjStatus*)malloc(m_totalFrame * sizeof(ObjStatus));
    target_hData = (ObjStatus*)malloc(m_totalFrame * sizeof(ObjStatus));
    seeker_hData = (SeekerInfo*)malloc(m_totalFrame * sizeof(SeekerInfo));
    ship_surfaces_hData = (double3*)malloc(m_ship.num_surfaces * sizeof(double3));
    ship_vertices_hData = (double3*)malloc(m_ship.num_vertices * sizeof(double3));

    readFromFile(m_missile_data_filename, (double*)missile_hData, m_totalFrame, 6);
    readFromFile(m_target_data_filename, (double*)target_hData, m_totalFrame, 6);
    readFromFile(m_seeker_data_filename, (double*)seeker_hData, m_totalFrame, 2);
    readFromFile(m_ship_surfaces_data_filename, (double*)ship_surfaces_hData, m_ship.num_surfaces, 3);
    readFromFile(m_ship_vertices_data_filename, (double*)ship_vertices_hData, m_ship.num_vertices, 3);

    gpuErrChk(cudaMemcpy(m_missile, missile_hData, m_totalFrame * sizeof(ObjStatus), cudaMemcpyHostToDevice));
    gpuErrChk(cudaMemcpy(m_target, target_hData, m_totalFrame * sizeof(ObjStatus), cudaMemcpyHostToDevice));
    gpuErrChk(cudaMemcpy(m_seeker, seeker_hData, m_totalFrame * sizeof(SeekerInfo), cudaMemcpyHostToDevice));
    gpuErrChk(cudaMemcpy(m_ship.surfaces, ship_surfaces_hData, m_ship.num_surfaces * sizeof(double3), cudaMemcpyHostToDevice));
    gpuErrChk(cudaMemcpy(m_ship.vertices, ship_vertices_hData, m_ship.num_vertices * sizeof(double3), cudaMemcpyHostToDevice));

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
    gpuErrChk(cudaMallocManaged((void**)&m_ship.surfaces, m_ship.num_surfaces * sizeof(double3)));
    gpuErrChk(cudaMallocManaged((void**)&m_ship.vertices, m_ship.num_vertices *sizeof(double3)));
    gpuErrChk(cudaMallocManaged((void**)&m_ship.gps, m_ship.num_vertices *sizeof(GPS)));
    gpuErrChk(cudaMallocManaged((void**)&m_ship.imgPos, m_ship.num_vertices *sizeof(double2)));
    gpuErrChk(cudaMallocManaged((void**)&m_ship.surface_gps, m_ship.num_surfaces * sizeof(GPS3)));
    gpuErrChk(cudaMallocManaged((void**)&m_ship.surface_imgPos, m_ship.num_surfaces * sizeof(double6)));

    // Allocate core data for rendering image
    //    gpuErrChk(cudaMalloc((void**)&m_ray, grid_size * sizeof(RayInfo)));
    gpuErrChk(cudaMalloc((void**)&(m_ray.angle), grid_size * sizeof(double)));
    gpuErrChk(cudaMalloc((void**)&(m_ray.distance), grid_size * sizeof(double)));
    gpuErrChk(cudaMalloc((void**)&(m_ray.objIdx), grid_size * sizeof(int)));

    gpuErrChk(cudaMalloc((void**)&m_partialRadiance, m_batch_size * PIXEL_GRID_SIZE * PIXEL_GRID_SIZE * sizeof(double)));
    gpuErrChk(cudaMalloc((void**)&m_renderedImg, m_width * m_height * sizeof(unsigned char)));
    gpuErrChk(cudaMalloc((void**)&m_maskImg, m_width * m_height * sizeof(unsigned char)));

    gpuErrChk(cudaMemset(m_maskImg, 0, m_width * m_height * sizeof(unsigned char)));
    // Allocate memory for transformation matrices
    gpuErrChk(cudaMallocManaged((void**)&m_Rb2c_cur, 9 * sizeof(double)));
    gpuErrChk(cudaMallocManaged((void**)&m_Rb2c_prev, 9 * sizeof(double)));
    gpuErrChk(cudaMallocManaged((void**)&m_Ri2b_missile_cur, 9 * sizeof(double)));
    gpuErrChk(cudaMallocManaged((void**)&m_Ri2b_missile_prev, 9 * sizeof(double)));
    gpuErrChk(cudaMallocManaged((void**)&m_Ri2b_target, 9 * sizeof(double)));
    gpuErrChk(cudaMallocManaged((void**)&m_Re2i_missile, 9 * sizeof(double)));
    gpuErrChk(cudaMallocManaged((void**)&m_Re2i_target, 9 * sizeof(double)));
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
    //    gpuErrChk(cudaFree(m_ray));
    gpuErrChk(cudaFree(m_ray.angle));
    gpuErrChk(cudaFree(m_ray.distance));
    gpuErrChk(cudaFree(m_ray.objIdx));

    gpuErrChk(cudaFree(m_partialRadiance));
    gpuErrChk(cudaFree(m_renderedImg));
    gpuErrChk(cudaFree(m_maskImg));

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
        m_ship.surface_gps[i].x = m_ship.gps[(int)(m_ship.surfaces[i].x - 1)];
        m_ship.surface_gps[i].y = m_ship.gps[(int)(m_ship.surfaces[i].y - 1)];
        m_ship.surface_gps[i].z = m_ship.gps[(int)(m_ship.surfaces[i].z - 1)];

        m_ship.surface_imgPos[i].x = m_ship.imgPos[(int)(m_ship.surfaces[i].x - 1)];
        m_ship.surface_imgPos[i].y = m_ship.imgPos[(int)(m_ship.surfaces[i].y - 1)];
        m_ship.surface_imgPos[i].z = m_ship.imgPos[(int)(m_ship.surfaces[i].z - 1)];
    }
}

void Simulator::run()
{
    ObjStatus * missile_cur, *target_cur, *missile_prev, *target_prev;
    SeekerInfo * seeker_cur, *seeker_prev;
    gpuErrChk(cudaMalloc((void**)&missile_cur, sizeof(ObjStatus)));
    gpuErrChk(cudaMalloc((void**)&target_cur, sizeof(ObjStatus)));
    gpuErrChk(cudaMalloc((void**)&missile_prev, sizeof(ObjStatus)));
    gpuErrChk(cudaMalloc((void**)&target_prev, sizeof(ObjStatus)));
    gpuErrChk(cudaMalloc((void**)&seeker_cur, sizeof(SeekerInfo)));
    gpuErrChk(cudaMalloc((void**)&seeker_prev, sizeof(SeekerInfo)));

    for(int m_current_img_id = 0; m_current_img_id < m_fps * m_duration; m_current_img_id++)
    {
        auto start = getMoment;

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
            calcDistance(offset, missile_cur);
            calcRadiance(offset);
            renderPartialImg(offset);
        }

        cv::Mat img(m_height, m_width, CV_8UC1);
        gpuErrChk(cudaMemcpy(img.data, m_renderedImg, m_width * m_height * sizeof(unsigned char), cudaMemcpyDeviceToHost));
        cv::imwrite("../img/" + std::string(std::to_string(m_current_img_id)) + ".jpg", img);
        auto end = getMoment;
        getTimeElapsed("Elapsed Time = ", end, start);
    }

    gpuErrChk(cudaFree(missile_cur));
    gpuErrChk(cudaFree(target_cur));
    gpuErrChk(cudaFree(missile_prev));
    gpuErrChk(cudaFree(target_prev));
    gpuErrChk(cudaFree(seeker_cur));
    gpuErrChk(cudaFree(seeker_prev));
}

void Simulator::testCalcShipData(ObjStatus * missile, ObjStatus * target)
{
    GPS *ship_gps_gt;               // gps data of each vertex
    double2 *ship_imgPos_gt;          // pixel position of each vertex when projecting onto the image

    GPS3 * ship_surface_gps_gt;
    double6 * ship_surface_imgPos_gt;
    gpuErrChk(cudaMallocManaged((void**)&ship_gps_gt, m_ship.num_vertices *sizeof(GPS)));
    gpuErrChk(cudaMallocManaged((void**)&ship_imgPos_gt, m_ship.num_vertices *sizeof(double2)));
    gpuErrChk(cudaMallocManaged((void**)&ship_surface_gps_gt, m_ship.num_surfaces * sizeof(GPS3)));
    gpuErrChk(cudaMallocManaged((void**)&ship_surface_imgPos_gt, m_ship.num_surfaces * sizeof(double6)));

    readFromFile("../reference/ship_face_gps_pos.txt", (double*)ship_surface_gps_gt, m_ship.num_surfaces, 9);
    readFromFile("../reference/ship_face_image_pos.txt", (double*)ship_surface_imgPos_gt, m_ship.num_surfaces, 6);

    getExeTime("convert to image time = ", convertToImage(missile, target));

    //    checkEqual("Test ship_GPS", (double*)ship_gps_gt, (double*)m_ship.gps, 3, m_ship.num_vertices, false);
    //    checkEqual("Test ship_imgPos", (double*)ship_imgPos_gt, (double*)m_ship.imgPos, 2, m_ship.num_vertices, false);

    printf("Convert To Image: DONE!!!\n");
    getExeTime("calcSurfaceData = ", calcSurfaceData());

    GPS3 * h_ship_face_gps = (GPS3*)malloc(m_ship.num_surfaces * sizeof(GPS3));
    double6* h_ship_face_imgPos = (double6*)malloc(m_ship.num_surfaces * sizeof(double6));

    gpuErrChk(cudaMemcpy(h_ship_face_gps, m_ship.surface_gps, m_ship.num_surfaces * sizeof(GPS3), cudaMemcpyDeviceToHost));
    gpuErrChk(cudaMemcpy(h_ship_face_imgPos, m_ship.surface_imgPos, m_ship.num_surfaces * sizeof(double6), cudaMemcpyDeviceToHost));

    writeTofile("../log/ship_face_gps.txt", (double*)h_ship_face_gps, m_ship.num_surfaces, 9);
    writeTofile("../log/ship_face_imgPos.txt", (double*)h_ship_face_imgPos, m_ship.num_surfaces, 6);

    free(h_ship_face_gps);
    free(h_ship_face_imgPos);

    checkEqual("Test ship_surface_GPS", (double*)ship_surface_gps_gt, (double*)m_ship.surface_gps, 9, m_ship.num_surfaces, false);
    checkEqual("Test ship_surface_imgPos", (double*)ship_surface_imgPos_gt, (double*)m_ship.surface_imgPos, 6, m_ship.num_surfaces, false);

    printf("Calculate SurfaceData: DONE!!!\n");

    gpuErrChk(cudaFree(ship_gps_gt));
    gpuErrChk(cudaFree(ship_imgPos_gt));
    gpuErrChk(cudaFree(ship_surface_gps_gt));
    gpuErrChk(cudaFree(ship_surface_imgPos_gt));
}

void Simulator::test()
{
    ObjStatus *missile;
    ObjStatus *target;
    SeekerInfo *seeker;

    ObjStatus missile_data(GPS(19.09816472762636, 106.0423359433193, 6.330028288997710), RotationAngle(-0.000064, 0.054904570813033, 1.277504088548353));
    ObjStatus target_data(GPS(19.11172390, 106.08994220, 6.0000), RotationAngle(0.0000, 0.0000, 2.8483));
    SeekerInfo seeker_data(0.0000, -0.054904570813033);

    gpuErrChk(cudaMalloc((void**)&missile, sizeof(ObjStatus)));
    gpuErrChk(cudaMalloc((void**)&target, sizeof(ObjStatus)));
    gpuErrChk(cudaMalloc((void**)&seeker, sizeof(SeekerInfo)));

    gpuErrChk(cudaMemcpy(missile, &missile_data, sizeof(ObjStatus), cudaMemcpyHostToDevice));
    gpuErrChk(cudaMemcpy(target, &target_data, sizeof(ObjStatus), cudaMemcpyHostToDevice));
    gpuErrChk(cudaMemcpy(seeker, &seeker_data, sizeof(SeekerInfo), cudaMemcpyHostToDevice));

    cv::Mat Rb2c_cur_gt = (cv::Mat_<double>(3, 3) << 0,	-0.0549,	0.9985,
                           1,	0,	0,
                           0,	0.9985,	0.0549);

    cv::Mat Ri2b_missile_gt = (cv::Mat_<double>(3, 3) << 0.2887,	-0.9573,	0.0158,
                               0.9559,	0.2891,	0.0526,
                               -0.0549,	-6.39120480315225e-05,	0.9985);

    cv::Mat Ri2b_target_gt = (cv::Mat_<double>(3, 3) << -0.9573,	-0.2891,	0,
                              0.2891,	-0.9573,	0,
                              0,	0,	1);


    cv::Mat Re2i_missile_gt = (cv::Mat_<double>(3, 3) << 0.0904,	-0.3144,	0.9450,
                               -0.9611,	-0.2763,	0,
                               0.2611,	-0.9082,	-0.3272);
    cv::Mat Re2i_target_gt = (cv::Mat_<double>(3, 3) << 0.0907,	-0.3146,	0.9449,
                              -0.9608,	-0.2771,	0,
                              0.2619,	-0.9079,	-0.3274);

    printf("****** Start rendering image #%d ******\n", m_current_img_id + 1);
    getExeTime("calcMatrix time = ", calcTranformationMatrices(missile, missile,target, target,
                                                               seeker, seeker));

    checkEqual("Test Rb2c", (double*)Rb2c_cur_gt.data, m_Rb2c_cur, 3, 3);
    checkEqual("Test Ri2b_missile", (double*)Ri2b_missile_gt.data, m_Ri2b_missile_cur, 3, 3);
    checkEqual("Test Ri2b_target", (double*)Ri2b_target_gt.data, m_Ri2b_target, 3, 3);
    checkEqual("Test Re2i_missile", (double*)Re2i_missile_gt.data, m_Re2i_missile, 3, 3);
    checkEqual("Test Re2i_target", (double*)Re2i_target_gt.data, m_Re2i_target, 3, 3);

    printf("Calculate Transformation Matrices: DONE !!!\n");
    convertToImage(missile, target);
    GPS * h_ship_gps = (GPS*)malloc(m_ship.num_vertices * sizeof(GPS));
    double2* h_ship_imgPos = (double2*)malloc(m_ship.num_vertices * sizeof(double2));

    gpuErrChk(cudaMemcpy(h_ship_gps, m_ship.gps, m_ship.num_vertices * sizeof(GPS), cudaMemcpyDeviceToHost));
    gpuErrChk(cudaMemcpy(h_ship_imgPos, m_ship.imgPos, m_ship.num_vertices * sizeof(double2), cudaMemcpyDeviceToHost));

    writeTofile("../log/ship_gps.txt", (double*)h_ship_gps, m_ship.num_vertices, 3);
    writeTofile("../log/ship_imgPos.txt", (double*)h_ship_imgPos, m_ship.num_vertices, 2);

    free(h_ship_imgPos);
    free(h_ship_gps);

    testCalcShipData(missile, target);
    auto start = getMoment;
    for(int offset = 0; offset < m_width * m_height / m_batch_size; offset++)
//    for(int offset = 0; offset < 1; offset++)
    {
        //            printf("Rendering image part %d\n", offset);
        getExeTime("calcDistance Time = ", calcDistance(offset, missile));
        getExeTime("calcRadiance Time = ", calcRadiance(offset));
        getExeTime("calcRenderPartialImg Time = ", renderPartialImg(offset));
    }

    cv::Mat img(m_height, m_width, CV_8UC1);
    gpuErrChk(cudaMemcpy(img.data, m_renderedImg, m_width * m_height * sizeof(unsigned char), cudaMemcpyDeviceToHost));
    cv::imwrite("../img/" + std::string(std::to_string(m_current_img_id)) + ".jpg", img);
    auto end = getMoment;
    getTimeElapsed("Total Time = ", end, start);
    gpuErrChk(cudaMemcpy(img.data, m_maskImg, m_width * m_height * sizeof(unsigned char), cudaMemcpyDeviceToHost));
    cv::imwrite("../img/" + std::string(std::to_string(m_current_img_id)) + "_mask.jpg", img);

    //    cv::waitKey(2);
    gpuErrChk(cudaFree(missile));
    gpuErrChk(cudaFree(target));
    gpuErrChk(cudaFree(seeker));
}
