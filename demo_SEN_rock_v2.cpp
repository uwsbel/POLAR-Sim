// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Bo-Hsun Chen
// =============================================================================
//
// Chrono demonstration of a camera sensor.
// Generates a mesh object and rotates camera sensor around the mesh.
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualShape.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterGrayscale.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterCameraNoise.h"
#include "chrono_sensor/filters/ChFilterCameraExposure.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"

#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"
#include "chrono_thirdparty/rapidjson/istreamwrapper.h"
#include <iomanip>
#include <unordered_map>

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::sensor;
// using namespace rapidjson;

// -----------------------------------------------------------------------------
// Camera parameters
// -----------------------------------------------------------------------------

// Camera lens model, either PINHOLE or SPHERICAL
CameraLensModelType lens_model = CameraLensModelType::PINHOLE;
float update_rate = 0.4; // [Hz], update rate
unsigned int image_width = 1936; // [pixel], image width
unsigned int image_height = 1216; // [pixel], image height
float fov = (float)((44.0 + 1.0/60) * CH_C_PI / 180.0); // [rad], camera's horizontal field of view
float lag = .05f; // [sec], // lag between sensing and when data becomes accessible
int alias_factor = 1; // ????

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

size_t NumSimStep = 900; // number of simulation steps
double step_size = 1e-2; // simulation step size
float end_time = 20000.0f; // [sec], simulation end time
bool save = true; // save camera images or not
bool vis = false; // // render camera images or not
bool exposure_correct_switch = false;  // whether turn on exposure correction filter
bool DebugMode = true;

// copied from ChUtilsJSON.cpp
// read json file into a variable
void ReadFileJSON(const std::string& filename, rapidjson::Document& d) {
    std::ifstream ifs(filename);
    if (!ifs.good()) {
        GetLog() << "ERROR: Could not open JSON file: " << filename << "\n";
    }
    else {
        rapidjson::IStreamWrapper isw(ifs);
        d.ParseStream<rapidjson::ParseFlag::kParseCommentsFlag>(isw);
        if (d.IsNull()) {
            GetLog() << "ERROR: Invalid JSON file: " << filename << "\n";
        }
    }
}


int main(int argc, char* argv[]) {
    // ----------------- //
    // parameter setting //
    // ----------------- //
    if (argc < 4) {
        if (save == false && vis == true) { // test mode
            std::cout << "./demo_SEN_rock [BRDF name, ex: hapke, default] [simple/complex] test [terrain ID] [config index] [expsr time]\n";
        }
        else {
            std::cout << "./demo_SEN_rock [BRDF name, ex: hapke, default] [simple/complex] [terrain ID]\n";
        }
        exit(1);
    }

    GetLog() << "Copyright (c) 2022 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    GetLog() << "Alias Factor: " << alias_factor << "\n\n";

    std::string brdf_type = std::string(argv[1]);
    std::string expsr_type = std::string(argv[2]);
    std::string terrainID = std::string(argv[3]);
    
    size_t rock_num = 0;
    size_t ground_num = 1;
    double x_offset = -0.2;
    double z_offset = 0.;
    if (terrainID == "01") { // Terrain 1
        rock_num = 15;
        x_offset = 0.;
    }
    else if (terrainID == "02") { // Terrain 2
        rock_num = 16;
    }
    else if (terrainID == "03") { // Terrain 3
        rock_num = 28;
    }
    else if (terrainID == "04") { // Terrain 4
        rock_num = 27;
        x_offset = 0.;
        z_offset = -0.2;
    }
    else if (terrainID == "05") { // Terrain 5
        rock_num = 23;
    }
    else if (terrainID == "06") { // Terrain 6
        rock_num = 0;
    }
    else if (terrainID == "07") { // Terrain 7
        rock_num = 18;
    }
    else if (terrainID == "08") { // Terrain 8
        rock_num = 16;
    }
    else if (terrainID == "09") { // Terrain 9
        rock_num = 20;
    }
    else if (terrainID == "10") { // Terrain 10
        rock_num = 20;
    }
    else if (terrainID == "11") { // Terrain 11
        rock_num = 9;
        x_offset = 0.;
    }
    else if (terrainID == "12") { // Terrain 12
        rock_num = 23;
    }
    else if (terrainID == "test") { // test
        rock_num = 27;
        x_offset = 0.;
        z_offset = -0.2;
    }
    else {
        std::cout << "unknown terrain ID\n";
        exit(1);
    }

    // ---- data path ---- //
    const std::string out_folder = "SENSOR_OUTPUT/LunarProject/"; // output folder for saved images
    const std::string setting_table_path = GetChronoDataFile("robot/curiosity/rocks/setting_table.json");
    const std::string rock_mesh_dir = GetChronoDataFile("robot/curiosity/rocks/Terrain" + terrainID + "/");
    const std::string ground_mesh_dir = GetChronoDataFile("robot/curiosity/rocks/Terrain" + terrainID + "/");

    // ---- set class ID ---- //
    unsigned short rock_mat_classID = 65534; // red value
    unsigned short ground_mat_classID = 32768; // red value


    // ---- set instance ID ---- //
    unsigned short rock_mat_instanceID_range = 65535; // green value range
    unsigned short ground_mat_instanceID_range = 32768; // green value range

    // ---- create maps for positions of Sun and cameras ---- //
    std::unordered_map<char, std::unordered_map<char, ChVector<float>>> camera_posi_map{ // [m]
        {'A', {{'L', {3.541f, -0.303f, 1.346f}}, {'R', {3.532f, -0.002f, 1.344f}}}},
        {'B', {{'L', {5.500f, -0.303f, 1.345f}}, {'R', {5.500f, -0.002f, 1.344f}}}},
        {'C', {{'L', {0.316f, -3.540f, 1.346f}}, {'R', {0.611f, -3.479f, 1.344f}}}}
    };

    std::unordered_map<char, double> camera_yaw_map{ // [rad]
        {'A', CH_C_PI},
        {'B', CH_C_PI},
        {'C', 100 * (CH_C_PI / 180)}
    };

    std::unordered_map<char, ChVector<float>> rover_light_posi_map{ // [m]
        {'A', {3.54f, -0.15f, 1.34f}},
        {'B', {5.04f, -0.15f, 1.34f}},
        {'C', {0.46f, -3.51f, 1.34f}}
    };

    std::unordered_map<int, ChVector<float>> sun_posi_map{ // [m]
        {30, {3.594f, 1.422f, 0.583f + 0.22f}}, {180, {-3.544f, 0.475f, 0.556f + 0.22f}},
        {270, {-0.271f, -4.109f, 0.591f + 0.22f}}, {350, {4.026f, -0.896f, 0.551f + 0.22f}}
    };

    // ----------------- //
    // Create the system //
    // ----------------- //
    ChSystemNSC sys;
    if (DebugMode == true) std::cout << "system built" << std::endl;
    
    // ----------------- //
    // Create the ground //
    // ----------------- //
    std::shared_ptr<ChVisualMaterial> ground_mats[ground_num];
    std::shared_ptr<ChTriangleMeshShape> ground_meshs[ground_num];
    std::shared_ptr<ChBodyAuxRef> ground_bodies[ground_num];
    double terrain_scale_ratio = 1.0;
    std::string ground_mesh_path = "";
    printf("Loading grounds into system ...\n");
    for (size_t ground_idx = 0; ground_idx < ground_num; ++ground_idx) {
        // set up ground material
        ground_mats[ground_idx] = chrono_types::make_shared<ChVisualMaterial>();
        ground_mats[ground_idx]->SetAmbientColor({0.0, 0.0, 0.0});
        ground_mats[ground_idx]->SetDiffuseColor({0.7, 0.7, 0.7});
        ground_mats[ground_idx]->SetSpecularColor({1.0, 1.0, 1.0});
        ground_mats[ground_idx]->SetUseSpecularWorkflow(true);
        ground_mats[ground_idx]->SetRoughness(0.8f);
        ground_mats[ground_idx]->SetAnisotropy(1.f);
        ground_mats[ground_idx]->SetUseHapke((brdf_type == "hapke") ? true : false);
        
        // ground segment color: (1.0, 1.0 / 4 * (ground_idx + 1), 0.)
        ground_mats[ground_idx]->SetClassID(ground_mat_classID);
        ground_mats[ground_idx]->SetInstanceID((ground_mat_instanceID_range / ground_num) * (ground_idx + 1));

        // set up ground mesh
        ground_meshs[ground_idx] = chrono_types::make_shared<ChTriangleMeshShape>();
        // load mesh from obj file
        if (ground_num == 1) {
            ground_mesh_path = ground_mesh_dir + "terrain" + terrainID + "_ground.obj";
        }
        else {
            ground_mesh_path = ground_mesh_dir + "terrain" + terrainID + "_ground" + std::to_string(ground_idx + 1) + ".obj";
        }
        // printf("loading Terrain %zu mesh from %s ...\n", ground_idx + 1, ground_mesh_path.c_str());
        auto ground_mesh_loader = ChTriangleMeshConnected::CreateFromWavefrontFile(ground_mesh_path, false, false);
        ground_mesh_loader->Transform(ChVector<>(x_offset, 0, z_offset), ChMatrix33<>(terrain_scale_ratio));  // scale to a different size
        ground_mesh_loader->RepairDuplicateVertexes(1e-9); // if meshes are not watertight
        ground_meshs[ground_idx]->SetMesh(ground_mesh_loader);
        ground_meshs[ground_idx]->SetBackfaceCull(true);
        
        // set up grounds
        ground_bodies[ground_idx] = chrono_types::make_shared<ChBodyAuxRef>();
        ground_bodies[ground_idx]->AddVisualShape(ground_meshs[ground_idx]);
        ground_bodies[ground_idx]->GetVisualShape(0)->SetMaterial(0, ground_mats[ground_idx]);
        ground_bodies[ground_idx]->SetBodyFixed(true);
        ground_bodies[ground_idx]->SetPos({0., 0., 0}); // [m]
        sys.Add(ground_bodies[ground_idx]);
        printf("Ground %zu added to system\n", ground_idx + 1);
    }
    printf("Finished loading grounds into system\n");

    // options to generate high-resolution pictures
    // ---------------- //
    // Create all rocks //
    // ---------------- //
    std::shared_ptr<ChVisualMaterial> rock_mats[rock_num];
    std::shared_ptr<ChTriangleMeshShape> rock_meshes[rock_num];
    std::shared_ptr<ChBodyAuxRef> rock_bodies[rock_num];
    double rock_scale_ratio = 1.0;
    std::string rock_mesh_path = "";
    for (size_t rock_idx = 0; rock_idx < rock_num; ++rock_idx) {
        // set up rock material
        rock_mats[rock_idx] = chrono_types::make_shared<ChVisualMaterial>();
        rock_mats[rock_idx]->SetAmbientColor({0.0, 0.0, 0.0});
        rock_mats[rock_idx]->SetDiffuseColor({0.7, 0.7, 0.7});
        rock_mats[rock_idx]->SetSpecularColor({1.0, 1.0, 1.0});
        rock_mats[rock_idx]->SetUseSpecularWorkflow(true);
        rock_mats[rock_idx]->SetRoughness(0.8f);
        rock_mats[rock_idx]->SetAnisotropy(1.f);
        rock_mats[rock_idx]->SetUseHapke((brdf_type == "hapke") ? true : false);
        
        // rock segment color: (1.0, 1.0 / 15 * (rock_idx + 1), 0.)
        rock_mats[rock_idx]->SetClassID(rock_mat_classID);
        rock_mats[rock_idx]->SetInstanceID((rock_mat_instanceID_range / rock_num) * (rock_idx + 1));

        // set up rock mesh
        rock_meshes[rock_idx] = chrono_types::make_shared<ChTriangleMeshShape>();
        // load mesh from obj file
        if (terrainID == "test" && rock_num > 0) {
            rock_mesh_path = GetChronoDataFile("robot/curiosity/rocks/Terrain" + std::string(argv[4]) + "/terrain" + std::string(argv[4]) + "_rock" + std::to_string(rock_idx+1) + ".obj"); // test
        }
        else {
            rock_mesh_path = rock_mesh_dir + "terrain" + terrainID + "_rock" + std::to_string(rock_idx+1) + ".obj";
        }
        // printf("loading Rock %zu mesh from %s ...\n", rock_idx + 1, rock_mesh_path.c_str());
        auto rock_mesh_loader = ChTriangleMeshConnected::CreateFromWavefrontFile(rock_mesh_path, false, false);
        rock_mesh_loader->Transform(ChVector<>(x_offset, 0, z_offset), ChMatrix33<>(rock_scale_ratio));  // scale to a different size
        rock_mesh_loader->RepairDuplicateVertexes(1e-9); // if meshes are not watertight
        rock_meshes[rock_idx]->SetMesh(rock_mesh_loader);
        rock_meshes[rock_idx]->SetBackfaceCull(true);
        
        // set up rocks
        rock_bodies[rock_idx] = chrono_types::make_shared<ChBodyAuxRef>();
        rock_bodies[rock_idx]->AddVisualShape(rock_meshes[rock_idx]);
        rock_bodies[rock_idx]->GetVisualShape(0)->SetMaterial(0, rock_mats[rock_idx]);
        rock_bodies[rock_idx]->SetBodyFixed(true);
        rock_bodies[rock_idx]->SetPos({0., 0., 0.}); // [m]
        sys.Add(rock_bodies[rock_idx]);
        printf("Rock %zu added to system\n", rock_idx + 1);
    }
    

    // --------------------------------------- //
    // get setting table of all configurations //
    // --------------------------------------- //
    rapidjson::Document setting_table;
    ReadFileJSON(setting_table_path, setting_table);
    int num_setting = terrainID == "test" ? 1 : setting_table["length"].GetInt();
    // iterate over all settings to generate all synthetic images
    for (size_t setting_idx = 0; setting_idx < num_setting; ++setting_idx) {    
        if (terrainID == "test" && (vis == true && save == false)) { // test
            setting_idx = atoi(argv[5]);
        }
        printf("Idx: %zd\n", setting_idx);
        rapidjson::Value& setting_params = setting_table[std::to_string(setting_idx).c_str()];
        char camera_posi_idx = setting_params["camera_position"].GetString()[0];
        bool rover_light = setting_params["rover_lights"].GetInt();
        int sun_azimuth = setting_params["sun_azimuth"].GetInt();
        int exposure_time = setting_params["exposure_time"].GetInt();
        
        if (exposure_correct_switch == false && exposure_time != 32) {
            continue;
        }

        char camera_idx = setting_params["camera_index"].GetString()[0];

        // debug
        // printf("number of settings: %d\n", num_setting);
        // printf("camera position idx: %c\n", camera_posi_idx); // v
        // printf("rover light: %s\n", rover_light ? "ON" : "OFF"); // v
        // printf("sun azimuth: %d\n", sun_azimuth); // v
        // printf("exposure time: %d ms\n", exposure_time); // v
        // printf("camera idx: %c\n", camera_idx); // v

        // -------------- //
        // set background //
        // -------------- //
        auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
        Background b;
        b.mode = BackgroundMode::SOLID_COLOR;
        b.color_zenith = {0, 0, 0};
        manager->scene->SetBackground(b);

        // add sun light
        if (sun_azimuth > 0) {
            manager->scene->AddPointLight(sun_posi_map[sun_azimuth], // [m], position
                                        {1.0f, 1.0f, 1.0f}, // [1/1], color in RGB
                                        100.0f); // [m], max range of 99% attenuation
            // printf("sun light added\n");
        }

        // add rover light
        if (rover_light == true) {
            manager->scene->AddPointLight(rover_light_posi_map[camera_posi_idx], // [m], position
                                          {0.035f, 0.035f, 0.035f}, // [1/1], color in RGB
                                          100.0f); // [m], max range of 99% attenuation
            // printf("rover light added\n");
        }

        // -------------------------------------------------------
        // Create a camera and add it to the sensor manager
        // -------------------------------------------------------
        chrono::ChFrame<double> camera_pose(
            camera_posi_map[camera_posi_idx][camera_idx],
            Q_from_AngAxis(camera_yaw_map[camera_posi_idx], {0, 0, 1}) * Q_from_AngAxis(23 * (CH_C_PI / 180), {0, 1, 0})
        );
        auto cam = chrono_types::make_shared<ChCameraSensor>(ground_bodies[0],  // body that camera is attached to
                                                            // update_rate,   // update rate in Hz
                                                            1000.0f / float(exposure_time) - 0.01f, // update rate in Hz
                                                            camera_pose,  // offset pose
                                                            image_width,   // image width
                                                            image_height,  // image height
                                                            fov,           // camera's horizontal field of view
                                                            alias_factor,  // supersample factor for antialiasing
                                                            lens_model,    // FOV
                                                            true);         // use global illumination or not
        cam->SetName("Global Illum Camera");
        
        cam->SetLag(lag);
        // cam->SetUpdateRate(1000.0f / float(exposure_time)); // [Hz]
        
        cam->SetCollectionWindow(float(exposure_time) / 1000.0f); // [sec]
        if (vis)
            cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Global Illumination"));
        
        if (save) {
            std::string sun_str = (sun_azimuth > 0) ? ("/Sun_" + std::to_string(sun_azimuth)) : "/NoSun";
            std::ostringstream ostr;
            ostr << std::setfill('0') << std::setw(4) << exposure_time;
            std::string exposure_time_str = ostr.str();
            std::string out_dir = out_folder + "Pos" + camera_posi_idx + "_L" + (rover_light ? "on" : "off") \
                                +  sun_str + "/Cam" + camera_idx + "_" + exposure_time_str + "/";
            cam->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir));
        
        }
        manager->AddSensor(cam);
        
        // --------------- //
        // Simulate system //
        // --------------- //
        float ch_time = 0.0;
        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();    
        if (vis == true && save == false) {
            while (ch_time < end_time) { // test and viz
                // Update sensor manager, will render/save/filter automatically
                manager->Update();

                // Perform step of dynamics
                sys.DoStepDynamics(step_size);

                // Get the current time of the simulation
                ch_time = (float)sys.GetChTime();
            }
        }
        else {
            for (size_t sim_step = 0; sim_step < NumSimStep / (2048 / exposure_time); ++sim_step) {    
                // Update sensor manager, will render/save/filter automatically
                manager->Update();

                // Perform step of dynamics
                sys.DoStepDynamics(step_size);

                // Get the current time of the simulation
                ch_time = (float)sys.GetChTime();
            }
        }
        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        std::cout << "Setting " <<  setting_idx << ": simulation time " << ch_time << " sec, wall time " << wall_time.count() << " sec.\n\n";
    }

    return 0;
}