#include "highway.h"

int main(int argc, char** argv)
{
    // Suppress unused-parameter warnings.
    (void)argc;
    (void)argv;

    // Create and configure the 3D viewer.
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();

    // Set camera position and orientation.
    const float x_pos = 0.0f;
    viewer->setCameraPosition(x_pos - 26.0f, 0.0f, 15.0f,
                              x_pos + 25.0f, 0.0f, 0.0f,
                              0.0f, 0.0f, 1.0f);

    // Create the highway simulation environment.
    Highway highway(viewer);

    // Simulation settings.
    const int frames_per_sec = 30;
    const int simulation_duration_sec = 10;
    const int total_frames = frames_per_sec * simulation_duration_sec;

    int frame_count = 0;
    int time_us = 0;

    const double ego_velocity = 25.0;

    while (frame_count < total_frames)
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        highway.stepHighway(ego_velocity, time_us, frames_per_sec, viewer);
        viewer->spinOnce(1000 / frames_per_sec);

        ++frame_count;
        time_us = 1000000 * frame_count / frames_per_sec;
    }

    return 0;
}
