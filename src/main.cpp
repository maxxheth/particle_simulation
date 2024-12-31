#include <ros/ros.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <GL/glut.h>
#include <chrono>
#include <iostream>
#include <string>

#include "particle_simulation/particle.hpp"
#include "particle_simulation/particle_manager.hpp"

#define MIN_GEN_DT 0.01
#define FRAME_DT 0.01

// 마우스 클릭 상태와 위치를 저장할 변수
bool b_mouse_pressed_ = false;
double d_mouse_x_ = 0.0;
double d_mouse_y_ = 0.0;

double d_window_width_ = 800.0;
double d_window_height_ = 800.0;

double d_last_gen_time_ = 0.0;

ParticleManager particle_manager_;

// window 좌표계는 좌상단이 0,0
// OpenGL 좌표계는 좌하단이 0,0

// Convert window coordinates to OpenGL coordinates
std::pair<double, double> WindowToOpenGLCoords(const double& d_window_x, const double& d_window_y) {
    double d_opengl_x = (d_window_x / d_window_width_) * 2.0 - 1.0;
    double d_opengl_y = 1.0 - (d_window_y / d_window_height_) * 2.0;
    return {d_opengl_x, d_opengl_y};
}

// Convert OpenGL coordinates to window coordinates
std::pair<double, double> OpenGLToWindowCoords(const double& d_opengl_x, const double& d_opengl_y) {
    double d_window_x = (d_opengl_x + 1.0) / 2.0 * d_window_width_;
    double d_window_y = (1.0 - d_opengl_y) / 2.0 * d_window_height_;
    return {d_window_x, d_window_y};
}

// Convert window coordinates to physics coordinates
std::pair<double, double> WindowToPhysicsCoords(const double& d_window_x, const double& d_window_y) {
    double d_physics_x = d_window_x;
    double d_physics_y = d_window_height_ - d_window_y;
    return {d_physics_x, d_physics_y};
}

// Convert physics coordinates to window coordinates
std::pair<double, double> PhysicsToWindowCoords(const double& d_physics_x, const double& d_physics_y) {
    double d_window_x = d_physics_x;
    double d_window_y = d_window_height_ - d_physics_y;
    return {d_window_x, d_window_y};
}

void AddParticleMouse() {
    double current_time = ros::Time::now().toSec();
    if (current_time - d_last_gen_time_ > MIN_GEN_DT) {
        std::pair<double, double> d_physics_coords = WindowToPhysicsCoords(d_mouse_x_, d_mouse_y_);
        // particle_manager_.AddParticle(d_physics_coords.first, d_physics_coords.second, 0.0, 0.0, 0.0, -GRAVITY, particle_manager_.GetParticleCount());

        for(int i = 0; i < 10; i++) {
            particle_manager_.AddParticle(d_physics_coords.first - 10.0 + i * 2.0, d_physics_coords.second, 0.0, 0.0, 0.0, -GRAVITY, particle_manager_.GetParticleCount());
        }
        d_last_gen_time_ = current_time;
    }
}

// 마우스 버튼 콜백 함수
void MouseButtonCallback(GLFWwindow* p_window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS) {
            b_mouse_pressed_ = true;
            // std::cout << "Mouse Pressed" << std::endl;

            // Update mouse position when clicked
            double xpos, ypos;
            glfwGetCursorPos(p_window, &xpos, &ypos);
            d_mouse_x_ = xpos;
            d_mouse_y_ = ypos;

            AddParticleMouse();

        } else if (action == GLFW_RELEASE) {
            b_mouse_pressed_ = false;
        }
    }
}

// 마우스 위치 콜백 함수
void CursorPositionCallback(GLFWwindow* p_window, double xpos, double ypos) {
    if (b_mouse_pressed_) {
        d_mouse_x_ = xpos;
        d_mouse_y_ = ypos;

        AddParticleMouse();
    }
}

// 원 그리기 함수
void DrawCircle(double x, double y, double radius, float r = 1.0, float g = 1.0, float b = 1.0) {
    int i_segments = 6;
    glColor3f(r, g, b);
    glBegin(GL_TRIANGLE_FAN);
    glVertex2f(x, y);
    for (int i = 0; i <= i_segments; i++) {
        double angle = 2.0 * M_PI * i / i_segments;
        double dx = radius * cos(angle);
        double dy = radius * sin(angle);
        glVertex2f(x + dx, y + dy);
    }
    glEnd(); 
}

// Function to render text on the screen
void RenderText(float x, float y, const std::string& text) {
    glRasterPos2f(x, y);
    for (char c : text) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, c);
    }
}

int main(int argc, char** argv) {
    // Initialize GLUT
    glutInit(&argc, argv);

    // Initialize ROS
    ros::init(argc, argv, "particle_simulation");
    ros::NodeHandle nh;

    particle_manager_.SetBounds(0, d_window_width_, 0, d_window_height_);

    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "GLFW Initialize Failed" << std::endl;
        return -1;
    }

    // Set OpenGL version (2.1)
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);

    // Create window
    GLFWwindow* p_window = glfwCreateWindow(d_window_width_, d_window_height_, "Particle Simulation", nullptr, nullptr);
    if (!p_window) {
        std::cerr << "Window Create Failed" << std::endl;
        glfwTerminate();
        return -1;
    }

    // Set OpenGL context
    glfwMakeContextCurrent(p_window);

    // Initialize GLEW
    if (glewInit() != GLEW_OK) {
        std::cerr << "GLEW Initialize Failed" << std::endl;
        return -1;
    }

    // Enable blending
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Set mouse callback functions
    glfwSetMouseButtonCallback(p_window, MouseButtonCallback);
    glfwSetCursorPosCallback(p_window, CursorPositionCallback);

    // Main loop
    while (ros::ok() && !glfwWindowShouldClose(p_window)) {

        // Measure time taken by UpdateParticles
        auto start_time = std::chrono::high_resolution_clock::now();
        

        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        // 마우스 클릭 시 원 그리기
        if (b_mouse_pressed_) {
            auto [d_opengl_x, d_opengl_y] = WindowToOpenGLCoords(d_mouse_x_, d_mouse_y_);
            DrawCircle(d_opengl_x, d_opengl_y, 0.05); // 상대적인 반지름
        }

        // Measure time taken by UpdateParticles
        auto update_start_time = std::chrono::high_resolution_clock::now();
        
        // Update particles
        particle_manager_.UpdateParticles(FRAME_DT); // Assuming 60 FPS
        
        auto update_end_time = std::chrono::high_resolution_clock::now();
        auto update_duration = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(update_end_time - update_start_time).count();

        auto draw_start_time = std::chrono::high_resolution_clock::now();

        auto particles = particle_manager_.GetParticles();
        for (const auto& particle : particles) {
            auto [d_window_x, d_window_y] = PhysicsToWindowCoords(particle.pos.x(), particle.pos.y());
            auto [d_opengl_x, d_opengl_y] = WindowToOpenGLCoords(d_window_x, d_window_y);
            // Calculate color based on velocity
            float f_r = 0.0f, f_g = 0.0f, f_b = 1.0f; // Default to blue
            double d_velocity = particle.vel.norm(); // Assuming velocity is a vector with a norm() method

            if (d_velocity >= 200.0) {
                f_r = 1.0f; // Red
                f_g = 0.0f;
                f_b = 0.0f;
            } else if (d_velocity > 0.0) {
                // Interpolate between blue and red based on velocity
                f_r = static_cast<float>(d_velocity / 200.0);
                f_b = 1.0f - f_r;
            }

            DrawCircle(d_opengl_x, d_opengl_y, PARTICLE_RADIUS/d_window_width_ * 2.0, f_r, f_g, f_b); // Draw each particle with color based on velocity
        }

        auto draw_end_time = std::chrono::high_resolution_clock::now();
        auto draw_duration = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(draw_end_time - draw_start_time).count();

        // Get the total number of particles
        int i_particle_count = particle_manager_.GetParticleCount(); // Assuming this method exists

        glColor3f(1.0f, 1.0f, 1.0f);
        // Render the particle count on the window
        std::string particle_count_text = "Particle Count: " + std::to_string(i_particle_count);
        RenderText(-0.9f, 0.95f, particle_count_text); // Position the text slightly above the duration text
        
        // Render the duration on the window
        std::string duration_text = "UpdateParticles took " + std::to_string(update_duration) + " ms";
        RenderText(-0.9f, 0.9f, duration_text); // Position the text at the top-left corner

        std::string draw_duration_text = "DrawParticles took " + std::to_string(draw_duration) + " ms";
        RenderText(-0.9f, 0.85f, draw_duration_text); // Position the text at the top-left corner

        // Swap the front and back buffers to display the rendered image
        glfwSwapBuffers(p_window);
        
        // Poll for and process events (e.g., keyboard, mouse)
        glfwPollEvents();

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end_time - start_time).count();
        std::cout << "Total time: " << duration << " ms" << std::endl;

    }

    // Clean up
    glfwTerminate();
    return 0;
}
