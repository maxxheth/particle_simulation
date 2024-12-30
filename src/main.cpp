#include <ros/ros.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <iostream>

int main(int argc, char** argv) {
    // ROS 노드 초기화
    ros::init(argc, argv, "particle_simulation");
    ros::NodeHandle nh;

    // GLFW 초기화
    if (!glfwInit()) {
        std::cerr << "GLFW 초기화 실패" << std::endl;
        return -1;
    }

    // OpenGL 버전 설정 (3.3 사용)
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // 윈도우 생성
    GLFWwindow* p_window = glfwCreateWindow(800, 600, "Particle Simulation", nullptr, nullptr);
    if (!p_window) {
        std::cerr << "윈도우 생성 실패" << std::endl;
        glfwTerminate();
        return -1;
    }

    // OpenGL 컨텍스트 설정
    glfwMakeContextCurrent(p_window);

    // GLEW 초기화
    if (glewInit() != GLEW_OK) {
        std::cerr << "GLEW 초기화 실패" << std::endl;
        return -1;
    }

    // 메인 루프
    while (ros::ok() && !glfwWindowShouldClose(p_window)) {
        // 화면 클리어
        glClear(GL_COLOR_BUFFER_BIT);

        // 여기에 렌더링 코드 추가

        // 버퍼 스왑
        glfwSwapBuffers(p_window);
        glfwPollEvents();

        // ROS 콜백 처리
        ros::spinOnce();
    }

    // 정리
    glfwTerminate();
    return 0;
}
