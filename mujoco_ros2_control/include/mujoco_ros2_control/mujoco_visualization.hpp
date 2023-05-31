#ifndef MUJOCO_ROS2_CONTROL_MUJOCO_VISUALIZATION_HPP
#define MUJOCO_ROS2_CONTROL_MUJOCO_VISUALIZATION_HPP

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
namespace mujoco_visualization {

    class MujocoVisualization {
    public:
        void init(mjModel_ *model, mjData_ *data);
        void update();
        void terminate();

        static MujocoVisualization& getInstance()
        {
            static MujocoVisualization instance;
            return instance;
        }

    private:
        MujocoVisualization(void){};
        MujocoVisualization(MujocoVisualization const&);
    protected:
        // MuJoCo data structures
        mjModel* m = NULL;                  // MuJoCo model
        mjData* d = NULL;                   // MuJoCo data
        mjvCamera cam;                      // abstract camera
        mjvOption opt;                      // visualization options
        mjvScene scn;                       // abstract scene
        mjrContext con;                     // custom GPU context
        GLFWwindow* window = NULL;

        // mouse interaction
        bool button_left = false;
        bool button_middle = false;
        bool button_right =  false;
        double lastx = 0;
        double lasty = 0;

        static void scroll_cb(GLFWwindow *window, double xoffset, double yoffset);
        void scroll(GLFWwindow *window, double xoffset, double yoffset);

        static void mouse_move_cb(GLFWwindow *window, double xpos, double ypos);
        void mouse_move(GLFWwindow *window, double xpos, double ypos);

        static void mouse_button_cb(GLFWwindow *window, int button, int act, int mods);
        void mouse_button(GLFWwindow *window, int button, int act, int mods);

        static void keyboard_cb(GLFWwindow *window, int key, int scancode, int act, int mods);
        void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods);

    };

} // mujoco_visualization

#endif //MUJOCO_ROS2_CONTROL_MUJOCO_VISUALIZATION_HPP
